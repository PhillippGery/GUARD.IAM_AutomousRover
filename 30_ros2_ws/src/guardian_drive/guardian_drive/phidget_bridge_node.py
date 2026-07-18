# MIT License
# GUARDIAN — GUARD.IAM
# Node: phidget_bridge_node
# Owner: Vedant
# Purpose: low-level drive bridge. Subscribes to /wheel_rpm (the four wheel
#          speed targets produced by mecanum_kinematics_node) and drives four
#          Phidget DCC1120 BLDC controllers over VINT/USB using the built-in
#          closed-loop velocity controller.
#
# This node REPLACES serial_bridge_node for Phidget hardware. It honors the
# exact same /wheel_rpm contract, so nothing upstream (kinematics, teleop,
# Nav2) changes. This v1 is command-only — no /odom yet — so you can bring the
# wheels up first and add odometry once they spin correctly.
#
# ── What you must fill in ────────────────────────────────────────────────────
#   wheel_serials : the serial number of each DCC1120, in order [FL, FR, BL, BR].
#                   You already know FL = 782265. Plug each of the other three in
#                   and read its serial (your attach script or the Control Panel),
#                   then put them here (or override via ROS params / a YAML).
#                   A serial left as 0 means "not configured yet" — that wheel is
#                   skipped, so you can test one wheel at a time.
#
# ── Two things to calibrate on the bench (notes inline below) ─────────────────
#   1. rescale_factor : makes the controller command in real wheel RPM. Verify
#                       empirically (see comment at RESCALE_FACTOR).
#   2. wheel_directions : per-wheel sign so +RPM actually drives that corner the
#                         intended way. Determine one wheel at a time.

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from Phidget22.Devices.MotorVelocityController import MotorVelocityController
from Phidget22.PhidgetException import PhidgetException

# Wheel index convention — matches /wheel_rpm order everywhere in this file.
FL, FR, BL, BR = 0, 1, 2, 3
WHEEL_NAMES = ['FL', 'FR', 'BL', 'BR']


class PhidgetBridgeNode(Node):
    def __init__(self):
        super().__init__('phidget_bridge_node')

        # ── Parameters ────────────────────────────────────────────────────────
        # Serial numbers per wheel [FL, FR, BL, BR]. 0 = skip (not wired yet).
        self.declare_parameter('wheel_serials', [782265, 0, 0, 0])

        # Per-wheel sign. The kinematics already signs for mecanum handedness,
        # but physical rotation depends on mounting + phase wiring, so each wheel
        # may need a flip. Determine empirically, one wheel at a time.
        self.declare_parameter('wheel_directions', [1, -1, 1, -1])

        # RESCALE_FACTOR: the DCC1120's native velocity unit for a hall-fed BLDC
        # is commutations/sec. This motor: 12 commutations/motor-rev (4 poles x
        # 3 phases) x 22.667 gearbox = 272 commutations/wheel-rev. So 60/272 makes
        # the controller command directly in WHEEL RPM, matching /wheel_rpm.
        #   VERIFY ONCE: command a known value, measure real wheel RPM. If it's
        #   off by exactly 272/60 (or its reciprocal), flip this factor — Phidget's
        #   multiply/divide convention is the one thing worth confirming by hand.
        self.declare_parameter('rescale_factor', 60.0 / 272.0)

        # Controller current limit (A). DCC1120 floor is 5A. Keep modest on bench.
        self.declare_parameter('current_limit', 10.0)

        # Velocity ramp, in wheel RPM per second (after rescale). Gentle = safer.
        self.declare_parameter('acceleration', 200.0)

        # Safety: if no /wheel_rpm arrives within this window, zero all wheels.
        self.declare_parameter('command_timeout_sec', 0.3)

        # Clamp — motor's rated output is ~170 RPM; refuse to command past this.
        self.declare_parameter('max_wheel_rpm', 170.0)

        # Each DCC1120 exposes its velocity controller on channel 0.
        self.declare_parameter('channel', 0)

        # Seconds to wait for each controller to attach on startup.
        self.declare_parameter('attach_timeout_ms', 5000)

        self._serials = list(self.get_parameter('wheel_serials').value)
        self._dirs = list(self.get_parameter('wheel_directions').value)
        self._rescale = float(self.get_parameter('rescale_factor').value)
        self._current_limit = float(self.get_parameter('current_limit').value)
        self._accel = float(self.get_parameter('acceleration').value)
        self._cmd_timeout = float(self.get_parameter('command_timeout_sec').value)
        self._max_rpm = float(self.get_parameter('max_wheel_rpm').value)
        self._channel = int(self.get_parameter('channel').value)
        self._attach_timeout = int(self.get_parameter('attach_timeout_ms').value)

        # ── Open the four controllers ─────────────────────────────────────────
        # self._motors[i] is a MotorVelocityController or None (skipped/failed).
        self._motors = [None, None, None, None]
        for i in range(4):
            self._motors[i] = self._open_motor(i, self._serials[i])

        attached = [WHEEL_NAMES[i] for i in range(4) if self._motors[i] is not None]
        if not attached:
            self.get_logger().error(
                'No controllers attached. Set wheel_serials and check power/VINT. '
                'Node is running but will not drive anything.')
        else:
            self.get_logger().info(f'Controllers attached: {attached}')

        # ── ROS interface ─────────────────────────────────────────────────────
        self._last_cmd_time = self.get_clock().now()
        self._sub = self.create_subscription(
            Float32MultiArray, '/wheel_rpm', self._wheel_rpm_callback, 10)

        # Watchdog — zeroes the wheels if commands stop arriving.
        self.create_timer(0.05, self._watchdog)  # 20 Hz

        self.get_logger().info('phidget_bridge_node started')

    def _open_motor(self, index, serial):
        name = WHEEL_NAMES[index]
        if not serial:  # 0 or None → not configured yet
            self.get_logger().warn(f'{name}: serial not set — skipping this wheel')
            return None

        ch = MotorVelocityController()
        ch.setDeviceSerialNumber(int(serial))
        ch.setChannel(self._channel)
        try:
            ch.openWaitForAttachment(self._attach_timeout)
        except PhidgetException as e:
            self.get_logger().error(
                f'{name}: controller {serial} failed to attach ({e.details}). '
                'Skipping — check its VINT cable and power.')
            return None

        # Configure once attached.
        try:
            ch.setRescaleFactor(self._rescale)
            ch.setCurrentLimit(self._current_limit)
            ch.setAcceleration(self._accel)
            ch.setTargetVelocity(0.0)
            ch.setEngaged(True)  # REQUIRED — otherwise it freewheels and ignores you
        except PhidgetException as e:
            self.get_logger().error(f'{name}: config failed ({e.details})')
            return None

        self.get_logger().info(f'{name}: controller {serial} ready')
        return ch

    def _wheel_rpm_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn('wheel_rpm: expected 4 values [FL,FR,BL,BR]')
            return
        self._last_cmd_time = self.get_clock().now()
        for i in range(4):
            self._set_wheel(i, float(msg.data[i]))

    def _set_wheel(self, index, rpm):
        motor = self._motors[index]
        if motor is None:
            return
        # Clamp, then apply per-wheel sign.
        rpm = max(-self._max_rpm, min(self._max_rpm, rpm))
        target = self._dirs[index] * rpm
        try:
            motor.setTargetVelocity(target)
        except PhidgetException as e:
            self.get_logger().error(f'{WHEEL_NAMES[index]}: set velocity failed ({e.details})')

    def _watchdog(self):
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if age > self._cmd_timeout:
            # Stale command stream — stop all wheels (don't spam if already stopped).
            for i in range(4):
                self._set_wheel(i, 0.0)

    def shutdown(self):
        # Stop and release the motors cleanly.
        for i in range(4):
            motor = self._motors[i]
            if motor is None:
                continue
            try:
                motor.setTargetVelocity(0.0)
                motor.setEngaged(False)
                motor.close()
            except PhidgetException:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = PhidgetBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()