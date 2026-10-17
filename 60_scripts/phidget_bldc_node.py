#!/usr/bin/env python3
"""
GUARDIAN — phidget_bldc_node
Owner: Vedant

Custom outer-loop velocity PID running on top of BLDCMotor channels,
replacing MotorVelocityController's onboard closed loop. Built specifically
to get real active braking (BrakingEnabled=True), which is not available on
MotorVelocityController — confirmed via standalone bench test:

    Coast + BrakingEnabled=False -> braking_strength stays 0.0 at rest
    Coast + BrakingEnabled=True  -> braking_strength pins at 1.0 at rest
    DriveMode=FORCED (alone or combined with BrakingEnabled) -> braking_
        strength stayed 0.0, contrary to docs. Do NOT use Forced mode on
        this hardware — confirmed non-functional across two bench runs.

ARCHITECTURE — deliberately NOT a switching state machine:
    One BLDCMotor channel per wheel, opened ONCE at startup, held open for
    the entire session. BrakingEnabled=True is set once and never touched
    again. There is no "hold mode" vs "drive mode" — the same PID loop
    runs continuously; when target velocity is 0, the loop naturally
    commands duty=0, and the firmware's own BrakingEnabled logic applies
    active braking automatically. No close/reopen, no channel-exclusivity
    cost, no detach-handler race — the exact class of problems that sank
    the earlier MotorPositionController switching attempt does not apply
    here, because we never leave the channel.

FEEDBACK: BLDCMotor.getVelocity() is DUTY CYCLE, not RPM — useless as PID
    feedback. Real measured velocity is derived by differentiating
    getPosition() (raw hall-commutation counts) over time in the control
    loop timer, same as MotorVelocityController's firmware did internally,
    now done here instead.

UNVERIFIED, NEEDS BENCH CONFIRMATION:
    - position_rescale_factor: guessed as 1/272 (counts per wheel
      revolution, same physical constant as the velocity rescale, but
      WITHOUT the x60 used for velocity's per-minute units). Verify by
      commanding a fixed duty cycle, timing real wheel rotations by hand,
      comparing to what measured_rpm reports.
    - kp_custom / ki_custom starting values below are CONSERVATIVE GUESSES,
      not derived from the old onboard Kp=0.0030/Ki=0.0045 — those were
      calibrated to Phidget's internal (undocumented) units and do not
      transfer. This loop's gains are duty-cycle-per-RPM-error, a
      completely different scale. Expect to re-tune from scratch using the
      same empirical method as before (watch measured vs target RPM
      converge cleanly, no oscillation).
    - Kd is intentionally NOT implemented. Already confirmed on this
      hardware (MotorVelocityController bench testing) that derivative
      gain amplifies noisy hall-based velocity feedback into real
      chattering at a fairly small value. A hand-rolled loop over
      VINT/USB is slower and jitterier than the firmware's native loop —
      MORE exposed to that noise, not less. P+I only, deliberately.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from Phidget22.Devices.BLDCMotor import BLDCMotor
from Phidget22.PhidgetException import PhidgetException

FL, FR, BL, BR = 0, 1, 2, 3
WHEEL_NAMES = ['FL', 'FR', 'BL', 'BR']


class PhidgetBLDCNode(Node):
    def __init__(self):
        super().__init__('phidget_bldc_node')

        # ── Addressing — same convention as phidget_bridge_node ────────────
        self.declare_parameter('hub_serial', 782265)
        self.declare_parameter('wheel_hub_ports', [0, 5, 1, 4])
        self.declare_parameter('channel', 0)
        self.declare_parameter('attach_timeout_ms', 3000)
        self.declare_parameter('wheel_directions', [1, -1, 1, -1])

        # UNVERIFIED — see module docstring. Bench-check before trusting.
        self.declare_parameter('position_rescale_factor', 1.0 / 272.0)

        # ── Custom PID (P+I only — see docstring on why no D) ──────────────
        # CONSERVATIVE STARTING GUESSES — expect to re-tune empirically.
        self.declare_parameter('kp_custom', 0.012)
        self.declare_parameter('ki_custom', 0.01)
        self.declare_parameter('integral_limit', 15.0)  # anti-windup clamp —
            # confirmed on bench: 0.5 let a single brief transient swing the
            # integral to its opposite rail and get stuck there, producing
            # a permanent phantom duty command at rest. 0.1 is a much
            # tighter cap; may still need tuning.
        self.declare_parameter('control_rate_hz', 100.0)

        # ── Motor config ─────────────────────────────────────────────────
        self.declare_parameter('current_limit', 15.0)
        # UNVERIFIED-UNITS BUG, CAUGHT ON HARDWARE: BLDCMotor's Acceleration
        # is duty-cycle-per-second, NOT wheel-RPM/s like on
        # MotorVelocityController. 3000.0 (correct for the old node) is
        # wildly out of range here — confirmed firmware range is 0.1-1000.
        # 100.0 mirrors Phidget's own doc example ("Acceleration=100" ->
        # full duty in 0.01s) as a reasonably fast, safely in-range start.
        # Real Min/MaxAcceleration logged per-wheel at attach time below —
        # check that log before assuming 100.0 is the right value to tune
        # around.
        self.declare_parameter('acceleration', 100.0)
        self.declare_parameter('max_wheel_rpm', 170.0)
        self.declare_parameter('command_timeout_sec', 0.3)

        # ── Odometry — same as phidget_bridge_node ──────────────────────
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_rate_hz', 50.0)
        self.declare_parameter('wheel_radius', 0.0775)
        self.declare_parameter('wheel_base_length', 0.313588)
        self.declare_parameter('wheel_base_width', 0.390755)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        p = self.get_parameter
        self._hub_serial = int(p('hub_serial').value)
        self._ports = list(p('wheel_hub_ports').value)
        self._channel = int(p('channel').value)
        self._attach_timeout = int(p('attach_timeout_ms').value)
        self._dirs = list(p('wheel_directions').value)
        self._pos_rescale = float(p('position_rescale_factor').value)

        self._kp = float(p('kp_custom').value)
        self._ki = float(p('ki_custom').value)
        self._integral_limit = float(p('integral_limit').value)
        control_rate = float(p('control_rate_hz').value)

        self._current_limit = float(p('current_limit').value)
        self._accel = float(p('acceleration').value)
        self._max_rpm = float(p('max_wheel_rpm').value)
        self._cmd_timeout = float(p('command_timeout_sec').value)

        self._publish_odom = bool(p('publish_odom').value)
        self._publish_tf = bool(p('publish_tf').value)
        self._r = float(p('wheel_radius').value)
        self._L = float(p('wheel_base_length').value) / 2.0
        self._W = float(p('wheel_base_width').value) / 2.0
        self._odom_frame = p('odom_frame').value
        self._base_frame = p('base_frame').value

        # ── Per-wheel PID/feedback state ────────────────────────────────
        self._target_rpm = [0.0, 0.0, 0.0, 0.0]
        self._measured_rpm = [0.0, 0.0, 0.0, 0.0]
        self._integral = [0.0, 0.0, 0.0, 0.0]
        self._last_position = [None, None, None, None]
        self._last_control_time = self.get_clock().now()

        # ── Startup scan ─────────────────────────────────────────────────
        self._reattach_logged = [False] * 4
        self._motors = [self._open_motor(i) for i in range(4)]

        live = [WHEEL_NAMES[i] for i in range(4) if self._motors[i] is not None]
        missing = [WHEEL_NAMES[i] for i in range(4)
                   if self._ports[i] >= 0 and self._motors[i] is None]
        self.get_logger().info(f'Wheels driving (BLDC/custom PID): {live or "NONE"}')
        if missing:
            self.get_logger().error(f'Wheels CONFIGURED BUT NOT FOUND: {missing}.')

        # ── Pose state ───────────────────────────────────────────────────
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        # ── ROS interface ────────────────────────────────────────────────
        self._last_cmd_time = self.get_clock().now()
        self.create_subscription(
            Float32MultiArray, '/wheel_rpm', self._wheel_rpm_callback, 10)

        self._odom_pub = None
        self._tf_broadcaster = None
        if self._publish_odom:
            self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
            if self._publish_tf:
                self._tf_broadcaster = TransformBroadcaster(self)
            self._last_odom_time = self.get_clock().now()
            self.create_timer(1.0 / float(p('odom_rate_hz').value), self._odom_timer)

        # The actual PID loop — runs continuously regardless of target,
        # this IS the "braking" mechanism, not a separate mode.
        self.create_timer(1.0 / control_rate, self._control_timer)
        self.create_timer(0.05, self._watchdog)  # 20 Hz stale-command check
        self.create_timer(2.0, self._reattach_timer)

        self.get_logger().info(
            f'phidget_bldc_node started — custom PID @ {control_rate}Hz, '
            f'kp={self._kp} ki={self._ki}, BrakingEnabled=True on all wheels')

    # ── Setup ────────────────────────────────────────────────────────────
    def _open_motor(self, index, quiet=False):
        name = WHEEL_NAMES[index]
        port = self._ports[index]
        if port is None or port < 0:
            return None

        ch = BLDCMotor()
        if self._hub_serial:
            ch.setDeviceSerialNumber(self._hub_serial)
        ch.setHubPort(int(port))
        ch.setChannel(self._channel)

        try:
            ch.openWaitForAttachment(self._attach_timeout)
        except PhidgetException as e:
            if not quiet:
                self.get_logger().error(
                    f'{name}: no BLDC controller on hub port {port} ({e.details})')
            # REAL BUG, likely root cause of the rotating hardware faults
            # and segfault: this never closed `ch` on failure. Combined
            # with the reattach timer calling this every 2s at a 10ms
            # timeout — guaranteed to fail every time, since real attach
            # takes ~250-450ms per earlier logs — this was leaking a
            # driver-level handle every single cycle. Close it explicitly.
            try:
                ch.close()
            except PhidgetException:
                pass
            return None

        try:
            try:
                accel_min, accel_max = ch.getMinAcceleration(), ch.getMaxAcceleration()
                if not (accel_min <= self._accel <= accel_max):
                    self.get_logger().warn(
                        f'{name}: configured acceleration={self._accel} is '
                        f'OUTSIDE this device\'s real range '
                        f'[{accel_min}, {accel_max}] — will fail below.')
            except PhidgetException:
                pass
            ch.setCurrentLimit(self._current_limit)
            ch.setAcceleration(self._accel)
            ch.setRescaleFactor(self._pos_rescale)
            # Likely root cause of huge single-sample measured_rpm spikes
            # seen on the bench (e.g. -371 RPM from a stationary wheel):
            # if the device's own internal update rate is slower than our
            # 100Hz control loop poll rate, most polls read a stale value
            # (correctly giving delta=0), but when a real update lands, the
            # FULL interval's worth of movement gets divided by our much
            # smaller assumed dt, producing a burst reading far beyond
            # anything physical. Sync device update rate to our polling.
            try:
                min_interval = ch.getMinDataInterval()
                ch.setDataInterval(min_interval)
                self.get_logger().info(
                    f'{name}: DataInterval set to device minimum '
                    f'({min_interval}ms)')
            except PhidgetException as e:
                self.get_logger().warn(
                    f'{name}: could not set DataInterval ({e.details})')
            # Confirmed on standalone bench test: this is THE property that
            # produces real active braking (braking_strength pins at 1.0 at
            # rest). Set once, never touched again — no switching needed.
            ch.setBrakingEnabled(True)
            ch.setTargetVelocity(0.0)
            ch.setOnDetachHandler(
                lambda _ch, i=index, n=name: self._on_motor_detached(i, n))
        except PhidgetException as e:
            self.get_logger().error(f'{name}: config failed ({e.details})')
            try:
                ch.close()
            except PhidgetException:
                pass
            return None

        try:
            self._last_position[index] = ch.getPosition()
        except PhidgetException:
            # Was 0.0 — a real bug: if the initial read fails and later
            # succeeds, delta_rev = (real_position - 0.0) produces a huge
            # false velocity spike on the very next control tick. None
            # correctly triggers the existing "skip this tick, just record
            # position" path used everywhere else in the control loop.
            self._last_position[index] = None

        self.get_logger().info(
            f'{name}: ready on hub port {port} (BrakingEnabled='
            f'{ch.getBrakingEnabled()})')
        return ch

    def _on_motor_detached(self, index, name):
        self.get_logger().warn(f'{name}: detached — will retry attaching')
        self._motors[index] = None
        self._last_position[index] = None
        self._integral[index] = 0.0
        self._reattach_logged[index] = True

    def _reattach_missing_motors(self):
        for i in range(4):
            if self._ports[i] < 0 or self._motors[i] is not None:
                continue
            motor = self._open_motor(i, quiet=self._reattach_logged[i])
            if motor is not None:
                self._motors[i] = motor
                self._reattach_logged[i] = False
                self.get_logger().info(f'{WHEEL_NAMES[i]}: reattached')
            else:
                self._reattach_logged[i] = True

    def _reattach_timer(self):
        # Runs every 2s. openWaitForAttachment normally blocks for the full
        # attach_timeout_ms (3000ms) on failure — calling that directly from
        # a timer on the same executor as the 100Hz control loop would
        # stall it for up to 3 full seconds every retry. Shrink the timeout
        # to a near-instant check just for this periodic retry so a missing
        # wheel doesn't freeze active control of the other three.
        original_timeout = self._attach_timeout
        # Was 10ms — guaranteed to fail every single cycle, since real
        # attach takes ~250-450ms per earlier bench logs. That's not just
        # ineffective, it's *why* the leak (fixed above) triggered every
        # 2s without fail. 300ms still keeps this well short of the full
        # 3000ms default, and can actually succeed sometimes.
        self._attach_timeout = 300  # ms
        try:
            self._reattach_missing_motors()
        finally:
            self._attach_timeout = original_timeout

    # ── Commands ─────────────────────────────────────────────────────────
    def _wheel_rpm_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn('wheel_rpm: expected 4 values [FL,FR,BL,BR]')
            return
        self._last_cmd_time = self.get_clock().now()
        for i in range(4):
            rpm = float(msg.data[i])
            self._target_rpm[i] = max(-self._max_rpm, min(self._max_rpm, rpm))

    def _watchdog(self):
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if age > self._cmd_timeout:
            self._target_rpm = [0.0, 0.0, 0.0, 0.0]

    # ── The actual control loop ─────────────────────────────────────────
    def _control_timer(self):
        now = self.get_clock().now()
        dt = (now - self._last_control_time).nanoseconds / 1e9
        self._last_control_time = now
        if dt <= 0.0:
            return

        for i in range(4):
            motor = self._motors[i]
            if motor is None:
                continue

            # Measured velocity via position differencing — BLDCMotor has
            # no native RPM feedback, this replaces what the onboard
            # MotorVelocityController firmware used to do for us.
            try:
                pos_now = motor.getPosition()
            except PhidgetException:
                continue
            if self._last_position[i] is None:
                self._last_position[i] = pos_now
                continue
            delta_rev = pos_now - self._last_position[i]  # already rescaled
            self._last_position[i] = pos_now
            raw_rpm = (delta_rev / dt) * 60.0 * self._dirs[i]

            # SAFETY BACKSTOP: a single-tick reading this far beyond
            # anything physically possible is almost certainly a sensor
            # glitch, not real motion — confirmed on bench (readings of
            # -371, +357 RPM from wheels that were commanded 0-43 RPM).
            # Reject it before it ever reaches the EMA filter below —
            # otherwise one huge glitch sample would still drag the
            # smoothed average off target for several ticks.
            if abs(raw_rpm) > self._max_rpm * 2.0:
                self.get_logger().warn(
                    f'{WHEEL_NAMES[i]}: rejected implausible reading '
                    f'raw_rpm={raw_rpm:.1f} (holding smoothed: '
                    f'{self._measured_rpm[i]:.1f})')
                raw_rpm = self._measured_rpm[i]

            # EMA filter, replacing the earlier error-deadband hack. The
            # deadband only gated small errors after the fact — real
            # driving showed the quantization noise (~22 RPM steps) was
            # bad enough that different wheels under the IDENTICAL command
            # read wildly different instantaneous values (one bench sample:
            # 0, 44, 22, and -0 RPM on four wheels all commanded 61.61).
            # Smoothing the velocity signal itself, before error is ever
            # computed, is the right fix — it addresses the noisy
            # measurement directly rather than papering over its symptoms.
            # alpha=0.1 is a starting value, not yet bench-tuned — lower
            # values smooth harder but add lag; watch for whether real
            # step-changes in target now feel sluggish to respond to.
            self._measured_rpm[i] = (0.1 * raw_rpm) + (0.7 * self._measured_rpm[i])

            target = self._target_rpm[i]
            error = target - self._measured_rpm[i]

            # CONFIRMED BUG FIX (bench-diagnosed): with no reset, a single
            # brief command's transient could swing integral to a clamp
            # and leave it stuck there indefinitely once error settled to
            # 0 — producing a permanent phantom duty command at true rest,
            # which fought against BrakingEnabled=True and produced
            # continuous chatter on every subsequent command. Snap
            # integral to 0 whenever there's no active command, rather
            # than letting stale windup persist across commands.
            if abs(target) < 0.5:  # RPM — effectively "no command"
                self._integral[i] = 0.0
            else:
                self._integral[i] += error * dt
            self._integral[i] = max(-self._integral_limit,
                                     min(self._integral_limit, self._integral[i]))

            duty = self._kp * error + self._ki * self._integral[i]
            duty = max(-1.0, min(1.0, duty))

            # TEMP DIAGNOSTIC — throttled to ~2Hz per wheel so it's
            # readable. Watch this BEFORE touching Kp/Ki again: if
            # measured_rpm is wildly larger/smaller than target for a
            # normal push, that's the rescale factor being wrong. If
            # measured_rpm's SIGN is opposite of target's sign while
            # actively driving forward, that's the direction-sign bug —
            # and that one specifically causes runaway oscillation
            # (positive feedback), which matches "swaying back and forth."
            if not hasattr(self, '_diag_last_log'):
                self._diag_last_log = [self.get_clock().now()] * 4
            since_log = (now - self._diag_last_log[i]).nanoseconds / 1e9
            if since_log > 0.5:
                self._diag_last_log[i] = now
                self.get_logger().info(
                    f'{WHEEL_NAMES[i]}: target={target:.2f} '
                    f'measured={self._measured_rpm[i]:.2f} error={error:.2f} '
                    f'duty={duty:.3f} integral={self._integral[i]:.3f}')

            try:
                # FIX: duty is computed in robot-frame convention (error =
                # robot-frame target - robot-frame measured), but the
                # hardware needs hardware-native direction. Mirrored wheels
                # (dirs=-1: FR, BR) were previously getting the wrong
                # physical direction — this is what caused the "chatters
                # side to side, never nets forward" symptom: left and right
                # sides were fighting each other on every forward command.
                motor.setTargetVelocity(self._dirs[i] * duty)

            except PhidgetException as e:
                self.get_logger().error(
                    f'{WHEEL_NAMES[i]}: setTargetVelocity failed ({e.details})')

    # ── Odometry — reuses measured_rpm from the control loop directly ──
    def _read_wheel_rad_s(self, index):
        return self._measured_rpm[index] * 2.0 * math.pi / 60.0

    def _odom_timer(self):
        now = self.get_clock().now()
        dt = (now - self._last_odom_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self._last_odom_time = now

        fl = self._read_wheel_rad_s(FL)
        fr = self._read_wheel_rad_s(FR)
        bl = self._read_wheel_rad_s(BL)
        br = self._read_wheel_rad_s(BR)

        r = self._r
        vx = r / 4.0 * (fl + fr + bl + br)
        vy = r / 4.0 * (-fl + fr + bl - br)
        omega = r / (4.0 * (self._L + self._W)) * (-fl + fr - bl + br)

        self._x += (vx * math.cos(self._yaw) - vy * math.sin(self._yaw)) * dt
        self._y += (vx * math.sin(self._yaw) + vy * math.cos(self._yaw)) * dt
        self._yaw += omega * dt

        self._publish(vx, vy, omega, now)

    def _publish(self, vx, vy, omega, stamp):
        qz = math.sin(self._yaw / 2.0)
        qw = math.cos(self._yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.10
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[7] = 0.05
        odom.twist.covariance[35] = 0.10
        self._odom_pub.publish(odom)

        if self._tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp.to_msg()
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(tf)

    # ── Teardown ─────────────────────────────────────────────────────────
    def shutdown(self):
        for motor in self._motors:
            if motor is None:
                continue
            try:
                motor.setTargetVelocity(0.0)
                motor.close()
            except PhidgetException:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = PhidgetBLDCNode()
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