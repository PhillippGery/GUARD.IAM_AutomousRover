# MIT License
# GUARDIAN — GUARD.IAM
# Node: phidget_bridge_node
# Owner: Vedant
#
# Purpose: low-level drive bridge. Subscribes to /wheel_rpm (four wheel speed
#          targets from mecanum_kinematics_node), drives the Phidget DCC1120
#          BLDC controllers over VINT/USB using their built-in closed-loop
#          velocity controllers, reads hall-based wheel velocity back, and
#          publishes /odom plus the odom->base_link TF.
#
#          This is the phidget22 wrapper: it wraps Phidgets' own Python library
#          and exposes it to ROS. Drop-in replacement for serial_bridge_node —
#          same /wheel_rpm input, same [FL, FR, BL, BR] order, same RPM units,
#          same /odom output, so nothing upstream changes.
#
# ── ADDRESSING (important) ───────────────────────────────────────────────────
#   VINT devices report the serial number of the HUB they're plugged into, not
#   one of their own. All four DCC1120s share serial 782265 (the hub). What
#   distinguishes them is the HUB PORT. So wheels are addressed by hub port.
#
#   Nothing can auto-detect which wheel is on which port — that's physical, it's
#   which socket the cable is in. So the map is declared in config, and the node
#   scans at startup and drives whatever is actually present.
#
#   Current map:  FL = hub port 0,  FR = hub port 5,
#                 BL = hub port 2,  BR = hub port 3.
#   Set a wheel's port to -1 to skip it (useful for one-wheel bring-up).
#
# ── Two things still to calibrate on the bench ───────────────────────────────
#   1. rescale_factor  — makes the controller speak wheel RPM (see note below).
#   2. wheel_directions — per-wheel sign so +RPM drives that corner forward.
#      Left and right motors are mirror-mounted, so they take opposite signs.
#      Get all four right BEFORE trusting strafing: one wrong sign makes
#      diagonal motion fail in ways that look like a kinematics bug.

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from Phidget22.Devices.MotorVelocityController import MotorVelocityController
from Phidget22.PhidgetException import PhidgetException

FL, FR, BL, BR = 0, 1, 2, 3
WHEEL_NAMES = ['FL', 'FR', 'BL', 'BR']


class PhidgetBridgeNode(Node):
    def __init__(self):
        super().__init__('phidget_bridge_node')

        # ── Addressing ───────────────────────────────────────────────────────
        # Serial of the VINT HUB (not the controllers). 0 = don't care.
        self.declare_parameter('hub_serial', 782265)
        # VINT hub port per wheel, order [FL, FR, BL, BR]. -1 = not wired yet.
        self.declare_parameter('wheel_hub_ports', [0, 5, 2, 3])
        self.declare_parameter('channel', 0)
        self.declare_parameter('attach_timeout_ms', 3000)

        # Per-wheel sign. Mirror-mounted motors need opposite signs left/right.
        self.declare_parameter('wheel_directions', [1, -1, 1, -1])

        # RESCALE FACTOR
        # The DCC1120's native velocity unit on hall feedback is commutations/sec.
        # DCM4109: 12 commutations per motor rev (4 poles x 3 phases) x 22.667
        # gearbox = 272 per WHEEL rev. So 60/272 makes both setTargetVelocity()
        # and getVelocity() speak wheel RPM, matching /wheel_rpm directly.
        #
        # VERIFY ONCE ON THE BENCH: command a known RPM, measure real wheel RPM
        # (phone slow-mo works). If you're off by exactly 272/60 or its
        # reciprocal, flip this. Phidget's multiply-vs-divide convention is the
        # one thing here worth confirming by hand rather than trusting.
        self.declare_parameter('rescale_factor', 60.0 / 272.0)

        # ── Motion limits / safety ───────────────────────────────────────────
        self.declare_parameter('current_limit', 10.0)       # A. DCC1120 floor is 5A.
        self.declare_parameter('acceleration', 200.0)       # wheel RPM/s ramp
        self.declare_parameter('max_wheel_rpm', 170.0)      # DCM4109 rated output
        self.declare_parameter('command_timeout_sec', 0.3)  # stale cmd -> stop

        # ── Odometry ─────────────────────────────────────────────────────────
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_rate_hz', 50.0)
        # Defaults match robot_params.yaml's /** wildcard block (which
        # normally overrides these) — CAD-measured values from
        # guardian_description/urdf/dimensions.xacro.
        self.declare_parameter('wheel_radius', 0.0775)      # VEX Pro 6"
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
        self._rescale = float(p('rescale_factor').value)
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

        # ── Startup scan ─────────────────────────────────────────────────────
        self._motors = [self._open_motor(i) for i in range(4)]

        live = [WHEEL_NAMES[i] for i in range(4) if self._motors[i] is not None]
        skipped = [WHEEL_NAMES[i] for i in range(4) if self._ports[i] < 0]
        missing = [WHEEL_NAMES[i] for i in range(4)
                   if self._ports[i] >= 0 and self._motors[i] is None]

        self.get_logger().info(f'Wheels driving: {live or "NONE"}')
        if skipped:
            self.get_logger().info(f'Wheels not wired yet (port -1): {skipped}')
        if missing:
            self.get_logger().error(
                f'Wheels CONFIGURED BUT NOT FOUND: {missing}. '
                'Check 24V power and VINT cabling for those ports.')
        if not live:
            self.get_logger().error(
                'No controllers attached — node is up but will not move anything.')

        # ── Pose state ───────────────────────────────────────────────────────
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        # ── ROS interface ────────────────────────────────────────────────────
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

        self.create_timer(0.05, self._watchdog)  # 20 Hz

        self.get_logger().info(
            f'phidget_bridge_node started (odom={self._publish_odom}, '
            f'tf={self._publish_tf})')

    # ── Setup ────────────────────────────────────────────────────────────────
    def _open_motor(self, index):
        name = WHEEL_NAMES[index]
        port = self._ports[index]
        if port is None or port < 0:
            return None  # deliberately not wired yet

        ch = MotorVelocityController()
        if self._hub_serial:
            ch.setDeviceSerialNumber(self._hub_serial)
        ch.setHubPort(int(port))
        ch.setChannel(self._channel)

        try:
            ch.openWaitForAttachment(self._attach_timeout)
        except PhidgetException as e:
            self.get_logger().error(
                f'{name}: no controller on hub port {port} ({e.details})')
            return None

        try:
            ch.setRescaleFactor(self._rescale)
            ch.setCurrentLimit(self._current_limit)
            ch.setAcceleration(self._accel)
            ch.setTargetVelocity(0.0)
            ch.setEngaged(True)  # REQUIRED — otherwise it freewheels
        except PhidgetException as e:
            self.get_logger().error(f'{name}: config failed ({e.details})')
            try:
                ch.close()
            except PhidgetException:
                pass
            return None

        self.get_logger().info(f'{name}: ready on hub port {port}')
        return ch

    # ── Commands ─────────────────────────────────────────────────────────────
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
        rpm = max(-self._max_rpm, min(self._max_rpm, rpm))
        try:
            motor.setTargetVelocity(self._dirs[index] * rpm)
        except PhidgetException as e:
            self.get_logger().error(
                f'{WHEEL_NAMES[index]}: setTargetVelocity failed ({e.details})')

    def _watchdog(self):
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if age > self._cmd_timeout:
            for i in range(4):
                self._set_wheel(i, 0.0)

    # ── Feedback ─────────────────────────────────────────────────────────────
    def _read_wheel_rad_s(self, index):
        """Measured wheel speed in rad/s, in the robot's sign convention.

        getVelocity() returns measured velocity; with rescale_factor set above
        that's wheel RPM. Undo the per-wheel direction flip so the kinematics
        sees consistent signs.

        A wheel that isn't wired reads 0.0, which would skew the mecanum
        averages, so odom is only meaningful with all four connected.
        """
        motor = self._motors[index]
        if motor is None:
            return 0.0
        try:
            rpm = motor.getVelocity()
        except PhidgetException:
            return 0.0
        rpm *= self._dirs[index]
        return rpm * 2.0 * math.pi / 60.0

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

        # Mecanum forward kinematics — same form as the old serial bridge.
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

        # Mecanum wheel odometry slips, and hall feedback is coarse (272 counts
        # per wheel rev). Keep covariance honest so the EKF weights this
        # sensibly instead of trusting it like a laser.
        odom.pose.covariance[0] = 0.05    # x
        odom.pose.covariance[7] = 0.05    # y
        odom.pose.covariance[35] = 0.10   # yaw
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

    # ── Teardown ─────────────────────────────────────────────────────────────
    def shutdown(self):
        for motor in self._motors:
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