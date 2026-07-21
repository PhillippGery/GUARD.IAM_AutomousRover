# MIT License
# GUARDIAN — StarkHacks 2026
# Node: serial_bridge_node
# Purpose: sends RPM targets to two ESP32 boards over USB serial, optionally reads encoder feedback and publishes /odom
# Layout: LEFT ESP32 = FL+BL (M1=FL, M2=BL), RIGHT ESP32 = FR+BR (M1=FR, M2=BR)

import math
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('serial_port',       '')
        self.declare_parameter('serial_port_left',  '/dev/ttyACM0')
        self.declare_parameter('serial_port_right', '/dev/ttyACM1')
        self.declare_parameter('baud_rate',          115200)
        self.declare_parameter('serial_timeout',     1.0)
        # Defaults match robot_params.yaml (which normally overrides these) —
        # CAD-measured values from guardian_description/urdf/dimensions.xacro.
        self.declare_parameter('wheel_radius',       0.0775)
        self.declare_parameter('wheel_base_length',  0.313588)
        self.declare_parameter('wheel_base_width',   0.390755)
        self.declare_parameter('cpr',                175.0)
        self.declare_parameter('max_ticks_per_sec',  3000.0)
        self.declare_parameter('sim_mode',           False)
        self.declare_parameter('publish_odom',       True)
        self.declare_parameter('publish_tf',         True)

        self.sim_mode  = self.get_parameter('sim_mode').value
        self.publish_odom = self.get_parameter('publish_odom').value
        self.publish_tf = self.get_parameter('publish_tf').value
        serial_port    = self.get_parameter('serial_port').value
        port_left      = self.get_parameter('serial_port_left').value
        port_right     = self.get_parameter('serial_port_right').value
        baud           = self.get_parameter('baud_rate').value
        timeout        = self.get_parameter('serial_timeout').value
        self.cpr               = self.get_parameter('cpr').value
        self.max_ticks_per_sec = self.get_parameter('max_ticks_per_sec').value

        if self.sim_mode:
            self.serial_mode = 'sim'
            self.ser_single = self.ser_left = self.ser_right = None
            self.get_logger().warn('sim_mode=true — serial disabled, publishing zero odometry')
        elif serial_port:
            self.serial_mode = 'single_arduino'
            self.ser_single = self._open_port(serial_port, baud, timeout)
            self.ser_left = self.ser_right = None
        else:
            self.serial_mode = 'dual_esp32'
            self.ser_single = None
            self.ser_left  = self._open_port(port_left,  baud, timeout)
            self.ser_right = self._open_port(port_right, baud, timeout)

        self.sub = self.create_subscription(
            Float32MultiArray, '/wheel_rpm', self.rpm_callback, 10)
        self.odom_pub = None
        self.tf_broadcaster = None
        if self.publish_odom:
            self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
            if self.publish_tf:
                self.tf_broadcaster = TransformBroadcaster(self)

        self.x = self.y = self.yaw = 0.0
        self.last_time = self.get_clock().now()

        # Cumulative tick tracking (per-board timestamps)
        # LEFT board:  M1=FL, M2=BL
        # RIGHT board: M1=FR, M2=BR
        self._prev_fl = self._prev_bl = 0
        self._prev_fr = self._prev_br = 0
        self._prev_tick_time_single = self.get_clock().now()
        self._prev_tick_time_left  = self.get_clock().now()
        self._prev_tick_time_right = self.get_clock().now()

        # Latest velocity estimates (ticks/sec)
        self._vel_fl = self._vel_fr = 0.0
        self._vel_bl = self._vel_br = 0.0

        if self.publish_odom:
            self.create_timer(0.02, self.read_encoders)  # 50 Hz

        if self.serial_mode == 'single_arduino':
            self.get_logger().info(
                f'serial_bridge_node started (MODE=single_arduino, PORT={serial_port}, '
                f'publish_odom={self.publish_odom}, publish_tf={self.publish_tf})')
        else:
            self.get_logger().info(
                f'serial_bridge_node started (MODE={self.serial_mode}, LEFT={port_left}, '
                f'RIGHT={port_right}, publish_odom={self.publish_odom}, '
                f'publish_tf={self.publish_tf})')

    def _open_port(self, port, baud, timeout):
        try:
            s = serial.Serial(port, baud, timeout=timeout)
            self.get_logger().info(f'Opened {port} at {baud} baud')
            return s
        except serial.SerialException as e:
            self.get_logger().warn(f'Could not open {port}: {e}')
            return None

    # ── Commands ────────────────────────────────────────────────────────────────
    def rpm_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn('wheel_rpm: need 4 values')
            return
        fl_rpm, fr_rpm, bl_rpm, br_rpm = msg.data[:4]

        if self.serial_mode == 'single_arduino':
            self._write_safe(
                self.ser_single,
                f'FL:{int(round(fl_rpm))} FR:{int(round(fr_rpm))} '
                f'BL:{int(round(bl_rpm))} BR:{int(round(br_rpm))}\n',
                'MAIN')
            return

        # LEFT board drives FL (M1) and BL (M2)
        self._write_safe(self.ser_left,  f'M1:{fl_rpm:.2f} M2:{bl_rpm:.2f}\n', 'LEFT')
        # RIGHT board drives FR (M1) and BR (M2)
        self._write_safe(self.ser_right, f'M1:{fr_rpm:.2f} M2:{br_rpm:.2f}\n', 'RIGHT')

    def _write_safe(self, ser, line, label):
        if ser is None:
            return
        try:
            ser.write(line.encode())
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial write ({label}): {e}')

    # ── Encoder reading ──────────────────────────────────────────────────────────
    def read_encoders(self):
        if not self.publish_odom:
            return
        if self.sim_mode or (self.ser_single is None and self.ser_left is None and self.ser_right is None):
            self._publish_odom(0.0, 0.0, 0.0, self.get_clock().now())
            return

        if self.serial_mode == 'single_arduino':
            updated = self._read_single_board(self.ser_single, 'MAIN')
        else:
            updated = False
            updated |= self._read_board(self.ser_left,  'LEFT',  is_left=True)
            updated |= self._read_board(self.ser_right, 'RIGHT', is_left=False)
        if updated:
            self._compute_odom()

    def _read_single_board(self, ser, label):
        try:
            if ser is None or not ser.in_waiting:
                return False
            raw = ser.readline()
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial read ({label}): {e}')
            return False

        line = raw.decode(errors='replace').strip()
        if not line.startswith('ENC:'):
            return False

        try:
            parts = {}
            for token in line[4:].split():
                k, _, v = token.partition(':')
                parts[k] = int(v)
            fl = parts['FL']
            fr = parts['FR']
            bl = parts['BL']
            br = parts['BR']
        except (ValueError, KeyError):
            return False

        now = self.get_clock().now()
        dt = (now - self._prev_tick_time_single).nanoseconds / 1e9
        if dt <= 0:
            return False

        self._vel_fl = (fl - self._prev_fl) / dt
        self._vel_fr = (fr - self._prev_fr) / dt
        self._vel_bl = (bl - self._prev_bl) / dt
        self._vel_br = (br - self._prev_br) / dt
        self._prev_fl = fl
        self._prev_fr = fr
        self._prev_bl = bl
        self._prev_br = br
        self._prev_tick_time_single = now
        return True

    def _read_board(self, ser, label, is_left):
        try:
            if ser is None or not ser.in_waiting:
                return False
            raw = ser.readline()
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial read ({label}): {e}')
            return False

        line = raw.decode(errors='replace').strip()

        if not line.startswith('ENC:'):
            if 'alive' in line:
                self.get_logger().debug(f'Heartbeat ({label}): {line}')
            return False

        try:
            parts = {}
            for token in line[4:].split():
                k, _, v = token.partition(':')
                parts[k] = int(v)
            m1 = parts['M1']
            m2 = parts['M2']
        except (ValueError, KeyError):
            return False

        now = self.get_clock().now()
        if is_left:
            dt = (now - self._prev_tick_time_left).nanoseconds / 1e9
            if dt <= 0:
                return False
            self._vel_fl = (m1 - self._prev_fl) / dt  # M1 = FL
            self._vel_bl = (m2 - self._prev_bl) / dt  # M2 = BL
            self._prev_fl = m1
            self._prev_bl = m2
            self._prev_tick_time_left = now
        else:
            dt = (now - self._prev_tick_time_right).nanoseconds / 1e9
            if dt <= 0:
                return False
            self._vel_fr = (m1 - self._prev_fr) / dt  # M1 = FR
            self._vel_br = (m2 - self._prev_br) / dt  # M2 = BR
            self._prev_fr = m1
            self._prev_br = m2
            self._prev_tick_time_right = now

        return True

    # ── Odometry ─────────────────────────────────────────────────────────────────
    def _compute_odom(self):
        r = self.get_parameter('wheel_radius').value
        L = self.get_parameter('wheel_base_length').value / 2.0
        W = self.get_parameter('wheel_base_width').value / 2.0

        # ticks/sec → rad/s
        k = 2.0 * math.pi / self.cpr
        fl = self._vel_fl * k
        fr = self._vel_fr * k
        bl = self._vel_bl * k
        br = self._vel_br * k

        vx    = r / 4.0 * (fl + fr + bl + br)
        vy    = r / 4.0 * (-fl + fr + bl - br)
        omega = r / (4.0 * (L + W)) * (-fl + fr - bl + br)

        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        self.x   += (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        self.y   += (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
        self.yaw += omega * dt

        self._publish_odom(vx, vy, omega, now)

    def _publish_odom(self, vx, vy, omega, stamp):
        if not self.publish_odom:
            return
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        odom = Odometry()
        odom.header.stamp    = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x    = vx
        odom.twist.twist.linear.y    = vy
        odom.twist.twist.angular.z   = omega
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp    = stamp.to_msg()
            tf.header.frame_id = 'odom'
            tf.child_frame_id  = 'base_link'
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z    = qz
            tf.transform.rotation.w    = qw
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
