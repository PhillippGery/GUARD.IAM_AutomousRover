# MIT License
# GUARDIAN — StarkHacks 2026
# Node: mecanum_kinematics_node
# Purpose: converts /cmd_vel Twist to 4× wheel RPM targets using mecanum kinematics

from math import pi
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class MecanumKinematicsNode(Node):
    def __init__(self):
        super().__init__('mecanum_kinematics_node')

        # Defaults match robot_params.yaml (which normally overrides these) —
        # CAD-measured values from guardian_description/urdf/dimensions.xacro.
        self.declare_parameter('wheel_radius', 0.0775)      # 6" VEXpro mecanum wheels
        self.declare_parameter('wheel_base_length', 0.313588)
        self.declare_parameter('wheel_base_width', 0.390755)
        self.declare_parameter('max_rpm', 170.0)  # real motor rated max continuous speed

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/wheel_rpm', 10)

        self.get_logger().info('mecanum_kinematics_node started')

    def cmd_vel_callback(self, msg: Twist):
        r       = self.get_parameter('wheel_radius').value
        L       = self.get_parameter('wheel_base_length').value / 2.0
        W       = self.get_parameter('wheel_base_width').value / 2.0
        max_rpm = self.get_parameter('max_rpm').value

        vx    = msg.linear.x
        vy    = msg.linear.y
        omega = msg.angular.z
        k     = L + W
        to_rpm = 60.0 / (2.0 * pi)

        fl = (vx - vy - k * omega) / r * to_rpm
        fr = (vx + vy + k * omega) / r * to_rpm
        bl = (vx + vy - k * omega) / r * to_rpm
        br = (vx - vy + k * omega) / r * to_rpm

        # Clamp while preserving wheel speed ratios
        peak = max(abs(fl), abs(fr), abs(bl), abs(br), 1e-6)
        if peak > max_rpm:
            scale = max_rpm / peak
            fl *= scale; fr *= scale; bl *= scale; br *= scale

        self.get_logger().debug(f'RPM FL={fl:.1f} FR={fr:.1f} BL={bl:.1f} BR={br:.1f}')

        out = Float32MultiArray()
        out.data = [fl, fr, bl, br]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
