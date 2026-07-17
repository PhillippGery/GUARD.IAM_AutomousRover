# MIT License
# GUARDIAN — StarkHacks 2026
# Node: arm_manager_node
# Owner: Dipam
# Purpose: manages both SO-101 follower arms, coordinates dual-arm operation

import rclpy
from rclpy.node import Node
# TODO: import message types needed
# from std_msgs.msg import Float32MultiArray
# from sensor_msgs.msg import JointState


class ArmManagerNode(Node):
    def __init__(self):
        super().__init__('arm_manager_node')
        self.get_logger().info('arm_manager_node started')

        # TODO: declare parameters
        # self.declare_parameter('left_arm_port', '/dev/ttyUSB2')
        # self.declare_parameter('right_arm_port', '/dev/ttyUSB3')

        # TODO: initialize SO-101 arm connections via LeRobot
        # from lerobot.common.robot_devices.robots.factory import make_robot
        # self.left_arm = ...
        # self.right_arm = ...

        # TODO: create subscribers
        # self.sub = self.create_subscription(
        #     Float32MultiArray, '/arm_commands', self.arm_callback, 10)

        # TODO: create publishers
        # self.pub = self.create_publisher(JointState, '/arm_joint_states', 10)

        # TODO: create timers if needed
        # self.timer = self.create_timer(0.05, self.timer_callback)

    # TODO: implement callbacks
    # def arm_callback(self, msg):
    #     pass

    # def timer_callback(self):
    #     pass


def main(args=None):
    rclpy.init(args=args)
    node = ArmManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
