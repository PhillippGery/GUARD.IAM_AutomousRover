# MIT License
# GUARDIAN — StarkHacks 2026
# Node: teleop_bridge_node
# Owner: Dipam
# Purpose: bridges Quest hand tracking input to LeRobot arm commands

import rclpy
from rclpy.node import Node
# TODO: import message types needed
# from geometry_msgs.msg import PoseArray
# from std_msgs.msg import Float32MultiArray


class TeleopBridgeNode(Node):
    def __init__(self):
        super().__init__('teleop_bridge_node')
        self.get_logger().info('teleop_bridge_node started')

        # TODO: declare parameters
        # self.declare_parameter('arm_scale', 1.0)

        # TODO: create subscribers
        # self.sub = self.create_subscription(
        #     PoseArray, '/quest/hand_poses', self.hand_callback, 10)

        # TODO: create publishers
        # self.pub = self.create_publisher(Float32MultiArray, '/arm_commands', 10)

        # TODO: create timers if needed
        # self.timer = self.create_timer(0.05, self.timer_callback)

    # TODO: implement callbacks
    # def hand_callback(self, msg):
    #     pass

    # def timer_callback(self):
    #     pass


def main(args=None):
    rclpy.init(args=args)
    node = TeleopBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
