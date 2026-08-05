# MIT License
# GUARDIAN — StarkHacks 2026
# Script: collect_training_data
# Owner: Dipam
# Purpose: records leader arm demonstrations for LeRobot imitation learning

import rclpy
from rclpy.node import Node
# TODO: import message types needed
# from sensor_msgs.msg import JointState


class CollectTrainingDataNode(Node):
    def __init__(self):
        super().__init__('collect_training_data_node')
        self.get_logger().info('collect_training_data_node started')

        # TODO: declare parameters
        # self.declare_parameter('output_dir', '/tmp/guardian_demos')
        # self.declare_parameter('episode_name', 'episode_000')

        # TODO: subscribe to leader arm joint states
        # self.sub = self.create_subscription(
        #     JointState, '/leader_arm/joint_states', self.record_callback, 10)

        # TODO: open file/dataset for writing
        # self.data = []

    # TODO: implement callbacks
    # def record_callback(self, msg):
    #     # Append timestamp + joint positions to self.data
    #     pass

    # TODO: implement save on shutdown
    # def save_dataset(self):
    #     pass


def main(args=None):
    rclpy.init(args=args)
    node = CollectTrainingDataNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # TODO: node.save_dataset()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
