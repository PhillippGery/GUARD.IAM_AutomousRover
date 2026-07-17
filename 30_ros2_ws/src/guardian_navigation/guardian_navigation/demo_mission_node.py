# MIT License
# GUARDIAN — StarkHacks 2026
# Node: demo_mission_node
# Purpose: autonomous waypoint mission using Nav2 simple commander API

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node


def make_pose(nav, x, y, yaw=0.0):
    import math
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main(args=None):
    rclpy.init(args=args)
    nav = BasicNavigator()

    nav.get_logger().info('GUARDIAN demo mission — waiting for Nav2...')
    nav.waitUntilNav2Active()
    nav.get_logger().info('Nav2 active — starting autonomous mission')

    # ── Waypoint 1: approach confined space ──────────────────────────────────
    goal1 = make_pose(nav, 3.0, 0.0)
    nav.get_logger().info('Navigating to intervention zone (3.0, 0.0)...')
    nav.goToPose(goal1)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            remaining = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            nav.get_logger().info(f'ETA: {remaining:.1f}s', throttle_duration_sec=2.0)

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        nav.get_logger().info('Arrived at intervention zone — arms ready')
    else:
        nav.get_logger().warn(f'Goal 1 failed: {result} — continuing mission')

    # ── Hold position (simulate arm operation) ───────────────────────────────
    import time
    nav.get_logger().info('Simulating arm operation (3s)...')
    time.sleep(3.0)

    # ── Waypoint 2: return to base ───────────────────────────────────────────
    goal2 = make_pose(nav, 0.0, 0.0)
    nav.get_logger().info('Returning to base (0.0, 0.0)...')
    nav.goToPose(goal2)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            remaining = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            nav.get_logger().info(f'ETA: {remaining:.1f}s', throttle_duration_sec=2.0)

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        nav.get_logger().info('Mission complete — GUARDIAN returned to base')
    else:
        nav.get_logger().warn(f'Return failed: {result}')

    nav.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
