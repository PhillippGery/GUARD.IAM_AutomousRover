# MIT License
# GUARDIAN — StarkHacks 2026
# Node: demo_mission_node
# Purpose: autonomous waypoint-to-waypoint demo mission using the Nav2
#   simple commander API. Waypoints come from config/waypoints.yaml — hand-
#   edit that file, or record one live by driving the robot there and
#   running `set_waypoint <index>` (see guardiam_env.sh / set_waypoint_node).
#   Drives to each waypoint in order; on arrival it hands off to the arm
#   manipulation ACT policy (see run_act_policy — not wired up yet) before
#   continuing on to the next waypoint.
#
# Run this against an already-running nav stack (e.g. after `guardiam_sim`):
#   ros2 run guardian_navigation demo_mission_node

import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration


def make_pose(nav, x, y, yaw=0.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def load_waypoints():
    bringup_dir = get_package_share_directory('guardian_bringup')
    waypoints_path = os.path.join(bringup_dir, 'config', 'waypoints.yaml')
    with open(waypoints_path) as f:
        data = yaml.safe_load(f) or {}
    return data.get('waypoints') or []


def navigate_to(nav, pose, label):
    nav.get_logger().info(
        f'Navigating to {label} '
        f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})...')
    nav.goToPose(pose)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            remaining = Duration.from_msg(
                feedback.estimated_time_remaining).nanoseconds / 1e9
            nav.get_logger().info(f'ETA: {remaining:.1f}s', throttle_duration_sec=2.0)
    return nav.getResult()


def run_act_policy(nav, index):
    # TODO: trigger the trained ACT policy on the arms here, once
    # guardian_manipulation/arm_manager_node is actually implemented — e.g.
    # call an action/service that node exposes (or invoke the LeRobot
    # policy runner directly) and block until the manipulation pass at
    # this waypoint finishes before navigating on to the next one.
    nav.get_logger().info(
        f'run_act_policy: TODO — would run the ACT policy at waypoint '
        f'{index} here')


def main(args=None):
    rclpy.init(args=args)
    nav = BasicNavigator()

    nav.get_logger().info('GUARDIAN demo mission — waiting for Nav2...')
    nav.waitUntilNav2Active()
    nav.get_logger().info('Nav2 active — starting waypoint mission')

    waypoints = load_waypoints()
    if not waypoints:
        nav.get_logger().error(
            'demo_mission_node: config/waypoints.yaml has no waypoints — '
            'record some with `set_waypoint <index>` first')
        nav.destroy_node()
        rclpy.shutdown()
        return

    for i, wp in enumerate(waypoints):
        pose = make_pose(nav, wp['x'], wp['y'], wp.get('yaw', 0.0))
        result = navigate_to(nav, pose, f'waypoint {i}')

        if result == TaskResult.SUCCEEDED:
            nav.get_logger().info(f'Arrived at waypoint {i}')
            run_act_policy(nav, i)
        else:
            nav.get_logger().warn(
                f'Waypoint {i} failed: {result} — continuing mission')

    nav.get_logger().info('Mission complete — all waypoints visited')
    # Deliberately not nav.lifecycleShutdown(): that waits on a
    # /lifecycle_manager_localization/manage_nodes service, which only
    # exists in stock nav2_bringup setups. guardian.launch.py activates
    # map_server/amcl via activate_lifecycle_node.sh instead — there's no
    # such lifecycle manager here, so that call would hang forever. This
    # node also runs against an already-running stack (e.g. after
    # guardiam_sim), so it shouldn't tear down Nav2's lifecycle on exit
    # anyway — just clean up this node and leave the stack running.
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
