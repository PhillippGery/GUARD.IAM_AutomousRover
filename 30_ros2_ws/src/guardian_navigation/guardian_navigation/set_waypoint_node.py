# MIT License
# GUARDIAN — StarkHacks 2026
# Node: set_waypoint_node
# Purpose: one-shot CLI helper — records the robot's current map-frame pose
#   (via the map->base_link TF) into config/waypoints.yaml at a given index,
#   so waypoints can be taught by driving the robot there instead of
#   hand-editing x/y/yaw numbers. Writes straight to the installed share/
#   path, which is a symlink back to this source tree's waypoints.yaml as
#   long as the workspace was built with --symlink-install (this repo's
#   `cb`/`cbs` aliases already do that).
#
# Usage: ros2 run guardian_navigation set_waypoint_node --ros-args -p index:=2
#   (or the `set_waypoint <index>` shell function in guardiam_env.sh)

import math
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class SetWaypointNode(Node):
    def __init__(self):
        super().__init__('set_waypoint_node')
        self.declare_parameter('index', -1)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('timeout_sec', 10.0)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def run(self):
        index = self.get_parameter('index').value
        if index < 0:
            self.get_logger().error(
                'set_waypoint_node: pass a waypoint index, e.g. '
                '--ros-args -p index:=2 (or `set_waypoint 2`)')
            return False

        map_frame  = self.get_parameter('map_frame').value
        base_frame = self.get_parameter('base_frame').value
        timeout    = self.get_parameter('timeout_sec').value

        # rclpy.time.Time() (zero) asks tf2 for the latest available
        # transform rather than a specific stamp, so this works the same
        # whether use_sim_time is true or false — no clock-domain match
        # needed for this one-shot read.
        transform = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                transform = self._tf_buffer.lookup_transform(
                    map_frame, base_frame, rclpy.time.Time())
                break
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.2)

        if transform is None:
            self.get_logger().error(
                f'set_waypoint_node: no {map_frame}->{base_frame} transform '
                f'after {timeout:.0f}s — is localization running?')
            return False

        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        bringup_dir = get_package_share_directory('guardian_bringup')
        waypoints_path = os.path.join(bringup_dir, 'config', 'waypoints.yaml')

        with open(waypoints_path) as f:
            data = yaml.safe_load(f) or {}
        waypoints = data.get('waypoints') or []

        while len(waypoints) <= index:
            waypoints.append({'x': 0.0, 'y': 0.0, 'yaw': 0.0})
        waypoints[index] = {
            'x': round(t.x, 4), 'y': round(t.y, 4), 'yaw': round(yaw, 4)}
        data['waypoints'] = waypoints

        with open(waypoints_path, 'w') as f:
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)

        self.get_logger().info(
            f'set_waypoint_node: waypoint[{index}] = '
            f'(x={t.x:.2f}, y={t.y:.2f}, yaw={math.degrees(yaw):.0f}deg) '
            f'written to {waypoints_path}')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SetWaypointNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
