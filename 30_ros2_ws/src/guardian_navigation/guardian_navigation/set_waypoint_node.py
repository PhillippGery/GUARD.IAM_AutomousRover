# MIT License
# GUARDIAN — StarkHacks 2026
# Node: set_waypoint_node
# Purpose: records the robot's current map-frame pose (via the map->
#   base_link TF) into config/waypoints.yaml at a given index, so
#   waypoints can be taught by driving the robot there instead of
#   hand-editing x/y/yaw numbers. Writes straight to the installed share/
#   path, which is a symlink back to this source tree's waypoints.yaml as
#   long as the workspace was built with --symlink-install (this repo's
#   `cb`/`cbs` aliases already do that).
#
# Three ways to trigger it, same underlying run(index) logic either way:
#   1. One-shot CLI (unchanged from before): pass the index as a ROS
#      param and this does the write and exits immediately.
#        ros2 run guardian_navigation set_waypoint_node --ros-args -p index:=2
#        (or the `set_waypoint <index>` shell function in guardiam_env.sh)
#   2. Persistent listener (for the browser control interface): launched
#      with no index param (guardian.launch.py's Nav2-core block does
#      this, alongside nav_status_node), it instead stays alive and
#      writes a waypoint every time an index arrives on
#      /guardian/set_waypoint_index (std_msgs/Int32) — type the index
#      into a Publish panel and hit Publish. Works, but the panel shows
#      raw JSON ({"data": 2}) rather than a plain number field.
#   3. Parameter-triggered (nicer UI): setting the 'target_waypoint_index'
#      ROS parameter (e.g. via Foxglove's Parameters panel, which renders
#      a real number input — no JSON, no custom panel code) fires the
#      same write through add_on_set_parameters_callback. ROS2's
#      /set_parameters service invokes this callback on every call
#      regardless of whether the value actually changed, so re-setting
#      the same index again (e.g. after re-driving to a corrected spot)
#      still re-records it — it's an RPC, not a change-watcher.
#   Confirmation/errors from any of the three go through get_logger(), so
#   a Log panel filtered to this node's name shows all of them without
#   any extra topic/code.

import math
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformException, TransformListener


class SetWaypointNode(Node):
    def __init__(self):
        super().__init__('set_waypoint_node')
        self.declare_parameter('index', -1)
        self.declare_parameter('target_waypoint_index', -1)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('timeout_sec', 10.0)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(
            Int32, '/guardian/set_waypoint_index',
            lambda msg: self.run(msg.data), 10)
        self.add_on_set_parameters_callback(self._on_set_parameters)

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == 'target_waypoint_index' and p.value >= 0:
                self.run(p.value)
        return SetParametersResult(successful=True)

    def run(self, index=None):
        # Single lookup attempt, no retry/spin loop here — this runs both
        # from main()'s CLI path (see the retry loop there instead) and,
        # in persistent mode, from inside this node's own subscription
        # callback while an executor is already spinning it. Calling
        # rclpy.spin_once() reentrantly from within a callback that's
        # itself running inside a spin isn't safe with the default
        # single-threaded executor, so the retry logic has to live
        # outside run(), where it's safe to spin.
        if index is None:
            index = self.get_parameter('index').value
        if index < 0:
            self.get_logger().error(
                'set_waypoint_node: pass a waypoint index, e.g. '
                '--ros-args -p index:=2 (or `set_waypoint 2`)')
            return False

        map_frame  = self.get_parameter('map_frame').value
        base_frame = self.get_parameter('base_frame').value

        # rclpy.time.Time() (zero) asks tf2 for the latest available
        # transform rather than a specific stamp, so this works the same
        # whether use_sim_time is true or false.
        try:
            transform = self._tf_buffer.lookup_transform(
                map_frame, base_frame, rclpy.time.Time())
        except TransformException as ex:
            # Throttled — the CLI path retries this every 0.2s, and
            # logging every single miss would flood the terminal while
            # waiting for TF to come up.
            self.get_logger().error(
                f'set_waypoint_node: no {map_frame}->{base_frame} transform '
                f'available right now ({ex}) — is localization running?',
                throttle_duration_sec=2.0)
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
        if node.get_parameter('index').value >= 0:
            # CLI one-shot: retry with real spinning (safe here — this
            # isn't inside a callback) until the transform shows up or
            # timeout expires, then exit.
            timeout = node.get_parameter('timeout_sec').value
            deadline = time.monotonic() + timeout
            while True:
                if node.run():
                    break
                if time.monotonic() >= deadline:
                    break
                rclpy.spin_once(node, timeout_sec=0.2)
        else:
            # Persistent listener mode — see class docstring.
            node.get_logger().info(
                'set_waypoint_node ready — listening on '
                '/guardian/set_waypoint_index')
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
