# MIT License
# GUARDIAN — StarkHacks 2026
# Node: web_ops_node
# Purpose: the only custom code the browser control interface needs.
#   Foxglove (via foxglove_bridge) already gives a full 3D view, click-to-
#   publish Nav2 goals, and a generic "Call Service" panel for free — this
#   node exists purely to expose the handful of actions that aren't a
#   single existing ROS2 service:
#     /guardian/switch_to_mapping    (std_srvs/Trigger)
#     /guardian/switch_to_navigation (std_srvs/Trigger)
#     /guardian/save_map             (std_srvs/Trigger)
#     /guardian/start_demo           (std_srvs/Trigger)
#   Mode switching isn't one ROS2 service — mapping vs navigation are two
#   different `ros2 launch guardian_bringup guardian.launch.py mode:=...`
#   invocations in this repo, so this shells out to systemctl against the
#   guardian-stack@.service template unit (see 60_scripts/systemd/) instead.
#   Map saving specifically must go through save_map.sh, not a raw
#   /slam_toolbox/save_map call — that script saves into the git-tracked
#   source tree and rebuilds guardian_bringup so mode:=navigation actually
#   picks up the new map; see save_map.sh's own header for why.
#
#   Meant to run as a systemd user service (guardian-ops.service) started
#   at boot alongside guardian-hardware.service and guardian-foxglove-
#   bridge.service, so it's already up by the time anyone opens Foxglove.

import os
import subprocess

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_srvs.srv import Trigger


class WebOpsNode(Node):
    def __init__(self):
        super().__init__('web_ops_node')
        self._demo_proc = None
        self.create_service(
            Trigger, '/guardian/switch_to_mapping',
            lambda req, res: self._switch_mode('mapping', res))
        self.create_service(
            Trigger, '/guardian/switch_to_navigation',
            lambda req, res: self._switch_mode('navigation', res))
        self.create_service(
            Trigger, '/guardian/save_map', self._save_map)
        self.create_service(
            Trigger, '/guardian/start_demo', self._start_demo)
        self.get_logger().info(
            'web_ops_node ready — /guardian/switch_to_mapping, '
            '/guardian/switch_to_navigation, /guardian/save_map, '
            '/guardian/start_demo')

    def _switch_mode(self, mode: str, response: Trigger.Response):
        # Stopping both instances first (not just the one we're leaving)
        # is deliberate — if this node itself was just restarted and lost
        # track of which instance was actually active, stopping an
        # already-stopped unit is a harmless no-op, but leaving the wrong
        # instance running while starting the new one would mean two
        # copies of Nav2 core fighting over /cmd_vel.
        stop = subprocess.run(
            ['systemctl', '--user', 'stop',
             'guardian-stack@mapping.service',
             'guardian-stack@navigation.service'],
            capture_output=True, text=True)
        start = subprocess.run(
            ['systemctl', '--user', 'start', f'guardian-stack@{mode}.service'],
            capture_output=True, text=True)

        if start.returncode == 0:
            response.success = True
            response.message = f'switched to {mode}'
            self.get_logger().info(f'web_ops_node: switched to {mode}')
        else:
            response.success = False
            response.message = (
                f'failed to start guardian-stack@{mode}.service: '
                f'{start.stderr.strip() or stop.stderr.strip()}')
            self.get_logger().error(f'web_ops_node: {response.message}')
        return response

    def _save_map(self, request, response: Trigger.Response):
        # save_map.sh's colcon build step can legitimately take well over
        # a minute under memory/swap pressure (see the swap-exhaustion
        # note in 10_docs/setup/web_control_interface_setup.md) — a
        # TimeoutExpired here must not propagate: an uncaught exception
        # in a service callback kills rclpy.spin() entirely, which was
        # observed taking down the whole web_ops_node (and, via
        # run_web_control.sh's trap, foxglove_bridge and the sim launch
        # right along with it) over one slow save. Report failure instead
        # and stay alive so mapping/nav/goal-sending keep working.
        bringup_dir = get_package_share_directory('guardian_bringup')
        save_map_sh = os.path.join(bringup_dir, 'scripts', 'save_map.sh')
        try:
            result = subprocess.run(
                ['bash', save_map_sh], capture_output=True, text=True, timeout=120)
        except subprocess.TimeoutExpired:
            response.success = False
            response.message = (
                'save_map.sh timed out after 120s (map_saver_cli or the '
                'colcon rebuild is stuck — check `free -h` for swap '
                'pressure, or run save_map.sh manually to see where it '
                'hangs)')
            self.get_logger().error(f'web_ops_node: {response.message}')
            return response

        if result.returncode == 0:
            response.success = True
            response.message = result.stdout.strip()[-500:]
            self.get_logger().info('web_ops_node: map saved')
        else:
            response.success = False
            response.message = (result.stderr.strip() or result.stdout.strip())[-500:]
            self.get_logger().error(f'web_ops_node: save_map failed: {response.message}')
        return response

    def _start_demo(self, request, response: Trigger.Response):
        # demo_mission_node runs for the length of the whole waypoint
        # mission (minutes, not seconds) and exits on its own when done
        # (see its own main()) — subprocess.run() here would block this
        # service callback, and therefore this entire node (same failure
        # mode _save_map used to have), for as long as the demo runs.
        # Popen is fire-and-forget: report started/already-running
        # immediately and let the demo's own process lifecycle be
        # whatever it is.
        if self._demo_proc is not None and self._demo_proc.poll() is None:
            response.success = False
            response.message = (
                f'demo already running (pid {self._demo_proc.pid})')
            self.get_logger().warn(f'web_ops_node: {response.message}')
            return response

        self._demo_proc = subprocess.Popen(
            ['ros2', 'run', 'guardian_navigation', 'demo_mission_node'])
        response.success = True
        response.message = f'demo started (pid {self._demo_proc.pid})'
        self.get_logger().info(f'web_ops_node: {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WebOpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
