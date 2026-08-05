# MIT License
# GUARDIAN — StarkHacks 2026
# Node: localization_bootstrap_node
# Purpose: get AMCL to a confident pose with zero manual input, so
#          guardian.launch.py can start fully autonomous / headless (no
#          RViz "2D Pose Estimate" click) in both sim and real:
#            - known_pose:=true  — seed a fixed pose directly (sim spawn
#              point, or a real deployment that always starts from the
#              same known spot). Instant.
#            - known_pose:=false — reinitialize AMCL's own global
#              localization (particles spread across the whole map), send
#              a Spin goal to the already-running behavior_server so the
#              robot gathers a few distinct scan viewpoints, then poll
#              /amcl_pose's covariance until it's confident. Retries up to
#              max_attempts before giving up.
#          Publishes /localized (std_msgs/Bool, transient-local so late
#          subscribers still see it) once resolved, so mission logic knows
#          whether it is safe to send Nav2 goals.

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import Spin
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import Empty


class LocalizationBootstrapNode(Node):
    def __init__(self):
        super().__init__('localization_bootstrap_node')

        self.declare_parameter('known_pose', False)
        self.declare_parameter('known_pose_x', 0.0)
        self.declare_parameter('known_pose_y', 0.0)
        self.declare_parameter('known_pose_yaw', 0.0)
        # Diagonal covariance (m^2 / rad^2) below which a pose counts as
        # "confident enough" — tune against real AMCL behavior on GUARDIAN.
        self.declare_parameter('covariance_threshold', 0.25)
        self.declare_parameter('spin_target_yaw', 2.0 * math.pi)
        self.declare_parameter('spin_timeout_sec', 30.0)
        self.declare_parameter('max_attempts', 5)
        # behavior_server's /spin action server can be discoverable before
        # its lifecycle node has actually reached the ACTIVE state — in
        # that window it rejects goals immediately (not a slow timeout), so
        # attempts need real spacing or they all burn through instantly.
        self.declare_parameter('retry_delay_sec', 3.0)

        localized_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._localized_pub = self.create_publisher(Bool, '/localized', localized_qos)
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 1)

        self._latest_covariance = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_pose_cb, 10)

        self._spin_client = ActionClient(self, Spin, '/spin')
        self._global_loc_client = self.create_client(
            Empty, '/reinitialize_global_localization')

        self._attempt = 0
        # Small grace period on top of whatever delay the launch file already
        # gives AMCL/behavior_server to finish activating.
        self._start_timer = self.create_timer(2.0, self._start)

    def _start(self):
        self._start_timer.cancel()
        if self.get_parameter('known_pose').value:
            self._seed_known_pose()
        else:
            self._reinitialize_global_localization()

    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped):
        self._latest_covariance = msg.pose.covariance

    def _seed_known_pose(self):
        x = self.get_parameter('known_pose_x').value
        y = self.get_parameter('known_pose_y').value
        yaw = self.get_parameter('known_pose_yaw').value

        # /initialpose has no retained delivery for a late-discovered
        # subscriber — if AMCL's subscription hasn't matched our publisher
        # yet at this exact instant, the one-shot publish below would just
        # be silently lost. Wait for the match first.
        deadline = time.monotonic() + 10.0
        while (self._initialpose_pub.get_subscription_count() == 0
               and time.monotonic() < deadline):
            time.sleep(0.1)
        if self._initialpose_pub.get_subscription_count() == 0:
            self.get_logger().warn(
                'localization_bootstrap_node: no /initialpose subscriber '
                'found after 10s (is AMCL up?) — publishing anyway')

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05   # x, y variance — tight, we're confident
        cov[35] = 0.05           # yaw variance
        msg.pose.covariance = cov

        self._initialpose_pub.publish(msg)
        self.get_logger().info(
            f'localization_bootstrap_node: seeded known pose '
            f'({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}deg) — done')
        self._declare_localized(True)

    def _reinitialize_global_localization(self):
        # Called exactly once — this resets AMCL's particle filter to spread
        # across the whole map. Retries below must NOT call this again, or
        # every attempt would throw away the convergence progress from the
        # spins before it and start over from scratch.
        self.get_logger().info(
            'localization_bootstrap_node: reinitializing global localization')

        if not self._global_loc_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                'localization_bootstrap_node: /reinitialize_global_localization '
                'unavailable — is AMCL running?')
            self._declare_localized(False)
            return

        future = self._global_loc_client.call_async(Empty.Request())
        future.add_done_callback(lambda f: self._attempt_spin())

    def _attempt_spin(self):
        self._attempt += 1
        self.get_logger().info(
            f'localization_bootstrap_node: attempt {self._attempt}/'
            f'{self.get_parameter("max_attempts").value} — '
            f'spinning to gather distinguishing scans')
        self._send_spin_goal()

    def _send_spin_goal(self):
        if not self._spin_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'localization_bootstrap_node: /spin action server unavailable '
                '— is behavior_server running?')
            self._declare_localized(False)
            return

        goal = Spin.Goal()
        goal.target_yaw = float(self.get_parameter('spin_target_yaw').value)
        goal.time_allowance.sec = int(self.get_parameter('spin_timeout_sec').value)

        send_future = self._spin_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_spin_goal_response)

    def _on_spin_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(
                'localization_bootstrap_node: spin goal rejected, retrying')
            self._retry_or_give_up()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_spin_result)

    def _on_spin_result(self, future):
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f'localization_bootstrap_node: spin did not succeed '
                f'(status={status}) — checking pose confidence anyway')
        self._check_convergence()

    def _check_convergence(self):
        if self._latest_covariance is None:
            self.get_logger().warn(
                'localization_bootstrap_node: no /amcl_pose received yet')
            self._retry_or_give_up()
            return

        cov = self._latest_covariance
        threshold = self.get_parameter('covariance_threshold').value
        xx, yy, yaw_yaw = cov[0], cov[7], cov[35]
        self.get_logger().info(
            f'localization_bootstrap_node: covariance x={xx:.3f} y={yy:.3f} '
            f'yaw={yaw_yaw:.3f} (threshold={threshold})')

        if xx < threshold and yy < threshold and yaw_yaw < threshold:
            self.get_logger().info(
                'localization_bootstrap_node: pose confident — localized')
            self._declare_localized(True)
        else:
            self._retry_or_give_up()

    def _retry_or_give_up(self):
        if self._attempt < self.get_parameter('max_attempts').value:
            delay = self.get_parameter('retry_delay_sec').value
            self.get_logger().info(
                f'localization_bootstrap_node: retrying in {delay:.0f}s')
            retry_timer_box = {}

            def _fire():
                retry_timer_box['timer'].cancel()
                self._attempt_spin()

            retry_timer_box['timer'] = self.create_timer(delay, _fire)
        else:
            self.get_logger().warn(
                f'localization_bootstrap_node: gave up after {self._attempt} '
                f'attempts — pose confidence still below threshold. '
                f'Navigation may be unreliable until this resolves '
                f'(retry manually via /reinitialize_global_localization, '
                f'or send a 2D Pose Estimate in RViz).')
            self._declare_localized(False)

    def _declare_localized(self, success: bool):
        self._localized_pub.publish(Bool(data=success))


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationBootstrapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
