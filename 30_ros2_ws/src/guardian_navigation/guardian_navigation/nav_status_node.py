# MIT License
# GUARDIAN — StarkHacks 2026
# Node: nav_status_node
# Purpose: two read-only status signals for the browser control interface,
#   neither of which Nav2 exposes as a plain topic on its own:
#     /guardian/nav_status  (std_msgs/String, latched) — re-publishes
#       NavigateToPose's action feedback (distance_remaining,
#       estimated_time_remaining — Nav2 already computes both, nothing to
#       compute ourselves). Action feedback topics are hidden by default
#       and awkward to add as a Foxglove panel directly; a bare
#       subscription to the raw /navigate_to_pose/_action/feedback topic
#       is enough for a read-only display with no need to track/cancel
#       goals via a real ActionClient.
#     /guardian/stack_ready  (std_msgs/Bool, latched) — true once
#       controller_server and bt_navigator both report lifecycle state
#       ACTIVE, i.e. once it's actually safe to send a Nav2 goal or start
#       the demo mission. Both are common to mapping and navigation mode
#       (unlike amcl, which only exists in navigation mode), so this
#       works as a mode-agnostic "is the stack up" signal. Polled via
#       plain GetState service calls — the same mechanism
#       nav2_simple_commander's waitUntilNav2Active() uses internally.
#   Status text also goes through get_logger() on every change (not
#   every feedback tick — that would spam a Log panel), so a Foxglove Log
#   panel filtered to this node's name gives a nicer-looking, timestamped
#   status feed for free instead of raw JSON in a Raw Messages panel.

import rclpy
from action_msgs.msg import GoalStatusArray
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

IDLE_TIMEOUT_SEC = 2.0
READY_POLL_SEC = 1.0
STATE_ACTIVE = 3

# Latched — a Foxglove tab opened after these nodes have been running for
# a while must see the current status immediately, not just future
# updates; default VOLATILE QoS would silently drop everything published
# before it subscribed.
LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class NavStatusNode(Node):
    def __init__(self):
        super().__init__('nav_status_node')
        self._last_feedback_time = None
        self._last_status_text = None
        self._ready = None
        self._ready_call_pending = False

        self.status_pub = self.create_publisher(
            String, '/guardian/nav_status', LATCHED_QOS)
        self.ready_pub = self.create_publisher(
            Bool, '/guardian/stack_ready', LATCHED_QOS)

        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self._on_feedback, 10)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self._on_status, 10)
        self.create_timer(0.5, self._check_idle)

        self._controller_client = self.create_client(
            GetState, '/controller_server/get_state')
        self._bt_navigator_client = self.create_client(
            GetState, '/bt_navigator/get_state')
        self.create_timer(READY_POLL_SEC, self._poll_ready)

        # Publish once at startup — otherwise a fresh Foxglove subscriber
        # (or any consumer) sees nothing at all until the first goal /
        # readiness poll completes, since every other publish is
        # event-driven.
        self._publish_status('Idle')
        self._publish_ready(False)
        self.get_logger().info(
            'nav_status_node ready — publishing /guardian/nav_status and '
            '/guardian/stack_ready')

    def _on_feedback(self, msg: NavigateToPose.Impl.FeedbackMessage):
        self._last_feedback_time = self.get_clock().now()
        fb = msg.feedback
        eta = fb.estimated_time_remaining.sec + fb.estimated_time_remaining.nanosec / 1e9
        self._publish_status(
            f'Navigating — {fb.distance_remaining:.2f}m remaining, '
            f'ETA {eta:.0f}s, {fb.number_of_recoveries} recoveries',
            log_throttle_sec=2.0)

    def _on_status(self, msg: GoalStatusArray):
        # STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6 — any
        # terminal status means the goal is done; report Idle immediately
        # instead of waiting out the feedback timeout.
        if msg.status_list and msg.status_list[-1].status in (4, 5, 6):
            self._last_feedback_time = None
            self._publish_status('Idle')

    def _check_idle(self):
        if self._last_feedback_time is None:
            return
        age = (self.get_clock().now() - self._last_feedback_time).nanoseconds / 1e9
        if age > IDLE_TIMEOUT_SEC:
            self._last_feedback_time = None
            self._publish_status('Idle')

    def _publish_status(self, text: str, log_throttle_sec=None):
        self.status_pub.publish(String(data=text))
        if text != self._last_status_text:
            self.get_logger().info(f'nav_status: {text}')
        elif log_throttle_sec is not None:
            self.get_logger().info(
                f'nav_status: {text}', throttle_duration_sec=log_throttle_sec)
        self._last_status_text = text

    def _publish_ready(self, ready: bool):
        changed = ready != self._ready
        self._ready = ready
        self.ready_pub.publish(Bool(data=ready))
        if changed:
            self.get_logger().info(
                f'stack_ready: {ready} '
                f'({"controller_server + bt_navigator active" if ready else "not fully active yet"})')

    def _poll_ready(self):
        # One in-flight round-trip at a time — GetState calls are cheap
        # but there's no reason to pile up overlapping requests if a
        # response is slow.
        if self._ready_call_pending:
            return
        if not (self._controller_client.service_is_ready()
                and self._bt_navigator_client.service_is_ready()):
            self._publish_ready(False)
            return

        self._ready_call_pending = True
        controller_future = self._controller_client.call_async(GetState.Request())
        bt_navigator_future = self._bt_navigator_client.call_async(GetState.Request())

        def on_done(_future):
            if not (controller_future.done() and bt_navigator_future.done()):
                return
            self._ready_call_pending = False
            try:
                controller_active = (
                    controller_future.result().current_state.id == STATE_ACTIVE)
                bt_navigator_active = (
                    bt_navigator_future.result().current_state.id == STATE_ACTIVE)
                self._publish_ready(controller_active and bt_navigator_active)
            except Exception as ex:
                self.get_logger().warn(
                    f'stack_ready poll failed: {ex}', throttle_duration_sec=5.0)
                self._publish_ready(False)

        controller_future.add_done_callback(on_done)
        bt_navigator_future.add_done_callback(on_done)


def main(args=None):
    rclpy.init(args=args)
    node = NavStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
