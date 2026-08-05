# MIT License
# GUARDIAN — StarkHacks 2026
# Node: lidar_republisher_node
# Purpose: republishes a 360° LIDAR scan to /scan_filtered for Nav2, fixing
#          frame_id and optionally blanking out angle ranges permanently
#          self-occluded by the robot's own mount/bracket (e.g. a wall or
#          post directly behind the sensor). Every parameter — topics,
#          frame_id, and the mask — is per-instance, so the same node can be
#          launched multiple times (distinct name= + parameters=) to cover
#          more than one physical LIDAR without any code changes.

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


def _angle_in_range(angle: float, lo: float, hi: float) -> bool:
    """True if angle lies in [lo, hi], wrapping correctly across +-pi
    (e.g. lo=2.9, hi=-2.9 covers the small arc that crosses the seam)."""
    two_pi = 2.0 * math.pi
    offset = (angle - lo) % two_pi
    span = (hi - lo) % two_pi
    return offset <= span


def _parse_mask_ranges(csv: str):
    """Parse 'lo1,hi1,lo2,hi2,...' into [(lo1,hi1), (lo2,hi2), ...].
    A plain string (not a double_array parameter) sidesteps a real ROS2
    rclpy bug where declaring a double_array with an empty-list default
    makes it reject a non-empty override with
    InvalidParameterTypeException (observed on Jazzy/rclpy — the override
    gets mis-typed as BYTE_ARRAY regardless of an explicit
    ParameterDescriptor type)."""
    values = [float(v) for v in csv.split(',') if v.strip()]
    if len(values) % 2 != 0:
        raise ValueError(
            f"mask_angle_ranges must have an even number of values, got {values}")
    return list(zip(values[0::2], values[1::2]))


class LidarRepublisherNode(Node):
    def __init__(self):
        super().__init__('lidar_republisher_node')

        self.declare_parameter('input_topic', '/sweep/scan')
        self.declare_parameter('output_topic', '/scan_filtered')
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('timestamp_offset_sec', 0.05)
        # 'lo1,hi1,lo2,hi2,...' (radians, in the scan's own angle_min/
        # angle_max frame) — angle ranges to blank out as self-occlusion,
        # e.g. a wall or mount post directly behind the sensor. Each pair's
        # beams get set to NaN (ignored by SLAM/costmaps, not treated as an
        # obstacle or as clear space). Wraps correctly across the +-pi seam.
        # Empty string (default) = no masking. See _parse_mask_ranges().
        self.declare_parameter('mask_angle_ranges', '')

        in_topic  = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        # Subscribe with BEST_EFFORT to match sweep scanner publisher
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        # Publish with RELIABLE so SLAM toolbox can subscribe
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)

        self.pub = self.create_publisher(LaserScan, out_topic, pub_qos)
        self.sub = self.create_subscription(LaserScan, in_topic, self.scan_callback, sub_qos)

        self.get_logger().info(
            f'lidar_republisher_node: {in_topic} → {out_topic}')

    def scan_callback(self, msg: LaserScan):
        offset_sec = float(self.get_parameter('timestamp_offset_sec').value)
        now = self.get_clock().now()
        offset = Duration(seconds=offset_sec)
        # rclpy.Time subtraction raises if the result would be negative —
        # happens for real right at sim startup, when /clock is still close
        # to t=0 and a scan arrives before a full offset's worth of sim time
        # has elapsed. Clamp to t=0 instead of crashing the node.
        stamp = (now - offset if now.nanoseconds >= offset.nanoseconds
                 else Time(clock_type=now.clock_type))
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value

        mask_csv = self.get_parameter('mask_angle_ranges').value
        if mask_csv:
            pairs = _parse_mask_ranges(mask_csv)
            self._mask_angle_ranges(msg, pairs)

        self.pub.publish(msg)

    @staticmethod
    def _mask_angle_ranges(msg: LaserScan, pairs):
        """Set msg.ranges[i] to NaN for every beam whose angle falls inside
        one of the (lo, hi) pairs."""
        for i in range(len(msg.ranges)):
            angle = msg.angle_min + i * msg.angle_increment
            if any(_angle_in_range(angle, lo, hi) for lo, hi in pairs):
                msg.ranges[i] = float('nan')


def main(args=None):
    rclpy.init(args=args)
    node = LidarRepublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
