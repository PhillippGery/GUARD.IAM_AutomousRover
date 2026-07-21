# MIT License
# GUARDIAN — StarkHacks 2026
# Node: lidar_merger_node
# Purpose: combines the front and back LIDAR scans into one virtual 360°
#          scan centered on base_link, for Nav2/SLAM to consume as a
#          single topic. Each sensor only sees ~360° around ITSELF, but
#          the two are mounted at opposite ends of the chassis rather
#          than at a shared origin, so this isn't just concatenating
#          ranges — each beam is converted to a point in base_link's
#          frame (using that sensor's fixed x/y/yaw offset — see
#          guardian_sim.urdf.xacro's laser/laser_back joints) and then
#          re-binned into one scan by angle from base_link's own origin,
#          keeping the closer of the two returns if both sensors saw
#          something in the same bin.
#
# Self-occlusion (each sensor's beams that hit this robot's own chassis)
# is NOT filtered here — that's lidar_republisher_node's mask_angle_ranges,
# applied upstream to each raw topic before it reaches this node. Run
# both sensors' raw topics through their own lidar_republisher_node
# first, then merge.

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


class LidarMergerNode(Node):
    def __init__(self):
        super().__init__('lidar_merger_node')

        self.declare_parameter('front_input_topic', '/scan')
        self.declare_parameter('back_input_topic', '/scan_back')
        self.declare_parameter('output_topic', '/scan_filtered')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('timestamp_offset_sec', 0.05)

        # Each sensor's fixed offset from base_link — must match the x/y
        # and yaw given to the `lidar` xacro macro in
        # guardian_sim.urdf.xacro (lidar_front_*/lidar_back_* come from
        # dimensions.xacro, auto-generated off the CAD). yaw is radians;
        # the back unit is physically mounted rotated 180 degrees so its
        # own "forward" points away from the chassis, not into it.
        self.declare_parameter('front_x', 0.319650)
        self.declare_parameter('front_y', 0.0)
        self.declare_parameter('front_yaw', 0.0)
        self.declare_parameter('back_x', -0.319650)
        self.declare_parameter('back_y', 0.0)
        self.declare_parameter('back_yaw', math.pi)

        # Output virtual scan geometry — matches the raw gpu_lidar sensor
        # config in guardian_sim.urdf.xacro (500 samples, full circle,
        # 0.1-20m) so nothing is lost re-binning either sensor into this.
        self.declare_parameter('num_samples', 500)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 20.0)

        front_topic = self.get_parameter('front_input_topic').value
        back_topic = self.get_parameter('back_input_topic').value
        out_topic = self.get_parameter('output_topic').value

        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)

        self._front_scan = None
        self._back_scan = None

        self.pub = self.create_publisher(LaserScan, out_topic, pub_qos)
        self.front_sub = self.create_subscription(
            LaserScan, front_topic, self._front_callback, sub_qos)
        self.back_sub = self.create_subscription(
            LaserScan, back_topic, self._back_callback, sub_qos)

        self.get_logger().info(
            f'lidar_merger_node: {front_topic} + {back_topic} -> {out_topic}')

    def _front_callback(self, msg):
        self._front_scan = msg
        self._publish_merged()

    def _back_callback(self, msg):
        self._back_scan = msg
        self._publish_merged()

    def _publish_merged(self):
        if self._front_scan is None or self._back_scan is None:
            return  # wait until both sensors have reported at least once

        num_samples = int(self.get_parameter('num_samples').value)
        range_min = float(self.get_parameter('range_min').value)
        range_max = float(self.get_parameter('range_max').value)
        out_angle_min = -math.pi
        out_angle_increment = (2.0 * math.pi) / num_samples

        merged = [float('inf')] * num_samples

        self._project_into(
            merged, self._front_scan,
            self.get_parameter('front_x').value,
            self.get_parameter('front_y').value,
            self.get_parameter('front_yaw').value,
            out_angle_min, out_angle_increment, num_samples,
            range_min, range_max)
        self._project_into(
            merged, self._back_scan,
            self.get_parameter('back_x').value,
            self.get_parameter('back_y').value,
            self.get_parameter('back_yaw').value,
            out_angle_min, out_angle_increment, num_samples,
            range_min, range_max)

        out = LaserScan()
        offset_sec = float(self.get_parameter('timestamp_offset_sec').value)
        now = self.get_clock().now()
        offset = Duration(seconds=offset_sec)
        # See lidar_republisher_node for why this is clamped rather than
        # subtracted unconditionally — negative Time near sim startup.
        stamp = (now - offset if now.nanoseconds >= offset.nanoseconds
                 else Time(clock_type=now.clock_type))
        out.header.stamp = stamp.to_msg()
        out.header.frame_id = self.get_parameter('frame_id').value
        out.angle_min = out_angle_min
        out.angle_max = out_angle_min + out_angle_increment * (num_samples - 1)
        out.angle_increment = out_angle_increment
        out.time_increment = 0.0
        out.scan_time = 0.0
        out.range_min = range_min
        out.range_max = range_max
        out.ranges = [r if math.isfinite(r) else float('inf') for r in merged]
        self.pub.publish(out)

    @staticmethod
    def _project_into(merged, scan, sx, sy, syaw, out_angle_min,
                       out_angle_increment, num_samples, range_min, range_max):
        cos_yaw = math.cos(syaw)
        sin_yaw = math.sin(syaw)
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < range_min or r > range_max:
                continue
            angle_local = scan.angle_min + i * scan.angle_increment
            x_local = r * math.cos(angle_local)
            y_local = r * math.sin(angle_local)
            # Rotate by the sensor's mount yaw, then translate by its
            # (x, y) offset — puts this beam's endpoint in base_link.
            x_base = sx + x_local * cos_yaw - y_local * sin_yaw
            y_base = sy + x_local * sin_yaw + y_local * cos_yaw
            range_base = math.hypot(x_base, y_base)
            angle_base = math.atan2(y_base, x_base)
            bin_idx = round((angle_base - out_angle_min) / out_angle_increment)
            bin_idx %= num_samples
            if range_base < merged[bin_idx]:
                merged[bin_idx] = range_base


def main(args=None):
    rclpy.init(args=args)
    node = LidarMergerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
