#!/usr/bin/env python3
"""
Stop-test logger for the GUARD.IAM drive base.

Drives a fixed velocity, commands zero, and logs everything until the base
has stopped. Run once per drive node (guardiam_hw, then guardiam_hw_bldc),
same floor, same battery state, so the two runs are comparable.

    ros2 run / python3 stop_test.py --label bridge      # with guardiam_hw up
    python3 stop_test.py --label bldc                   # with guardiam_hw_bldc up

Needs guardiam_hw (or _hw_bldc) running, plus mecanum_kinematics_node
(guardian_hardware.launch.py starts both). Nav2 does NOT need to run.
Clear ~3 m of floor in the direction of travel.

Output: ~/stop_tests/<label>_<timestamp>/
    trials.csv        one row per trial (stop distance, stop time, ...)
    ts_<n>.csv        time series per trial: t, phase, cmd, wheel_rpm x4, odom x/y/yaw, vx/vy/wz
Odom is encoder-derived, so it shows what the wheels did, not true ground
motion. Mark the start with tape and use --measure to type in the tape-measured
stop distance per trial; that is your ground truth for slip.
"""
import argparse, csv, math, os, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

AXES = {'fwd': (1, 0, 0), 'strafe': (0, 1, 0), 'rot': (0, 0, 1)}


class StopTest(Node):
    def __init__(self):
        super().__init__('stop_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(Float32MultiArray, '/wheel_rpm', self._rpm, 10)
        self.odom = None
        self.rpm = [float('nan')] * 4

    def _odom(self, m):
        p, t = m.pose.pose, m.twist.twist
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (p.position.x, p.position.y, yaw, t.linear.x, t.linear.y, t.angular.z)

    def _rpm(self, m):
        self.rpm = list(m.data)[:4]

    def send(self, vx, vy, wz):
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = float(vx), float(vy), float(wz)
        self.pub.publish(m)

    def spin_for(self, sec, rate=50.0, cb=None):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=1.0 / rate)
            if cb:
                cb()


def run_trial(node, axis, speed, args, writer_ts):
    ax = AXES[axis]
    cmd = [a * speed for a in ax]
    t0 = time.time()
    rows = []

    def sample(phase, c):
        if node.odom is None:
            return
        rows.append([round(time.time() - t0, 4), phase, *c, *node.rpm, *node.odom])

    # settle, then drive
    node.spin_for(0.5, cb=lambda: (node.send(0, 0, 0), sample('settle', [0, 0, 0])))
    while node.odom is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.spin_for(args.drive_sec, cb=lambda: (node.send(*cmd), sample('drive', cmd)))
    # pose at the moment zero is commanded
    x0, y0, yaw0 = node.odom[0], node.odom[1], node.odom[2]
    t_stop_cmd = time.time() - t0
    # stop phase: command zero, log until speed ~0 for 0.5 s or timeout
    still_since = None
    t_end = time.time() + args.stop_window
    t_still = None
    while time.time() < t_end:
        node.send(0, 0, 0)
        rclpy.spin_once(node, timeout_sec=0.02)
        sample('stop', [0, 0, 0])
        if node.odom is None:
            continue
        sp = math.hypot(node.odom[3], node.odom[4]) + abs(node.odom[5]) * 0.2
        if sp < args.still_thresh:
            still_since = still_since or time.time()
            if time.time() - still_since > 0.5:
                t_still = still_since - t0
                break
        else:
            still_since = None
    node.spin_for(0.3, cb=lambda: (node.send(0, 0, 0), sample('post', [0, 0, 0])))
    xf, yf, yawf = node.odom[0], node.odom[1], node.odom[2]
    dist = math.hypot(xf - x0, yf - y0)
    dyaw = abs(math.atan2(math.sin(yawf - yaw0), math.cos(yawf - yaw0)))
    writer_ts.writerows(rows)
    return dist, dyaw, (t_still - t_stop_cmd) if t_still else float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', required=True, help='bridge or bldc (or anything)')
    ap.add_argument('--axes', default='fwd,strafe,rot')
    ap.add_argument('--speeds', default='0.2,0.4,0.6', help='m/s (rad/s for rot)')
    ap.add_argument('--repeats', type=int, default=3)
    ap.add_argument('--drive-sec', type=float, default=1.5)
    ap.add_argument('--stop-window', type=float, default=4.0)
    ap.add_argument('--still-thresh', type=float, default=0.01)
    ap.add_argument('--measure', action='store_true', help='prompt for tape-measured stop distance')
    args = ap.parse_args()

    rclpy.init()
    node = StopTest()
    out = os.path.expanduser(f'~/stop_tests/{args.label}_{time.strftime("%Y%m%d_%H%M%S")}')
    os.makedirs(out, exist_ok=True)
    tf = open(os.path.join(out, 'trials.csv'), 'w', newline='')
    tw = csv.writer(tf)
    tw.writerow(['trial', 'axis', 'cmd_speed', 'rep', 'odom_stop_dist_m', 'odom_stop_dyaw_rad',
                 'time_to_still_s', 'tape_dist_m'])
    n = 0
    try:
        for axis in args.axes.split(','):
            for sp in [float(s) for s in args.speeds.split(',')]:
                for rep in range(args.repeats):
                    input(f'[{axis} @ {sp}  rep {rep+1}/{args.repeats}] Reset robot to start mark, clear floor, press Enter...')
                    n += 1
                    with open(os.path.join(out, f'ts_{n}.csv'), 'w', newline='') as f:
                        w = csv.writer(f)
                        w.writerow(['t', 'phase', 'cmd_vx', 'cmd_vy', 'cmd_wz', 'rpm_fl', 'rpm_fr', 'rpm_bl',
                                    'rpm_br', 'x', 'y', 'yaw', 'vx', 'vy', 'wz'])
                        dist, dyaw, tstill = run_trial(node, axis, sp, args, w)
                    tape = input('  tape-measured stop distance past mark (m, blank to skip): ') if args.measure else ''
                    tw.writerow([n, axis, sp, rep + 1, round(dist, 4), round(dyaw, 4), round(tstill, 3), tape])
                    tf.flush()
                    print(f'  odom stop dist {dist:.3f} m, dyaw {dyaw:.3f} rad, still after {tstill:.2f} s')
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(10):
            node.send(0, 0, 0)
            rclpy.spin_once(node, timeout_sec=0.02)
        tf.close()
        node.destroy_node()
        rclpy.shutdown()
        print('saved to', out)


if __name__ == '__main__':
    main()
