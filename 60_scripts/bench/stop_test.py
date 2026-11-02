#!/usr/bin/env python3
"""
Stop-test logger for the GUARD.IAM drive base.

Drives a fixed velocity for --drive-sec, commands zero, and logs until the
base is still. Run once per drive node on the same floor, same day:

    guardiam_hw      ->  python3 stop_test.py --label bridge
    guardiam_hw_bldc ->  python3 stop_test.py --label bldc

Quick scale check first (3 drives at 0.2 m/s, compares tape vs odom):

    python3 stop_test.py --label bldc_cal --calibrate

Needs the hardware launch running (drive node + mecanum_kinematics_node).
Nav2 does not need to run. Clear ~3 m ahead and to the right.

TAPE MEASUREMENT (the ground truth, odom cannot see slip):
  fwd / strafe: total distance, in metres, from the start mark to the same
                corner of the robot after it stops (NOT just the part after
                the command went to zero).
  rot:          total rotation in degrees (mark a line on the floor along
                the robot's side before the trial; read the angle after).

Output: ~/stop_tests/<label>_<timestamp>/
    meta.txt     battery, commit, args, notes
    trials.csv   one row per trial
    ts_<n>.csv   50 Hz time series: command, commanded RPM, measured RPM x4,
                 odom pose and body velocity
"""
import argparse, csv, math, os, subprocess, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

AXES = {'fwd': (1, 0, 0), 'strafe': (0, 1, 0), 'rot': (0, 0, 1)}
TICK = 0.02  # 50 Hz command and log rate
NAN = float('nan')


class StopTest(Node):
    def __init__(self):
        super().__init__('stop_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(Float32MultiArray, '/wheel_rpm', self._cmd_rpm, 10)
        self.create_subscription(Float32MultiArray, '/wheel_rpm_measured', self._meas_rpm, 10)
        self.odom = None
        self.cmd_rpm = [NAN] * 4
        self.meas_rpm = None
        self.rows = []
        self.t0 = time.time()

    def _odom(self, m):
        p, t = m.pose.pose, m.twist.twist
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (p.position.x, p.position.y, yaw, t.linear.x, t.linear.y, t.angular.z)

    def _cmd_rpm(self, m):
        self.cmd_rpm = list(m.data)[:4]

    def _meas_rpm(self, m):
        self.meas_rpm = list(m.data)[:4]

    def send(self, c):
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = float(c[0]), float(c[1]), float(c[2])
        self.pub.publish(m)

    def tick(self, cmd, phase):
        """Publish cmd, spin for one 20 ms period, log one row."""
        self.send(cmd)
        end = time.time() + TICK
        while True:
            left = end - time.time()
            if left <= 0:
                break
            rclpy.spin_once(self, timeout_sec=left)
        if self.odom is not None:
            self.rows.append([round(time.time() - self.t0, 4), phase, *cmd,
                              *self.cmd_rpm, *(self.meas_rpm or [NAN] * 4), *self.odom])

    def wait_ready(self, timeout=6.0):
        end = time.time() + timeout
        while time.time() < end and (self.odom is None or self.meas_rpm is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        missing = []
        if self.odom is None:
            missing.append('/odom')
        if self.meas_rpm is None:
            missing.append('/wheel_rpm_measured (rebuild with cb, and restart the hardware launch)')
        return missing


def yaw_diff(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def run_trial(node, axis, speed, args):
    cmd = [a * speed for a in AXES[axis]]
    node.rows = []
    node.t0 = time.time()
    for _ in range(int(0.5 / TICK)):
        node.tick([0, 0, 0], 'settle')
    s = node.odom
    for _ in range(int(args.drive_sec / TICK)):
        node.tick(cmd, 'drive')
    d = node.odom                      # pose when zero is commanded
    t_zero = time.time()
    still_since = None
    t_still = NAN
    while time.time() - t_zero < args.stop_window:
        node.tick([0, 0, 0], 'stop')
        sp = math.hypot(node.odom[3], node.odom[4]) + 0.2 * abs(node.odom[5])
        if sp < args.still_thresh:
            still_since = still_since or time.time()
            if time.time() - still_since > 0.5:
                t_still = still_since - t_zero
                break
        else:
            still_since = None
    for _ in range(int(0.3 / TICK)):
        node.tick([0, 0, 0], 'post')
    f = node.odom
    if axis == 'rot':
        total, drive, stop = yaw_diff(f[2], s[2]), yaw_diff(d[2], s[2]), yaw_diff(f[2], d[2])
    else:
        total = math.hypot(f[0] - s[0], f[1] - s[1])
        drive = math.hypot(d[0] - s[0], d[1] - s[1])
        stop = math.hypot(f[0] - d[0], f[1] - d[1])
    return total, drive, stop, yaw_diff(f[2], d[2]), t_still


def ask_tape(axis):
    unit = 'deg' if axis == 'rot' else 'm'
    prompt = ('  total rotation in degrees' if axis == 'rot'
              else '  total distance from start mark in metres') + ' (blank to skip): '
    raw = input(prompt).replace('\\', '').strip()
    try:
        return float(raw), unit
    except ValueError:
        return NAN, unit


def git_rev():
    try:
        d = os.path.dirname(os.path.abspath(__file__))
        return subprocess.check_output(['git', '-C', d, 'rev-parse', '--short', 'HEAD'],
                                       text=True).strip()
    except Exception:
        return 'unknown'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', required=True, help='bridge or bldc (or anything)')
    ap.add_argument('--axes', default='fwd,strafe,rot')
    ap.add_argument('--speeds', default='0.2,0.4,0.6', help='m/s (rad/s for rot)')
    ap.add_argument('--repeats', type=int, default=3)
    ap.add_argument('--drive-sec', type=float, default=1.5)
    ap.add_argument('--stop-window', type=float, default=4.0)
    ap.add_argument('--still-thresh', type=float, default=0.01)
    ap.add_argument('--no-measure', action='store_true', help='skip tape prompts')
    ap.add_argument('--calibrate', action='store_true',
                    help='3 forward drives at 0.2 m/s, report odom-vs-tape scale')
    args = ap.parse_args()
    if args.calibrate:
        args.axes, args.speeds, args.repeats = 'fwd', '0.2', 3
        args.no_measure = False

    rclpy.init()
    node = StopTest()
    missing = node.wait_ready()
    if missing:
        print('Not receiving:', ', '.join(missing))
        rclpy.shutdown()
        return

    out = os.path.expanduser(f'~/stop_tests/{args.label}_{time.strftime("%Y%m%d_%H%M%S")}')
    os.makedirs(out, exist_ok=True)
    volts = input('Battery pack voltage now (e.g. 25.2): ')
    notes = input('Floor / anything unusual (optional): ')
    with open(os.path.join(out, 'meta.txt'), 'w') as f:
        f.write(f'label: {args.label}\ndate: {time.ctime()}\ncommit: {git_rev()}\n'
                f'battery_start_v: {volts}\nnotes: {notes}\nargs: {vars(args)}\n')

    tf = open(os.path.join(out, 'trials.csv'), 'w', newline='')
    tw = csv.writer(tf)
    tw.writerow(['trial', 'axis', 'cmd_speed', 'rep', 'odom_total', 'odom_drive', 'odom_stop',
                 'odom_stop_dyaw_rad', 'time_to_still_s', 'tape_value', 'tape_unit'])
    n, ratios = 0, []
    try:
        for axis in args.axes.split(','):
            for sp in [float(s) for s in args.speeds.split(',')]:
                for rep in range(args.repeats):
                    input(f'[{axis} @ {sp}  rep {rep+1}/{args.repeats}] Robot on start mark, '
                          f'floor clear, spotter ready. Enter to go...')
                    n += 1
                    total, drive, stop, dyaw, tstill = run_trial(node, axis, sp, args)
                    with open(os.path.join(out, f'ts_{n}.csv'), 'w', newline='') as f:
                        w = csv.writer(f)
                        w.writerow(['t', 'phase', 'cmd_vx', 'cmd_vy', 'cmd_wz',
                                    'cmdrpm_fl', 'cmdrpm_fr', 'cmdrpm_bl', 'cmdrpm_br',
                                    'measrpm_fl', 'measrpm_fr', 'measrpm_bl', 'measrpm_br',
                                    'x', 'y', 'yaw', 'vx', 'vy', 'wz'])
                        w.writerows(node.rows)
                    unit = 'deg' if axis == 'rot' else 'm'
                    odom_t = math.degrees(total) if axis == 'rot' else total
                    print(f'  odom total {odom_t:.3f} {unit}, stop part {stop:.3f}, '
                          f'still after {tstill:.2f} s')
                    tape, tunit = (NAN, unit) if args.no_measure else ask_tape(axis)
                    if tape == tape and odom_t > 0:
                        ratios.append(tape / odom_t)
                    tw.writerow([n, axis, sp, rep + 1, round(total, 4), round(drive, 4),
                                 round(stop, 4), round(dyaw, 4), round(tstill, 3), tape, tunit])
                    tf.flush()
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(10):
            node.tick([0, 0, 0], 'abort')
        tf.close()
        if args.calibrate and ratios:
            r = sum(ratios) / len(ratios)
            print(f'\nCalibration: tape / odom = {r:.3f}  (1.000 means odom is right)')
            print('  If this is not about 1.0, scale position_rescale_factor (bldc) or '
                  'rescale_factor (bridge) by this number, rebuild, and run --calibrate again.')
        node.destroy_node()
        rclpy.shutdown()
        print('saved to', out)


if __name__ == '__main__':
    main()
