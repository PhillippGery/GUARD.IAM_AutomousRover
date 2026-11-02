#!/usr/bin/env python3
"""
Summarise stop_test.py output. Works on the time-series files, so it also
reads older runs that lack the newer trials.csv columns.

    python3 analyze_stop_tests.py ~/stop_tests/bldc_2026* [~/stop_tests/bridge_2026*]

Per (label, axis, speed): mean stop distance and settle time (from odom),
odom total vs tape (scale check), and speed tracking (odom speed in the
last 0.5 s of the drive vs the commanded speed).
"""
import csv, glob, math, os, sys
from collections import defaultdict
from statistics import mean


def metrics(ts_path, axis):
    rows = list(csv.DictReader(open(ts_path)))
    ph = [r['phase'] for r in rows]
    i0, i1 = ph.index('drive'), ph.index('stop')
    g = lambda k: [float(r[k]) for r in rows]
    t, x, y, yaw = g('t'), g('x'), g('y'), g('yaw')
    if axis == 'rot':
        total, stop = abs(yaw[-1] - yaw[i0]), abs(yaw[-1] - yaw[i1])
        v = [abs(float(r['wz'])) for r in rows[i0:i1]]
    else:
        total = math.hypot(x[-1] - x[i0], y[-1] - y[i0])
        stop = math.hypot(x[-1] - x[i1], y[-1] - y[i1])
        v = [math.hypot(float(r['vx']), float(r['vy'])) for r in rows[i0:i1]]
    n = max(1, int(0.5 / (t[i1] - t[i0]) * len(v))) if i1 > i0 else 1
    return total, stop, mean(v[-n:])


def main(dirs):
    groups = defaultdict(list)
    for d in dirs:
        label = os.path.basename(d.rstrip('/')).split('_')[0]
        for r in csv.DictReader(open(os.path.join(d, 'trials.csv'))):
            n, axis, sp = r['trial'], r['axis'], float(r['cmd_speed'])
            ts = os.path.join(d, f'ts_{n}.csv')
            if not os.path.exists(ts):
                continue
            total, stop, vss = metrics(ts, axis)
            tape = (r.get('tape_value') or r.get('tape_dist_m') or '').replace('\\', '')
            try:
                tape = float(tape)
            except ValueError:
                tape = float('nan')
            if axis == 'rot' and r.get('tape_unit') == 'deg':
                tape = math.radians(tape)
            groups[(label, axis, sp)].append(
                (stop, float(r['time_to_still_s']), total, tape, vss / sp))
    print(f"{'label':8s}{'axis':8s}{'cmd':>5s} {'n':>2s} {'stop':>8s} {'t_still':>8s} "
          f"{'odom_tot':>9s} {'tape':>7s} {'tape/odom':>9s} {'v/cmd':>6s}")
    for (label, axis, sp), v in sorted(groups.items()):
        c = list(zip(*v))
        tape = mean([a for a in c[3] if a == a]) if any(a == a for a in c[3]) else float('nan')
        u = 'rad' if axis == 'rot' else 'm'
        print(f"{label:8s}{axis:8s}{sp:5.1f} {len(v):2d} {mean(c[0]):7.3f}{u[0]} {mean(c[1]):7.2f}s "
              f"{mean(c[2]):8.3f}{u[0]} {tape:7.3f} {tape / mean(c[2]):9.2f} {mean(c[4]):6.2f}")
    print('\nstop = distance (m) or angle (rad) travelled after the zero command; '
          't_still = time until still; tape/odom ~1.0 means odom is trustworthy; '
          'v/cmd = achieved odom speed / commanded speed.')


if __name__ == '__main__':
    main([d for a in sys.argv[1:] for d in glob.glob(os.path.expanduser(a))] or sys.exit(__doc__))
