"""
Debug script — prints RIGHT leader + RIGHT follower joint angles every tick.

Bypasses record_loop entirely so we get raw, per-tick numbers with no rerun
overhead. Use this to catch the exact moment the right follower "flies back"
during a pick attempt — watch for either:
  (a) a sudden jump near 0/4095 in one joint  -> encoder wraparound
  (b) a gap/repeat in the printed numbers      -> comms dropout mid-motion
  (c) neither, but the arm still snaps         -> software/config issue

Run:  python debug_right_arm.py
Ctrl-C to stop.
"""

import time
from config import make_follower, make_leader

robot = make_follower()
teleop = make_leader()

robot.connect()
teleop.connect()

print("Move the RIGHT leader through the pick motion now. Ctrl-C to stop.\n")
print(f"{'tick':>5}  {'joint':<14}  {'leader':>8}  {'follower':>8}  {'diff':>8}")

tick = 0
try:
    while True:
        action = teleop.get_action()
        robot.send_action(action)
        obs = robot.get_observation()

        # Pull only right-side joints. Prefix may be 'right_' per bimanual.py's
        # left/right prefixing — adjust here if your key names differ.
        right_leader = {k: v for k, v in action.items() if k.startswith("right_")}
        right_follower = {k: v for k, v in obs.items() if k.startswith("right_")}

        for joint in sorted(right_leader.keys()):
            lval = right_leader.get(joint)
            fjoint = joint  # observation keys may match action keys directly
            fval = right_follower.get(fjoint, right_follower.get(joint.replace(".pos", "")))
            diff = None
            try:
                diff = round(float(lval) - float(fval), 1) if fval is not None else None
            except (TypeError, ValueError):
                pass
            print(f"{tick:>5}  {joint:<14}  {str(lval):>8}  {str(fval):>8}  {str(diff):>8}")

        tick += 1
        time.sleep(0.05)  # ~20Hz, matches FPS
except KeyboardInterrupt:
    print("\nstopped")
finally:
    robot.disconnect()
    teleop.disconnect()
