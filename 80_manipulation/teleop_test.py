"""
Bimanual teleop test WITH live streaming to rerun.
==================================================
Drive both followers from both leaders and SEE the wrist cameras + arm state in
rerun. No recording.

Why the old version showed nothing: its loop only called get_action/send_action.
It never read an observation, so nothing was ever handed to rerun. init_rerun
opens the viewer but does not pull frames by itself.

This version uses LeRobot's record_loop with display_data=True — the same path
collect.py uses — which reads observations each tick and logs the camera frames,
state, and action to rerun. Passing no `dataset` means nothing is written to disk.

Run:  python teleop_test.py
Ctrl-C to stop.

Use it to confirm, before collecting:
  - both wrist feeds are live and framed on the workspace
  - each follower gripper FULLY opens AND closes with the new grippers
"""

from config import make_follower, make_leader, FPS
from lerobot.scripts.lerobot_record import record_loop
from lerobot.utils.keyboard_input import init_keyboard_listener
from lerobot.utils.visualization_utils import init_rerun
from lerobot.processor import make_default_processors

robot = make_follower()
teleop = make_leader()

_, events = init_keyboard_listener()
init_rerun(session_name="bimanual_teleop_test")
teleop_action_proc, robot_action_proc, robot_obs_proc = make_default_processors()

robot.connect()
teleop.connect()

print("Teleop + live view running. Move the leaders; watch the followers and the")
print("camera feeds in the rerun window.")
print("CHECK: does each follower gripper fully open AND fully close? "
      "If it stops short or strains, recalibrate that follower (new grippers).")

try:
    # record_loop runs for control_time_s then returns; loop it so this stays
    # live until you Ctrl-C. display_data=True is what streams to rerun. No
    # `dataset` arg => nothing is recorded.
    while True:
        record_loop(
            robot=robot,
            teleop=teleop,
            events=events,
            fps=FPS,
            teleop_action_processor=teleop_action_proc,
            robot_action_processor=robot_action_proc,
            robot_observation_processor=robot_obs_proc,
            control_time_s=60,
            single_task="teleop test",
            display_data=True,
        )
except KeyboardInterrupt:
    pass
finally:
    robot.disconnect()
    teleop.disconnect()
    print("Disconnected.")