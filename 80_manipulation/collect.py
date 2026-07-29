"""
Bimanual data collection — records a LeRobot dataset for ACT training.
Cameras + 12-dim action + 12-dim state come from config.py (shared with
teleop_test.py and inference, so nothing drifts).

FPS comes from config.py. Whatever it is (currently 20), the policy is trained
and run at that same rate — don't mix rates across a dataset.

Set NUM_EPISODES = 3 for the DRY RUN (collect -> shape-check -> train once)
before committing to a full ~50-episode session.

Run:  python collect.py

Controls during recording (foot pedal or keyboard):
    RIGHT ARROW  keep this episode, continue
    LEFT ARROW   scrap & re-record this episode (discards the take)
    ESC          stop the whole session
"""

import os
import shutil
from pathlib import Path

from config import make_follower, make_leader, FPS
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from lerobot.scripts.lerobot_record import record_loop
from lerobot.utils.control_utils import init_keyboard_listener
from lerobot.utils.visualization_utils import init_rerun
from lerobot.processor import make_default_processors
from lerobot.utils.utils import log_say

# ── Settings ───────────────────────────────────────────────────────────────────
NUM_EPISODES = 75                          # 3 for dry run, ~50 for real collection
EPISODE_TIME_SEC = 60                     # give yourself time — right arrow ends early
RESET_TIME_SEC = 20                       # time to reset the scene between episodes
REPO_ID = "vedant/guardian_pick_place"    # any local name; keep practice runs on a
                                          # different id (e.g. guardian_practice)
TASK = "Pick up the object and place it in the bin"   # must match what you demo

# Practice mode: wipe any existing dataset with this REPO_ID before running, so
# repeated dry runs don't hit the "dataset exists" error. FLIP THIS TO FALSE
# before real collection so a stray re-run can't delete a session you care about.
WIPE_BEFORE_RUN = True
# ───────────────────────────────────────────────────────────────────────────────


def wipe_existing_dataset():
    """Delete the dataset dir for REPO_ID (keys off REPO_ID only, never a broader
    path). No-op if it doesn't exist."""
    root = Path.home() / ".cache" / "huggingface" / "lerobot" / REPO_ID
    if root.exists():
        shutil.rmtree(root)
        print(f"[wipe] removed existing dataset at {root}")


def print_banner():
    bar = "=" * 60
    print(bar)
    print(f"  GUARDIAN COLLECTION  —  {REPO_ID}")
    print(f"  {NUM_EPISODES} episodes @ {FPS} fps   |   task: {TASK}")
    if WIPE_BEFORE_RUN:
        print("  MODE: practice (dataset is wiped each run)")
    print(bar)
    print("  CONTROLS (foot pedal or keyboard):")
    print("    RIGHT ARROW  ->  keep this episode, continue")
    print("    LEFT ARROW   ->  scrap & re-record this episode")
    print("    ESC          ->  stop the whole session")
    print(bar, flush=True)


if WIPE_BEFORE_RUN:
    wipe_existing_dataset()

robot = make_follower()
teleop = make_leader()

action_features = hw_to_dataset_features(robot.action_features, "action")
obs_features = hw_to_dataset_features(robot.observation_features, "observation")
dataset_features = {**action_features, **obs_features}

dataset = LeRobotDataset.create(
    repo_id=REPO_ID,
    fps=FPS,
    features=dataset_features,
    robot_type=robot.name,
    use_videos=True,
    image_writer_threads=4,
)

_, events = init_keyboard_listener()
init_rerun(session_name="bimanual_collect")
teleop_action_proc, robot_action_proc, robot_obs_proc = make_default_processors()

robot.connect()
teleop.connect()

print_banner()

# ── Episode loop ────────────────────────────────────────────────────────────────
# while-loop (not for-loop) so a scrapped episode does NOT advance the counter —
# left-arrow re-records the SAME episode index instead of consuming it.
episode_idx = 0
stop = False

while episode_idx < NUM_EPISODES and not stop:
    print(f"\n[EPISODE {episode_idx + 1}/{NUM_EPISODES}] recording {EPISODE_TIME_SEC}s "
          f"— go. (RIGHT=keep, LEFT=redo, ESC=stop)", flush=True)
    log_say(f"Recording episode {episode_idx + 1} of {NUM_EPISODES}", blocking=True)

    events["rerecord_episode"] = False
    events["exit_early"] = False

    record_loop(
        robot=robot,
        teleop=teleop,
        events=events,
        fps=FPS,
        teleop_action_processor=teleop_action_proc,
        robot_action_processor=robot_action_proc,
        robot_observation_processor=robot_obs_proc,
        dataset=dataset,
        control_time_s=EPISODE_TIME_SEC,
        single_task=TASK,
        display_data=True,
    )

    # ESC — stop the whole session (don't save the in-progress take)
    if events.get("stop_recording", False):
        print("\n[STOP] Session stopped by user.", flush=True)
        stop = True
        break

    # LEFT ARROW — scrap this take. Clear the buffer (incl. this take's image
    # files) and loop back WITHOUT incrementing, so the same episode is redone.
    if events.get("rerecord_episode", False):
        print(f"[SCRAP] Discarding episode {episode_idx + 1} — record it again.", flush=True)
        log_say("Re-recording", blocking=True)
        dataset.clear_episode_buffer(delete_images=True)
        events["rerecord_episode"] = False
        continue

    # RIGHT ARROW or time expired — keep it.
    dataset.save_episode()
    print(f"[EPISODE {episode_idx + 1}/{NUM_EPISODES}] saved.", flush=True)
    episode_idx += 1

    # Scene reset between kept episodes (skip after the last one).
    if episode_idx < NUM_EPISODES:
        print(f"[RESET] {RESET_TIME_SEC}s — reset the scene to a fresh start.", flush=True)
        log_say("Reset the scene", blocking=True)
        events["exit_early"] = False
        record_loop(
            robot=robot,
            teleop=teleop,
            events=events,
            fps=FPS,
            teleop_action_processor=teleop_action_proc,
            robot_action_processor=robot_action_proc,
            robot_observation_processor=robot_obs_proc,
            control_time_s=RESET_TIME_SEC,
            single_task=TASK,
            display_data=False,
        )
        if events.get("stop_recording", False):
            print("\n[STOP] Session stopped during reset.", flush=True)
            stop = True

# ── Finalize + exit ───────────────────────────────────────────────────────────
# Finalize FIRST — write the episodes index while the interpreter is alive,
# before anything else can throw and skip it.
for _m in ("finalize", "consolidate"):
    _fn = getattr(dataset, _m, None)
    if callable(_fn):
        print(f"[finalize] {_m}() ...", flush=True)
        _fn()
        break
else:
    print("[finalize] no finalize/consolidate method found — tell Claude", flush=True)

# Disconnect best-effort — a follower that dropped off the bus makes disconnect()
# raise, which must NOT prevent the index write above or the clean exit.
for _dev in (robot, teleop):
    try:
        _dev.disconnect()
    except Exception as e:
        print(f"[disconnect] {type(e).__name__} ignored: {e}", flush=True)

print("\n" + "=" * 60)
print(f"  DONE — {episode_idx} episode(s) @ {FPS} fps recorded to {REPO_ID}")
print("=" * 60, flush=True)

# Hard-exit to skip the interpreter teardown that pyarrow crashes on. Everything
# is already written and finalized above, so exiting abruptly is safe here.
os._exit(0)