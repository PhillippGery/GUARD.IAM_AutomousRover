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

Between episodes only (NOT during a recording take):
    SPACEBAR     pause for a break  (center foot pedal maps to SPACE via
                 pedal_bridge.py, so you can pause hands-free during scene reset)
"""

import os
import shutil
import threading
from pathlib import Path

from config import make_follower, make_leader, FPS
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.feature_utils import hw_to_dataset_features
from lerobot.scripts.lerobot_record import record_loop
from lerobot.utils.keyboard_input import init_keyboard_listener
from lerobot.utils.visualization_utils import init_rerun
from lerobot.processor import make_default_processors
from lerobot.utils.utils import log_say

# pynput ships with lerobot (its own keyboard listener uses it). We use it for a
# SEPARATE, passive spacebar watcher that only sets a flag — it never consumes or
# competes for the arrow/ESC keys that lerobot's record_loop listens for, because
# we only READ our flag between episodes, when record_loop is not running.
try:
    from pynput import keyboard as _pk
except Exception:                       # pragma: no cover
    _pk = None

# ── Settings ───────────────────────────────────────────────────────────────────
NUM_EPISODES =  50               # real collection
EPISODE_TIME_SEC = 60                     # give yourself time — right arrow ends early
RESET_TIME_SEC = 20                       # time to reset the scene between episodes
PAUSE_EVERY = 10                          # auto-pause for a break every N kept episodes
                                          # (0 disables auto-pause). SPACEBAR (or the
                                          # center pedal) also pauses between episodes.
REPO_ID = "guardian_strawberry_pick"   # real dataset
                                          # different id (e.g. guardian_practice)
TASK = "Pick up the strawberry and place it in the bin"

# Practice mode: wipe any existing dataset with this REPO_ID before running, so
# repeated dry runs don't hit the "dataset exists" error. FLIP THIS TO FALSE
# before real collection so a stray re-run can't delete a session you care about.
WIPE_BEFORE_RUN = False
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
    print("    SPACEBAR     ->  pause (between episodes only; center pedal maps here)")
    print(bar, flush=True)


# ── Spacebar pause: passive flag only ──────────────────────────────────────────
# A tiny listener sets _space_pressed when SPACE goes down. We ONLY read/clear it
# between episodes (record_loop not running), so it never competes with lerobot's
# own arrow/ESC listener during a take. If pynput is unavailable, spacebar-pause
# is simply disabled (auto-pause via PAUSE_EVERY still works).
_space_pressed = threading.Event()


def _on_press(key):
    if key == _pk.Key.space:
        _space_pressed.set()


def _start_space_listener():
    if _pk is None:
        print("[pause] pynput unavailable — spacebar pause off (PAUSE_EVERY still works).",
              flush=True)
        return None
    listener = _pk.Listener(on_press=_on_press)
    listener.daemon = True
    listener.start()
    return listener


def pause_for_break(next_episode_num):
    """Between-episode pause. Blocks on a terminal prompt (recording is stopped and
    the arms are idle here, so a plain input() is safe — it does NOT fight the
    arrow/pedal listener, which only matters during a recording loop).

    ENTER            -> resume, record the next episode
    type 'p' + ENTER -> extended pause: hold here until you press ENTER again
    type 'q' + ENTER -> stop the session cleanly (finalizes what's collected)

    Returns "go" to continue or "stop" to end the session.
    """
    log_say("Paused", blocking=False)
    print("\n" + "-" * 60, flush=True)
    print(f"  PAUSED before episode {next_episode_num}/{NUM_EPISODES}.", flush=True)
    print("    ENTER   -> resume and record the next episode", flush=True)
    print("    p+ENTER -> hold here for a longer break", flush=True)
    print("    q+ENTER -> stop the session (saves what you've collected)", flush=True)
    print("-" * 60, flush=True)
    while True:
        try:
            choice = input("  [pause] > ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\n[pause] interpreted as STOP.", flush=True)
            return "stop"
        if choice == "q":
            return "stop"
        if choice == "p":
            print("  [pause] holding — take your time. Press ENTER when ready.", flush=True)
            continue
        log_say("Resuming", blocking=True)
        return "go"


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
_space_listener = _start_space_listener()
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
    episode_idx += 1

    # Progress line + when the next auto-break lands, so the operator always
    # knows how far in they are and how long until they can step away.
    if PAUSE_EVERY:
        until_break = PAUSE_EVERY - (episode_idx % PAUSE_EVERY)
        if episode_idx % PAUSE_EVERY == 0 or episode_idx >= NUM_EPISODES:
            hint = "break coming up"
        else:
            hint = f"{until_break} more until the next break"
        print(f"[EPISODE {episode_idx}/{NUM_EPISODES}] saved  —  {hint}.", flush=True)
    else:
        print(f"[EPISODE {episode_idx}/{NUM_EPISODES}] saved.", flush=True)

    # Pause between episodes if EITHER: spacebar/center-pedal was pressed during
    # the take just finished, OR we've hit the PAUSE_EVERY auto-break. Checked
    # here (record_loop not running) so it never fights lerobot's key listener.
    space_asked = _space_pressed.is_set()
    _space_pressed.clear()
    auto_break = (PAUSE_EVERY and episode_idx < NUM_EPISODES
                  and episode_idx % PAUSE_EVERY == 0)
    if (space_asked or auto_break) and episode_idx < NUM_EPISODES:
        if space_asked:
            print("[pause] spacebar/pedal pause requested.", flush=True)
        if pause_for_break(episode_idx + 1) == "stop":
            print("\n[STOP] Session stopped from pause.", flush=True)
            stop = True
            break

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