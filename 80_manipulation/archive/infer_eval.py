"""
infer_eval.py — evaluate a trained ACT policy on the real bimanual arms.
=========================================================================
Runs the POLICY on the followers (no leaders, no teleop, NO dataset recording).
This is a lean evaluation tool: point it at a checkpoint, watch the arms attempt
the task in rerun, Ctrl-C when done.

WHY THIS EXISTS (vs the old infer.py):
  infer.py loaded normalization with
      PolicyProcessorPipeline.from_pretrained(ckpt, "policy_preprocessor.json")
  and on the arms it collapsed to a halfway pose then went limp — the classic
  normalization-mismatch signature. bimanual_inference.py, which reached toward
  the object, instead loaded normalization with
      make_pre_post_processors(policy_cfg=policy, pretrained_path=ckpt)
  which pulls the checkpoint's OWN saved stats. This script keeps infer.py's
  0.6.1-compatible manual send-action loop but uses that proven normalization
  path. (record_loop(policy=...) no longer exists in lerobot 0.6.1, so the
  manual loop is required.)

TWO MODES:
  --check     NO robot. Loads N training frames and prints predicted vs
              demonstrated actions. RUN THIS FIRST. If pred tracks true within a
              few degrees per joint, normalization is correct and the hardware
              rollout is safe to try. If pred is flat / collapsed, stop — the
              checkpoint or its stats are the problem, not the arms.
  (default)   Drives the arms for --seconds. Ctrl-C to stop.

BEFORE HARDWARE:
  - Reseat the flaky LEFT arm (USB + power).
  - Keep a hand on Ctrl-C on the first rollout — a fresh/bad policy can snap
    to a pose abruptly.

Run:
  # desk check first (no arms):
  python infer_eval.py --checkpoint outputs/train/guardian_act/checkpoints/010000/pretrained_model --check
  # then on the arms:
  python infer_eval.py --checkpoint outputs/train/guardian_act/checkpoints/010000/pretrained_model
"""

import argparse
import time

import numpy as np
import torch

from config import make_follower, FPS
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.processor import TransitionKey
from lerobot.processor.converters import create_transition
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.feature_utils import build_dataset_frame
from lerobot.utils.visualization_utils import init_rerun

FEATURES_REPO_ID = "vedant/guardian_pick_place"   # dataset used only for feature schema
DEVICE = "cpu"


def busy_wait(seconds):
    if seconds > 0:
        time.sleep(seconds)


def extract_action(post_out):
    """Pull a flat numpy action vector out of whatever the postprocessor returns."""
    a = post_out
    if isinstance(a, dict):
        a = a.get("action", a)
    if torch.is_tensor(a):
        a = a.squeeze(0).detach().cpu().numpy()
    else:
        a = np.asarray(a).squeeze()
    return a


def load_policy_and_processors(ckpt):
    """Load the policy AND the checkpoint's own saved normalization stats.
    This is the path that drove the arms correctly (make_pre_post_processors),
    NOT infer.py's from_pretrained(json) path that collapsed."""
    policy = ACTPolicy.from_pretrained(ckpt)
    policy.config.device = DEVICE
    policy.to(DEVICE)
    policy.eval()
    policy.reset()
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy,
        pretrained_path=ckpt,
    )
    return policy, preprocessor, postprocessor


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", required=True,
                   help="a checkpoint's pretrained_model dir")
    p.add_argument("--seconds", type=float, default=60.0)
    p.add_argument("--task", type=str,
                   default="Pick up the object and place it in the bin")
    p.add_argument("--check", action="store_true",
                   help="no robot: print pred vs true on training frames, then exit")
    p.add_argument("--check-n", type=int, default=6,
                   help="how many frames to check in --check mode")
    return p.parse_args()


# ─────────────────────────────────────────────────────────────────────────────
# --check : verify normalization WITHOUT the robot
# ─────────────────────────────────────────────────────────────────────────────
def run_check(ckpt, n):
    print(f"[check] loading dataset schema from: {FEATURES_REPO_ID}", flush=True)
    ds = LeRobotDataset(FEATURES_REPO_ID)
    action_names = ds.features["action"]["names"]

    print(f"[check] loading policy + checkpoint stats from: {ckpt}", flush=True)
    policy, preprocessor, postprocessor = load_policy_and_processors(ckpt)

    print(f"\n[check] predicted vs demonstrated actions on {n} training frames.")
    print("[check] pred should track true within a few degrees per joint.")
    print("[check] if pred is flat/identical across frames -> normalization is")
    print("[check] broken (this is the halfway-pose cause). Stop before hardware.\n")

    worst = 0.0
    for i in range(n):
        f = ds[i]
        obs = {k: f[k] for k in f if k.startswith("observation")}
        transition = create_transition(
            observation=obs,
            complementary_data={"task": f.get("task", "")},
        )
        with torch.inference_mode():
            processed = preprocessor(transition)
            action = policy.select_action(processed)
        pred = extract_action(postprocessor({"action": action}))
        true = f["action"].numpy() if torch.is_tensor(f["action"]) else np.asarray(f["action"])
        diff = np.abs(pred - true)
        worst = max(worst, float(diff.max()))
        print(f"  frame {i}")
        print(f"    pred {np.round(pred, 1)}")
        print(f"    true {np.round(true, 1)}")
        print(f"    max |Δ| {diff.max():.1f}")
    print(f"\n[check] worst per-joint error across {n} frames: {worst:.1f}")
    if worst < 10:
        print("[check] PASS — pred tracks true. Normalization looks correct; "
              "hardware rollout is reasonable to try.")
    else:
        print("[check] SUSPECT — large error. Either the checkpoint is undertrained "
              "(fine, just early) or normalization is off. If pred is nearly identical "
              "across frames, it's normalization — do NOT run on the arms yet.")


# ─────────────────────────────────────────────────────────────────────────────
# default : drive the arms with the policy (rerun on, no recording)
# ─────────────────────────────────────────────────────────────────────────────
def run_on_robot(ckpt, seconds, task):
    robot = make_follower()
    features = LeRobotDataset(FEATURES_REPO_ID).features
    action_names = features["action"]["names"]
    print("[eval] action names:", action_names, flush=True)

    print(f"[eval] loading policy + checkpoint stats from: {ckpt}", flush=True)
    policy, preprocessor, postprocessor = load_policy_and_processors(ckpt)

    init_rerun(session_name="bi_eval")
    robot.connect()
    print(f"[eval] running policy for {seconds:.0f}s at {FPS} fps on {DEVICE}. "
          f"Ctrl-C to stop. KEEP A HAND ON IT.", flush=True)

    step = 0
    try:
        while step < seconds * FPS:
            t0 = time.perf_counter()

            obs = robot.get_observation()
            observation_frame = build_dataset_frame(features, obs, prefix="observation")

            if step == 0:
                print("[eval] first inference (CUDA warmup ~30-60s)...", flush=True)

            transition = create_transition(
                observation=observation_frame,
                complementary_data={"task": task},
            )
            with torch.inference_mode():
                processed = preprocessor(transition)
                action = policy.select_action(processed)
            arr = extract_action(postprocessor({"action": action}))
            action_dict = {n: float(arr[i]) for i, n in enumerate(action_names)}

            # First few steps: print actions so a collapse is obvious immediately.
            if step < 8:
                print(f"[eval] step {step} action:", np.round(arr, 1), flush=True)

            try:
                import rerun as rr
                rr.set_time_sequence("step", step)
                for n in action_names:
                    rr.log(f"action/{n}", rr.Scalar(float(action_dict[n])))
                for k, v in obs.items():
                    if k.endswith(".pos"):
                        try:
                            rr.log(f"state/{k}", rr.Scalar(float(v)))
                        except (TypeError, ValueError):
                            pass
            except Exception:
                pass

            robot.send_action(action_dict)
            busy_wait(max(0.0, (1.0 / FPS) - (time.perf_counter() - t0)))
            step += 1
    except KeyboardInterrupt:
        print("\n[eval] stopped by user.")
    finally:
        try:
            robot.disconnect()
        except Exception as e:
            print(f"[eval] disconnect: {type(e).__name__} ignored: {e}")
        print("[eval] done.")


def main():
    args = parse_args()
    if args.check:
        run_check(args.checkpoint, args.check_n)
    else:
        run_on_robot(args.checkpoint, args.seconds, args.task)


if __name__ == "__main__":
    main()