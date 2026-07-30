"""
Run a trained ACT policy on the real bimanual robot (GUARD.IAM pick-and-place).

Uses the direct processor path proven correct by the dataset-replay test
(pred matched demonstrated actions ~1-3 deg/joint):
  build_dataset_frame(obs) -> transition{OBSERVATION: frame} -> preprocessor
  -> policy.select_action -> postprocessor({'action': act}) -> send.

Normalization comes from the checkpoint's SAVED processors (from_pretrained), the
exact stats training used — that was the whole fix.

CRITICAL: config.py cameras must match collection (3 cams, MJPG, 20 fps, rotations).
Reseat the LEFT FOLLOWER first.

Run:
    python infer.py --checkpoint outputs/train/guardian_act/checkpoints/100000/pretrained_model
"""

import argparse
import time

import numpy as np
import torch

from config import make_follower, FPS
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.processor import PolicyProcessorPipeline, TransitionKey
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.feature_utils import build_dataset_frame
from lerobot.utils.visualization_utils import init_rerun

FEATURES_REPO_ID = "vedant/guardian_pick_place"
DEVICE = "cuda"
PRE_CFG = "policy_preprocessor.json"
POST_CFG = "policy_postprocessor.json"


def busy_wait(seconds):
    if seconds > 0:
        time.sleep(seconds)


def extract_action(post_out, action_names):
    a = post_out
    if isinstance(a, dict):
        a = a.get("action", a)
    if torch.is_tensor(a):
        a = a.squeeze(0).detach().cpu().numpy()
    else:
        a = np.asarray(a).squeeze()
    return a


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", required=True)
    p.add_argument("--seconds", type=float, default=60.0)
    p.add_argument("--task", type=str,
                   default="Pick up the object and place it in the bin")
    return p.parse_args()


def main():
    args = parse_args()
    ckpt = args.checkpoint

    robot = make_follower()
    features = LeRobotDataset(FEATURES_REPO_ID).features
    action_names = features["action"]["names"]
    print("[infer] action names:", action_names, flush=True)

    print(f"[infer] loading policy from: {ckpt}")
    policy = ACTPolicy.from_pretrained(ckpt)
    policy.config.device = DEVICE
    policy.to(DEVICE)
    policy.eval()
    policy.reset()

    preprocessor = PolicyProcessorPipeline.from_pretrained(ckpt, PRE_CFG)
    postprocessor = PolicyProcessorPipeline.from_pretrained(ckpt, POST_CFG)
    print("[infer] loaded saved pre/post processors from checkpoint", flush=True)

    init_rerun(session_name="bimanual_infer")

    robot.connect()
    print(f"[infer] running for {args.seconds:.0f}s at {FPS} fps on {DEVICE}. "
          f"Ctrl-C to stop.", flush=True)

    step = 0
    try:
        while step < args.seconds * FPS:
            t0 = time.perf_counter()

            obs = robot.get_observation()
            observation_frame = build_dataset_frame(features, obs, prefix="observation")

            if step == 0:
                print("[infer] first inference (CUDA warmup ~30-60s)...", flush=True)

            # Build transition with the enum key the processor reads.
            transition = {TransitionKey.OBSERVATION: observation_frame,
                          TransitionKey.COMPLEMENTARY_DATA: {"task": args.task}}
            processed = preprocessor(transition)

            with torch.inference_mode():
                action = policy.select_action(processed)

            # Postprocessor wants a dict (proven by replay test).
            action = postprocessor({"action": action})
            arr = extract_action(action, action_names)
            action_dict = {n: float(arr[i]) for i, n in enumerate(action_names)}

            if step < 8:
                print(f"[infer] step {step} action:", np.round(arr, 1), flush=True)

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
        print("\n[infer] stopped by user.")
    finally:
        try:
            robot.disconnect()
        except Exception as e:
            print(f"[infer] disconnect: {type(e).__name__} ignored: {e}")
        print("[infer] done.")


if __name__ == "__main__":
    main()