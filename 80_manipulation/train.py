"""
Train an ACT policy on the collected GUARDIAN dataset.
======================================================
Thin wrapper around LeRobot's training entrypoint (full loop: checkpointing,
resume, logging). Kept as a wrapper on purpose so it tracks your installed
LeRobot version instead of duplicating its policy/training API.

>>> Flag names shift between LeRobot versions. If one is rejected, run:
        lerobot-train --help
    (or  python -m lerobot.scripts.lerobot_train --help )
    and adjust. Everything you'd change is in CONFIG below.

Run:  python train.py
"""

import shutil
import subprocess
import sys

# ── CONFIG ──────────────────────────────────────────────────────────────────
REPO_ID    = "vedant/guardian_pick_place"    # MUST match collect.py's REPO_ID
OUTPUT_DIR = "outputs/train/guardian_act"
JOB_NAME   = "guardian_act"
POLICY     = "act"
DEVICE     = "cuda"          # "cuda", "cpu", or "mps"
BATCH_SIZE = 8
# STEPS: for the 3-episode DRY RUN just prove the pipeline runs — set ~2000.
# For a real ~50-episode set, ACT usually wants 80k–100k+.
STEPS      = 100_000
SAVE_FREQ  = 10_000          # checkpoint every N steps
LOG_FREQ   = 200
USE_WANDB  = False
# ────────────────────────────────────────────────────────────────────────────

# Prefer the console entrypoint; fall back to the module (name mirrors
# lerobot_record, so lerobot_train is the likely module in your version).
if shutil.which("lerobot-train"):
    base = ["lerobot-train"]
else:
    base = [sys.executable, "-m", "lerobot.scripts.lerobot_train"]

cmd = base + [
    f"--dataset.repo_id={REPO_ID}",
    f"--policy.type={POLICY}",
    f"--policy.device={DEVICE}",
    f"--policy.push_to_hub=false",
    f"--output_dir={OUTPUT_DIR}",
    f"--job_name={JOB_NAME}",
    f"--batch_size={BATCH_SIZE}",
    f"--steps={STEPS}",
    f"--save_freq={SAVE_FREQ}",
    f"--log_freq={LOG_FREQ}",
    f"--wandb.enable={'true' if USE_WANDB else 'false'}",
]

print("Running:\n  " + " ".join(cmd) + "\n")
sys.exit(subprocess.call(cmd))