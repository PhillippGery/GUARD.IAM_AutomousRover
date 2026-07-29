# GUARD.IAM — Bimanual Manipulation Pipeline

End-to-end ACT imitation-learning pipeline for the dual-arm GUARD.IAM follower:
teleoperate → collect demonstrations → train an ACT policy → run it on the real arms.

Built on the **equibim** LeRobot fork
(`git@github.com:ZhangZhiyuanZhang/lerobot-equibim.git`). This repo contains only
the pipeline scripts; the fork and ML dependencies are installed separately (below).

> ⚠️ Read the **Gotchas** section before your first real collection run. Several
> settings (FPS, MJPG, normalization) are load-bearing and expensive to get wrong.

---

## 0. Prerequisites

- Linux (developed on Ubuntu 24.04) with an NVIDIA GPU + recent driver.
  Verify: `nvidia-smi` should print the GPU table. If it errors with
  "Driver/library version mismatch", **reboot** first.
- [Miniforge / Miniconda](https://github.com/conda-forge/miniforge) installed.
- GitHub access to the **equibim** repo (SSH key set up, or use HTTPS + token).
- Dual-arm SO-101 hardware: 2 leaders, 2 followers, 3 USB cameras
  (2 wrist + 1 overhead), ideally on a **powered USB hub**.

---

## 1. Environment setup (conda)

```bash
# From this directory (80_manipulation/):
conda env create -f environment.yml
conda activate lerobot
```

> **Always work inside this env.** All dependencies live here. Do NOT
> `conda deactivate` while debugging — `pip install` works inside conda
> (no `--break-system-packages` needed).

If `conda env create` fails on the CUDA/torch pin (GPU/driver differences),
create a bare env and install torch manually:
```bash
conda create -n lerobot python=3.12 -y && conda activate lerobot
# install torch for your CUDA version per https://pytorch.org, then continue to step 2
```

---

## 2. Install the equibim LeRobot fork

The pipeline imports `lerobot` from the **equibim** fork (not stock LeRobot).

```bash
# SSH (needs GitHub SSH access):
git clone git@github.com:ZhangZhiyuanZhang/lerobot-equibim.git lerobot-equibim
# or HTTPS:
# git clone https://github.com/ZhangZhiyuanZhang/lerobot-equibim.git lerobot-equibim

cd lerobot-equibim
pip install -e .
cd ..
```

Verify it imports from the fork:
```bash
python -c "import lerobot; print(lerobot.__file__)"   # should point into lerobot-equibim/
```

---

## 3. Install remaining Python deps

```bash
pip install -r requirements.txt
```
(Includes `evdev` for the foot pedal. `requirements.txt` also re-pins the equibim
fork; the editable install in §2 takes precedence and is fine.)

---

## 4. Per-machine hardware setup (REQUIRED — does not transfer from git)

### 4a. udev symlinks (stable device names)

```bash
sudo cp ../60_scripts/99-guardian.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Expected (verify with `ls -l /dev/cam_* /dev/guardian_*`):
- `/dev/cam_left_wrist`, `/dev/cam_right_wrist`, `/dev/cam_overhead`
- `/dev/guardian_left_follower`, `/dev/guardian_right_follower`
- `/dev/guardian_left_leader`, `/dev/guardian_right_leader`

Arm-count sanity check (must print 4):
```bash
ls /dev/guardian_* | wc -l
```

> If you re-plug USB or change the hub, the underlying `3-x.y` port paths change
> and the symlinks break. Re-derive with:
> ```bash
> for d in /dev/ttyACM* /dev/video*; do
>   printf '%-16s %s\n' "$d" "$(udevadm info -a -n "$d" 2>/dev/null | grep -m1 KERNELS)"
> done
> ```
> then update the `KERNELS==` values in `99-guardian.rules` and reload.

### 4b. Camera format check (must support MJPG)

```bash
v4l2-ctl -d /dev/cam_left_wrist  --list-formats-ext   # confirm MJPG @ 640x480
v4l2-ctl -d /dev/cam_right_wrist --list-formats-ext
v4l2-ctl -d /dev/cam_overhead    --list-formats-ext
```
All three must offer MJPG at 640×480 (see Gotchas for why).

### 4c. Calibration

Each follower needs a calibration file in the LeRobot calibration cache:
`~/.cache/huggingface/lerobot/calibration/robots/bi_so_follower/`
- `left_guardian_follower.json`
- `right_guardian_follower.json`

> **The robot ids in `config.py` MUST match these filenames**
> (`left_guardian_follower`, `right_guardian_follower`, and the bimanual id
> `guardian_follower`). If they don't match, the arms prompt to recalibrate on
> connect — do NOT recalibrate over a working setup; fix the ids instead.

### 4d. Edit `config.py`

At the top of `config.py`, set:
- `PORT_*` — udev symlink paths for each arm
- `ID_*` — must match the calibration filenames (§4c)
- camera symlink paths, resolution, and **FPS = 20**

`config.py` is the single source of truth shared by collect, train, and inference.
Anything set here (cameras, fps, rotation) applies everywhere.

---

## 5. Foot pedal (optional — hands-free collection)

X-keys XK-3 foot pedal → arrow keys. Bridge: `../60_scripts/pedal_bridge.py`.

```bash
pip install evdev   # (already in requirements.txt)

# Run in a SECOND terminal, alongside collect.py. Needs sudo to read the device:
sudo $(which python) ../60_scripts/pedal_bridge.py
```

Mapping: **LEFT pedal → LEFT arrow** (scrap & redo), **RIGHT pedal → RIGHT arrow**
(keep & continue), **CENTER unmapped**. ESC (stop) stays on the keyboard.

Troubleshooting: if the pedal isn't found, run `sudo evtest`, find the XK-3 node
exposing `BTN_*` events, and set that `/dev/input/eventNN` path in
`pedal_bridge.py`. (Event numbers can shift across reboots.)

---

## 6. Teleoperation (verify arms + cameras)

```bash
python teleop_test.py
```
Drives followers from the leaders; streams the 3 camera feeds + state to rerun.
**Check:** each follower gripper fully opens AND closes, and all three feeds are
clean (no tearing). Fix any issue here before collecting.

---

## 7. Data collection

```bash
python collect.py
```

**Controls (foot pedal or keyboard):**

| Action | Key | Pedal |
|---|---|---|
| Keep episode, continue | RIGHT arrow | Right pedal |
| Scrap & re-record | LEFT arrow | Left pedal |
| Stop session | ESC | (keyboard only) |

**Before a REAL run:**
- Set `WIPE_BEFORE_RUN = False` (practice mode wipes the dataset each run).
- Use a real, unique `REPO_ID` (keep practice on a separate id like `guardian_practice`).
- **Reseat the left follower** — it can drop off the bus mid-episode and silently
  corrupt a take.
- Use a generous `EPISODE_TIME_SEC` (e.g. 60); RIGHT ends early when you finish,
  so long timers cost nothing.

**Collect good demos:** complete the task fully every episode (object placed,
grippers released), vary object position across episodes, scrap any fumble.

**Verify after collecting** (dataset loads + all 12 joints move):
```bash
python - <<'PY'
from lerobot.datasets.lerobot_dataset import LeRobotDataset
import numpy as np
d = LeRobotDataset("vedant/guardian_pick_place")   # <-- your REPO_ID
print("episodes", d.num_episodes, "frames", d.num_frames, "fps", d.fps)
s = np.stack([d[i]["observation.state"].numpy() for i in range(0, len(d), 50)])
print("per-joint range (all should be clearly > 0):", np.round(s.max(0)-s.min(0), 2))
PY
```

---

## 8. Training

```bash
python train.py
```
Wraps the fork's `lerobot-train` for ACT. Config block at the top:
- `STEPS` — 40k–100k for a real run. Watch the loss plateau; steps past it buy
  little. **Judge the policy by evaluating checkpoints on the arms, not loss alone.**
- `SAVE_FREQ = 10000` — checkpoints in `outputs/train/<job>/checkpoints/NNNNNN/`.
- `USE_WANDB` — optional. If used, run `wandb login` once. Loss also prints to console.

Leave the machine on AC power for long runs; it checkpoints every `SAVE_FREQ`
steps whether or not you're watching.

---

## 9. Inference (run the policy on the arms)

```bash
python bimanual_inference.py
```
Loads a checkpoint and drives the followers through the fork's `record_loop`.

Set at the top of `bimanual_inference.py`:
- `LOCAL_CKPT_PATH` — a checkpoint's `pretrained_model` dir, e.g.
  `outputs/train/guardian_act/checkpoints/060000/pretrained_model`
- `FPS = 20` (must match training)
- `USE_HOME_RESET` — `True` only if you have a `home_action.json` home pose

**Before running:** reseat the left follower; keep a hand near Ctrl-C on the first
rollout (a fresh policy can move abruptly).

**Sanity-check a checkpoint without the robot** — predicted actions should match
the demonstrated actions on training frames (within a few degrees per joint):
```bash
python - <<'PY'
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.processor import PolicyProcessorPipeline
import torch, numpy as np
ck = "outputs/train/guardian_act/checkpoints/100000/pretrained_model"  # <-- your checkpoint
ds = LeRobotDataset("vedant/guardian_pick_place")                       # <-- your REPO_ID
p = ACTPolicy.from_pretrained(ck); p.config.device="cuda"; p.to("cuda").eval(); p.reset()
pre  = PolicyProcessorPipeline.from_pretrained(ck, "policy_preprocessor.json")
post = PolicyProcessorPipeline.from_pretrained(ck, "policy_postprocessor.json")
for i in range(5):
    f = ds[i]; obs = {k: f[k] for k in f if k.startswith("observation")}; obs["task"]=f["task"]
    a = post({"action": p.select_action(pre(obs))})
    a = a["action"] if isinstance(a, dict) and "action" in a else a
    a = a.squeeze(0).detach().cpu().numpy() if torch.is_tensor(a) else np.asarray(a).squeeze()
    print("pred", np.round(a,1)); print("true", f["action"].numpy().round(1))
PY
```
If `pred` tracks `true`, inference is correct and any remaining gap is data quality.

---

## Gotchas (hard-won — DO NOT IGNORE)

- **FPS = 20 everywhere.** Collect, train, AND infer must all use 20 fps. 30 fps
  overruns the shared USB2 hub and starves a camera (torn frames / timeouts).
  Whatever fps you collect at is locked for training and inference.
- **3 cameras require `fourcc="MJPG"`** at 640×480. Raw YUYV exceeds the hub's
  bandwidth and causes tearing / camera timeouts. (Powered hub and longer read
  timeouts did NOT fix it — dropping to 20 fps + MJPG did.)
- **Overhead camera is mounted upside down** → `rotation=ROTATE_180` in `config.py`.
  Baked into the data; keep it identical between collect and infer.
- **`collect.py` must finalize the dataset before exit**, or `meta/episodes` never
  gets written and the dataset won't load. This repo's `collect.py` calls the
  finalize step explicitly before disconnect/exit (a bare `os._exit(0)` is NOT
  enough — it skips finalization).
- **Inference normalization comes from the checkpoint's SAVED processors.** Use the
  fork's `record_loop` + `make_pre_post_processors(pretrained_path=...)` (as
  `bimanual_inference.py` does). Recomputing normalization from dataset stats gives
  WRONG numbers and the policy collapses to a fixed pose.
- **Robot ids must match calibration filenames** (§4c) or the arms re-prompt to
  calibrate.
- **The left follower is the flaky one** — reseat its USB + power before real runs;
  a mid-episode bus drop silently corrupts data and fakes inference "failures".
- **Resolution / fps / camera set at collection must exactly match inference.**

---

## Not included in this repo (by design)

- **Datasets** (`~/.cache/huggingface/lerobot/...`) — too large. Re-collect, or
  pull a shared dataset from the HuggingFace Hub.
- **Checkpoints** (`outputs/train/...`) — too large. Share trained policies via the
  HuggingFace Hub and load with `from_pretrained("<hub_id>")`.
- **The equibim fork** — installed as a dependency (§2), not vendored here.

---

## Repo layout (relevant paths)

```
80_manipulation/
  config.py                 # single source of truth: ports, ids, cameras, fps
  teleop_test.py            # verify arms + cameras
  collect.py                # record demonstrations
  train.py                  # train ACT (wraps lerobot-train)
  bimanual_inference.py     # run a policy on the arms
  environment.yml           # conda env (this pipeline's deps)
  requirements.txt          # pip extras (evdev, equibim pin)
  README.md                 # this file
../60_scripts/
  99-guardian.rules         # udev rules for camera/arm symlinks
  pedal_bridge.py           # X-keys foot pedal → arrow keys
  aim_overhead.py           # overhead PTZ aim tool
```