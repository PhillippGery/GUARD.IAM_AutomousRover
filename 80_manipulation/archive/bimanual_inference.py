"""
Bimanual ACT inference on real arms — GUARD.IAM pick-and-place.

Uses the FORK'S OWN runner: make_pre_post_processors(pretrained_path=ckpt) loads
the checkpoint's saved normalization, and record_loop(policy=...) drives the arms
(same loop used for collection, minus the leader). This is the path that avoids
the transition-format issues a hand-built loop hits.

Policy is verified correct via dataset-replay (pred matched demonstrated actions).

Reseat the LEFT FOLLOWER before running.
"""

import json
import time

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig, Cv2Rotation
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.feature_utils import hw_to_dataset_features
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.robots.bi_so_follower import BiSOFollower, BiSOFollowerConfig
from lerobot.robots.so_follower import SO101FollowerConfig
from lerobot.scripts.lerobot_record import record_loop
from lerobot.utils.keyboard_input import init_keyboard_listener
from lerobot.utils.utils import log_say
from lerobot.utils.visualization_utils import init_rerun
from lerobot.processor import make_default_processors

# ===================== Basic Settings =====================
NUM_EPISODES = 5
FPS = 20                         # MUST match training (20, not 30)
EPISODE_TIME_SEC = 60
TASK_DESCRIPTION = "Pick up the object and place it in the bin"

LOCAL_CKPT_PATH = "/home/vedantpatkar/guardiam_pick_and_place/outputs/train/guardian_act/checkpoints/100000/pretrained_model"
# NEW throwaway id — this script CREATES a dataset, don't collide with the real one.
HF_DATASET_ID = "vedant/guardian_infer_run"

# Optional home-pose reset between episodes. Set to False if you don't have
# home_action.json yet (skips the reset so you can test inference immediately).
USE_HOME_RESET = False

# ===================== Cameras (mirror config.py exactly) =====================
CAM_W, CAM_H = 640, 480
camera_config = {
    "left_wrist":  OpenCVCameraConfig(index_or_path="/dev/cam_left_wrist",  width=CAM_W, height=CAM_H, fps=FPS, fourcc="MJPG"),
    "right_wrist": OpenCVCameraConfig(index_or_path="/dev/cam_right_wrist", width=CAM_W, height=CAM_H, fps=FPS, fourcc="MJPG"),
    "overhead":    OpenCVCameraConfig(index_or_path="/dev/cam_overhead",    width=CAM_W, height=CAM_H, fps=FPS,
                                      fourcc="MJPG", rotation=Cv2Rotation.ROTATE_180),
}

# ===================== Robot (bimanual follower, your ports) =====================
left_robot_config = SO101FollowerConfig(
    port="/dev/guardian_left_follower",
    id="left_guardian_follower",
    cameras=camera_config,
)

right_robot_config = SO101FollowerConfig(
    port="/dev/guardian_right_follower",
    id="right_guardian_follower",
)
bi_robot_config = BiSOFollowerConfig(
    left_robot_config,
    right_robot_config,
    id="guardian_follower",
)
robot = BiSOFollower(bi_robot_config)

# ===================== Load Policy =====================
policy = ACTPolicy.from_pretrained(LOCAL_CKPT_PATH)

# ===================== Dataset (created fresh for this run) =====================
action_features = hw_to_dataset_features(robot.action_features, "action")
obs_features = hw_to_dataset_features(robot.observation_features, "observation")
dataset_features = {**action_features, **obs_features}
dataset = LeRobotDataset.create(
    repo_id=HF_DATASET_ID,
    fps=FPS,
    features=dataset_features,
    robot_type=robot.name,
    use_videos=True,
    image_writer_threads=4,
)

# ===================== UI =====================
_, events = init_keyboard_listener()
init_rerun(session_name="bi_inference")

# ===================== Connect =====================
robot.connect()

# ===================== Pre/Post Processors (loads checkpoint's saved stats) =====================
preprocessor, postprocessor = make_pre_post_processors(
    policy_cfg=policy,
    pretrained_path=LOCAL_CKPT_PATH,
)
teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

# ===================== Optional home reset =====================
HOME_ACTION = None
if USE_HOME_RESET:
    with open("home_action.json") as f:
        HOME_ACTION = json.load(f)

def open_grippers(home_action):
    ha = home_action.copy()
    ha["left_gripper.pos"] = 50.0
    ha["right_gripper.pos"] = 50.0
    return ha

def reset_robot(robot, home_action, steps=100):
    for _ in range(steps):
        robot.send_action(home_action)
        time.sleep(0.04)  # 25 Hz

# ===================== Inference Loop =====================
for episode_idx in range(NUM_EPISODES):
    log_say(f"Episode {episode_idx + 1}", blocking=True)
    record_loop(
        robot=robot,
        events=events,
        fps=FPS,
        policy=policy,
        teleop_action_processor=teleop_action_processor,
        robot_action_processor=robot_action_processor,
        robot_observation_processor=robot_observation_processor,
        preprocessor=preprocessor,
        postprocessor=postprocessor,
        dataset=dataset,
        control_time_s=EPISODE_TIME_SEC,
        single_task=TASK_DESCRIPTION,
        display_data=True,
    )
    dataset.save_episode()
    if USE_HOME_RESET and HOME_ACTION is not None:
        reset_robot(robot, open_grippers(HOME_ACTION), steps=50)
        reset_robot(robot, HOME_ACTION, steps=100)

# ===================== Cleanup =====================
robot.disconnect()
print("[infer] done.")
