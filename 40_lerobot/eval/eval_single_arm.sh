#!/bin/bash

lerobot-record \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=follower_arm \
  --robot.cameras='{
      "camera1": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30},
      "camera2": {"type": "opencv", "index_or_path": 1, "width": 640, "height": 480, "fps": 30}
  }' \
  --dataset.single_task="Pick up the object" \
  --dataset.repo_id=dipampatel/eval_single_test \
  --dataset.episode_time_s=15 \
  --policy.path=outputs/train/smolvla_single/checkpoints/last/pretrained_model