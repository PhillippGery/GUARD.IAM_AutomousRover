#!/bin/bash

lerobot-record \
  --robot.type=so101_bimanual_follower \
  --robot.port_left=/dev/ttyACM0 \
  --robot.port_right=/dev/ttyACM1 \
  --robot.cameras='{
      "camera1": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30},
      "camera2": {"type": "opencv", "index_or_path": 1, "width": 640, "height": 480, "fps": 30},
      "camera3": {"type": "opencv", "index_or_path": 2, "width": 640, "height": 480, "fps": 30}
  }' \
  --dataset.single_task="Fold the object" \
  --dataset.repo_id=dipampatel/eval_dual_test \
  --dataset.episode_time_s=25 \
  --policy.path=outputs/train/smolvla_dual/checkpoints/last/pretrained_model