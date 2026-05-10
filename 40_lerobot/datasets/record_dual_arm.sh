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
  --teleop.type=so101_bimanual_leader \
  --teleop.port_left=/dev/ttyACM2 \
  --teleop.port_right=/dev/ttyACM3 \
  --dataset.single_task="Fold the object" \
  --dataset.repo_id=dipampatel/rover_dual_arm \
  --dataset.episode_time_s=25 \
  --dataset.reset_time_s=5 \
  --dataset.push_to_hub=true