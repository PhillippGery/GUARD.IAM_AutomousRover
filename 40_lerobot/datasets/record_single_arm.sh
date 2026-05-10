#!/bin/bash

# Change the following parameters as needed:
# - robot.port: the serial port for the follower robot (e.g., /dev/ttyACM0)
# - teleop.port: the serial port for the leader robot (e.g., /dev/ttyACM1)
# - dataset.repo_id: your Hugging Face repository ID (e.g., your_hf_username/rover_single_arm)
# - dataset.single_task: a description of the task being performed (e.g., "Pick up the object")
# - dataset.episode_time_s: the duration of each episode in seconds (e.g., 15)
# - dataset.reset_time_s: the time to wait between episodes in seconds (e.g., 5)
# - dataset.push_to_hub: whether to automatically push the dataset to Hugging Face Hub after recording (true or false)

lerobot-record \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=follower_arm \
  --robot.cameras='{
      "camera1": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30},
      "camera2": {"type": "opencv", "index_or_path": 1, "width": 640, "height": 480, "fps": 30}
  }' \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM1 \
  --teleop.id=leader_arm \
  --dataset.single_task="Pick up the object" \
  --dataset.repo_id=dipampatel/rover_single_arm \
  --dataset.episode_time_s=15 \
  --dataset.reset_time_s=5 \
  --dataset.push_to_hub=false