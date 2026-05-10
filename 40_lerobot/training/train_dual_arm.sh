#!/bin/bash

lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id=dipampatel/rover_dual_arm \
  --batch_size=16 \
  --steps=20000 \
  --save_freq=2000 \
  --log_freq=100 \
  --output_dir=outputs/train/smolvla_dual \
  --job_name=smolvla_dual_training \
  --policy.device=cuda \
  --wandb.enable=false \
  --policy.push_to_hub=false

# Resume Training
lerobot-train \
  --config_path=outputs/train/smolvla_single/checkpoints/last/pretrained_model/train_config.json \
  --resume=true