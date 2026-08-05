#!/bin/bash
#SBATCH --account=shey
#SBATCH --job-name=guardian_act
#SBATCH --output=slurm_%x_%j.out
#SBATCH --gpus-per-node=1
#SBATCH --time=08:00:00
#SBATCH --mem=32G
#SBATCH --cpus-per-task=8

module load anaconda
conda activate guardiam_collect

export HF_HOME=$CLUSTER_SCRATCH/hf_cache
export HF_LEROBOT_HOME=$CLUSTER_SCRATCH/hf_cache/lerobot
export HF_TOKEN=$(cat ~/.cache/huggingface/token)

cd $CLUSTER_SCRATCH/GUARD.IAM_AutomousRover/80_manipulation
python $1
