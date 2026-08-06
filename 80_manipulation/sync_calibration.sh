#!/usr/bin/env bash
# Copies the repo's calibration files into the local lerobot cache so this
# machine (laptop OR minipc) uses the shared, version-controlled calibration.
set -e
REPO_CAL="$(cd "$(dirname "$0")/calibration" && pwd)"
CACHE="$HOME/.cache/huggingface/lerobot/calibration"
mkdir -p "$CACHE/teleoperators/so_leader" "$CACHE/robots/so_follower"
cp "$REPO_CAL"/so_leader/*.json   "$CACHE/teleoperators/so_leader/"
cp "$REPO_CAL"/so_follower/*.json "$CACHE/robots/so_follower/"
echo "Calibration synced from repo -> $CACHE"
