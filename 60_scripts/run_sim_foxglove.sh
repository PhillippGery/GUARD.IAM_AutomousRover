#!/bin/bash
# MIT License
# GUARDIAN — StarkHacks 2026
# Script: run_sim_foxglove.sh
# Purpose: sim nav stack + foxglove_bridge together, for testing the
#   browser control interface without installing systemd units. RViz is
#   deliberately off (rviz:=false) — this is the Foxglove-driven workflow,
#   not the RViz one; use `guardiam_sim` instead if you actually want RViz.
#   foxglove_bridge is backgrounded and killed via trap when the main
#   `ros2 launch` exits (Ctrl+C or otherwise), so it never lingers after
#   the sim shuts down — same reasoning as kill_ros.sh's own header.
#
# Usage: guardiam_sim_foxglove [mode]   (mode defaults to mapping)

MODE="${1:-mapping}"
# guardiam_env.sh (not a bare ROS2 source) — it also sets ROS_DOMAIN_ID=42,
# without which this would run on the default domain and be invisible to
# every other GUARDIAN process/terminal that does source it.
source "$HOME/GUARD.IAM_AutomousRover/60_scripts/guardiam_env.sh"

ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 address:=0.0.0.0 &
FOXGLOVE_PID=$!

cleanup() {
  echo "run_sim_foxglove.sh: shutting down foxglove_bridge (pid $FOXGLOVE_PID)..."
  kill "$FOXGLOVE_PID" 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "run_sim_foxglove.sh: foxglove_bridge up on ws://0.0.0.0:8765 (pid $FOXGLOVE_PID), starting sim mode=$MODE..."
ros2 launch guardian_bringup guardian.launch.py use_sim:=true mode:="$MODE" rviz:=false teleop:=true
