#!/bin/bash
# MIT License
# GUARDIAN — StarkHacks 2026
# Script: run_guardian.sh
# Purpose: interactive launch menu

set -e

source /opt/ros/jazzy/setup.bash
source "$(dirname "$0")/../30_ros2_ws/install/setup.bash"

echo ""
echo "========================================"
echo "  GUARDIAN Launch Menu"
echo "  StarkHacks 2026 | Purdue University"
echo "========================================"
echo "  1) Sim, autonomous navigation"
echo "  2) Real hardware, mapping (drives + builds a map)"
echo "  3) Real hardware, autonomous navigation"
echo "  4) Teleop only (standalone keyboard node)"
echo "  5) Arms only"
echo "  6) Motor test"
echo "========================================"
read -rp "Select [1-6]: " choice

case $choice in
  1)
    echo "Launching sim, autonomous navigation..."
    ros2 launch guardian_bringup guardian.launch.py
    ;;
  2)
    echo "Launching real hardware, mapping mode..."
    ros2 launch guardian_bringup guardian.launch.py use_sim:=false mode:=mapping
    ;;
  3)
    echo "Launching real hardware, autonomous navigation..."
    ros2 launch guardian_bringup guardian.launch.py use_sim:=false
    ;;
  4)
    echo "Launching teleop (standalone)..."
    ros2 run guardian_teleop keyboard_teleop_node
    ;;
  5)
    echo "TODO: arms-only launch not yet implemented"
    # ros2 launch guardian_manipulation guardian_manipulation.launch.py
    ;;
  6)
    echo "TODO: motor test script not yet implemented"
    # ros2 run guardian_drive serial_bridge_node --ros-args -p test_mode:=true
    ;;
  *)
    echo "Invalid selection: $choice"
    exit 1
    ;;
esac
