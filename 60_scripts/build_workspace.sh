#!/bin/bash
# MIT License
# GUARDIAN — StarkHacks 2026
# Script: build_workspace.sh
# Purpose: source ROS2, install deps, build the workspace

set -e

echo "Building GUARDIAN ROS2 workspace..."

source /opt/ros/jazzy/setup.bash

cd "$(dirname "$0")/../30_ros2_ws"

echo "Installing rosdep dependencies..."
rosdep install --from-paths src --ignore-src -r -y

echo "Building with colcon..."
colcon build --symlink-install

echo ""
echo "Build complete."
echo "Run: source 30_ros2_ws/install/setup.bash"
echo "Then: ros2 launch guardian_bringup guardian.launch.py"
