#!/bin/bash
# GUARD.IAM Environment Setup
# Sourced automatically by ~/.bashrc

# ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Workspace — source if built
WS_SETUP="$HOME/GUARD.IAM_AutomousRover/30_ros2_ws/install/setup.bash"
if [ -f "$WS_SETUP" ]; then
  source "$WS_SETUP"
fi

# ROS2 settings
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# Gazebo
export GZ_SIM_RESOURCE_PATH=$HOME/GUARD.IAM_AutomousRover/30_ros2_ws/src

export ROS_WS=~/GUARD.IAM_AutomousRover/30_ros2_ws

# Aliases
alias rosd='cd $ROS_WS'      # change to ws1 directory
alias rosds='cd $ROS_WS/src' # change to ws1/src directory
alias _ws='rosd && source install/setup.bash'

alias rosclean='rosd && rm -rf build/ install/'   # remove all build files

alias cb='rosd && colcon build --symlink-install && _ws'
alias cbs='rosd && colcon build --symlink-install && source install/setup.bash'
alias cbt='rosd && colcon test'

alias guardiam_sim='source install/setup.bash && ros2 launch guardian_bringup guardian.launch.py use_sim:=true mode:=navigation teleop:=true'

# Real hardware is two separate launches now — guardiam_hw owns the drive
# chain, both LIDARs, and Xbox teleop (self-healing, respawn=True on every
# node, meant to be started once and left running), while guardiam_map/
# guardiam_real own only the software stack (Nav2, SLAM/AMCL, RViz) on top
# and assume guardiam_hw is already running. Start guardiam_hw first.
alias guardiam_hw='ros2 launch guardian_bringup guardian_hardware.launch.py'
alias guardiam_map='ros2 launch guardian_bringup guardian.launch.py use_sim:=false mode:=mapping'
alias guardiam_real='ros2 launch guardian_bringup guardian.launch.py use_sim:=false mode:=navigation'
alias killros='pkill -9 -f "ros2|gazebo|gzserver|gzclient|rviz2"'
alias tf_tree='ros2 run tf2_tools view_frames'





