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
# Graceful shutdown first (SIGINT to ros2 launch, same as Ctrl+C — lets
# phidget_bridge_node safely disengage motors and lets the watchdog
# scripts' SIGINT traps exit cleanly), force-kill only what's still alive
# after. A blunt `pkill -9` skips all of that — see kill_ros.sh's header.
alias killros='bash $HOME/GUARD.IAM_AutomousRover/60_scripts/kill_ros.sh'
alias tf_tree='ros2 run tf2_tools view_frames'

# Demo mission (waypoint-to-waypoint, see 30_ros2_ws/src/guardian_bringup/
# config/waypoints.yaml) — run against an already-running nav stack
# (guardiam_sim / guardiam_map / guardiam_real), not launched by it.
alias guardiam_demo='ros2 run guardian_navigation demo_mission_node'

# Sim nav stack + foxglove_bridge together, RViz off — the browser-control
# testing path. Optional arg: mode (mapping/navigation), default mapping.
# e.g. `guardiam_sim_foxglove` or `guardiam_sim_foxglove navigation`.
alias guardiam_sim_foxglove='bash $HOME/GUARD.IAM_AutomousRover/60_scripts/run_sim_foxglove.sh'

# The other browser-control path: foxglove_bridge + web_ops_node only, no
# stack. Use the 🗺️/🧭/💾 buttons in Foxglove to start/stop the stack from
# there instead. Don't run this AND guardiam_sim_foxglove at the same
# time — both can start Gazebo, and running both is how you end up with
# duplicate instances fighting each other.
alias guardiam_web='bash $HOME/GUARD.IAM_AutomousRover/60_scripts/run_web_control.sh'

# set_waypoint <index> — drive the robot to where you want waypoint <index>
# to be, then run this to overwrite that entry in waypoints.yaml with the
# robot's current map->base_link pose (via set_waypoint_node). A plain
# alias can't take an argument like this, so it's a function.
set_waypoint() {
  ros2 run guardian_navigation set_waypoint_node --ros-args -p index:="$1"
}

# Real-robot deployment (mini PC only — not this dev/lab machine).
# guardiam_test_boot: install the systemd --user units and start them NOW
# for testing — hardware, foxglove_bridge, web_ops_node all come up, but
# nothing is enabled at boot yet and no loginctl enable-linger is run, so
# a reboot right after this brings nothing back. Verify motors respond,
# the browser connects, buttons/demo/waypoints all work — THEN run
# install_services.sh (below) to actually commit to the boot sequence.
# See 10_docs/setup/real_robot_deployment.md for the full walkthrough.
alias guardiam_test_boot='bash $HOME/GUARD.IAM_AutomousRover/60_scripts/systemd/test_services.sh'
# guardiam_install_boot: the real commit — enables all three at boot
# (systemctl --user enable --now) and runs loginctl enable-linger so they
# survive a reboot with nobody logged in. Only run this after
# guardiam_test_boot has already proven everything works.
alias guardiam_install_boot='bash $HOME/GUARD.IAM_AutomousRover/60_scripts/systemd/install_services.sh'





