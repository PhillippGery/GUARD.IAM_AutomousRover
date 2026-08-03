#!/bin/bash
# MIT License
# GUARDIAN — StarkHacks 2026
# Script: run_web_control.sh
# Purpose: the ONE thing to run to bring up the browser control interface
#   for testing — starts guardian_sim.launch.py (Gazebo/robot/bridge/
#   lidar-merge, no Nav2), foxglove_bridge, and web_ops_node together, and
#   leaves the sim environment running continuously.
#
#   The sim environment has to be started here and stay up across mode
#   switches — the 🗺️ Start Mapping / 🧭 Start Navigation buttons only
#   restart the Nav2/SLAM/AMCL stack on top (guardian.launch.py
#   include_sim_env:=false via guardian-stack@.service), not Gazebo
#   itself. Restarting Gazebo on every mode switch used to reset /clock
#   back near zero and respawn the robot at the origin each time, which
#   made Foxglove's TF buffer treat the "new" but earlier-than-previously
#   -seen transforms as stale and drop them — the rendered robot pose
#   visibly froze and the URDF mesh stopped updating even though data was
#   still flowing underneath. See guardian_sim.launch.py's header for the
#   full story.
#
#   Don't combine this with guardiam_sim_foxglove — that one brings up
#   its own self-contained Gazebo instance too (include_sim_env:=true,
#   the default), and running both means two Gazebos fighting each other.
#
#   All three processes are backgrounded except web_ops_node, and killed
#   via trap when it exits (Ctrl+C or otherwise) so nothing lingers —
#   same reasoning as kill_ros.sh's own header.
#
# Usage: guardiam_web

source "$HOME/GUARD.IAM_AutomousRover/60_scripts/guardiam_env.sh"

# Install/refresh the sim-test systemd unit. Deliberately no Restart=
# line — a Restart=on-failure here silently resurrects Gazebo every time
# you try to kill it during testing, which is exactly what exhausted this
# machine's RAM+swap during initial testing. Always rewritten (not just
# installed if missing) so an older copy without include_sim_env:=false
# doesn't linger and cause the reset-Gazebo-on-mode-switch bug again.
UNIT_PATH="$HOME/.config/systemd/user/guardian-stack@.service"
mkdir -p "$HOME/.config/systemd/user"
cat > "$UNIT_PATH" << 'EOF'
[Unit]
Description=GUARDIAN nav stack TEST (sim), mode=%i

[Service]
Type=simple
ExecStart=/bin/bash -c 'source %h/GUARD.IAM_AutomousRover/60_scripts/guardiam_env.sh && exec ros2 launch guardian_bringup guardian.launch.py use_sim:=true mode:=%i rviz:=false teleop:=true include_sim_env:=false'
EOF
systemctl --user daemon-reload

ros2 launch guardian_bringup guardian_sim.launch.py &
SIM_ENV_PID=$!

ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 address:=0.0.0.0 &
FOXGLOVE_PID=$!

cleanup() {
  echo "run_web_control.sh: shutting down sim env (pid $SIM_ENV_PID) and foxglove_bridge (pid $FOXGLOVE_PID)..."
  kill "$SIM_ENV_PID" "$FOXGLOVE_PID" 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "run_web_control.sh: sim environment starting (pid $SIM_ENV_PID) — give it ~5s for Gazebo/robot/bridge"
echo "run_web_control.sh: foxglove_bridge up on ws://0.0.0.0:8765 (pid $FOXGLOVE_PID)"
echo "run_web_control.sh: starting web_ops_node — use the 🗺️/🧭/💾 buttons in Foxglove to bring up the nav stack"
ros2 run guardian_navigation web_ops_node
