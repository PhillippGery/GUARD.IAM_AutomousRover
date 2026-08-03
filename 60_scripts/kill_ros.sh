#!/bin/bash
# GUARDIAN — kill everything ROS-related, gracefully first.
#
# Not a blunt `pkill -9` — that skips phidget_bridge_node's safe motor
# shutdown (setEngaged(False) on every wheel, in its shutdown() handler)
# and skips the SIGINT traps in activate_lifecycle_node.sh/
# hardware_watchdog.sh, so motors could stay engaged and watchdog scripts
# would be left orphaned instead of exiting cleanly. Send SIGINT to the
# actual `ros2 launch` processes first (same as Ctrl+C), give them a real
# chance to cascade shutdown to every child, and only force-kill (-9)
# whatever is still alive after that grace period — covers genuine
# stragglers like l3xz_sweep_scanner_node occasionally blocking in an
# uninterruptible kernel read on shutdown (see hardware_watchdog.sh's
# header for why that specific case can't be avoided from here).

GRACE_SEC="${1:-8}"

LAUNCH_PIDS=$(pgrep -f "ros2 launch")
if [ -n "$LAUNCH_PIDS" ]; then
  echo "kill_ros.sh: sending SIGINT to ros2 launch (pid(s): $LAUNCH_PIDS), waiting up to ${GRACE_SEC}s for clean shutdown"
  kill -INT $LAUNCH_PIDS 2>/dev/null
  sleep "$GRACE_SEC"
else
  echo "kill_ros.sh: no ros2 launch process found"
fi

# Whatever's still alive after the grace period gets force-killed.
#
# This used to be a hand-enumerated list of binary names
# (nav2_controller, teleop_node, parameter_bridge, robot_state_publisher,
# ...) and kept growing every time a new one turned up orphaned — the
# node's *ROS name* (what shows up in `ros2 node list` / journal output,
# e.g. "gz_bridge", "teleop_twist_joy_node") is very often just a
# --ros-args -r __node:= remap and has nothing to do with the actual
# binary's filename on disk, so every hand-picked pattern here was one
# more binary away from missing the next orphan. Confirmed repeatedly
# this session: nav2_* executables, teleop_twist_joy's actual
# "teleop_node" binary, ros_gz_bridge's actual "parameter_bridge" binary,
# and even plain "robot_state_publisher" all individually slipped past
# earlier versions of this pattern and survived as orphans — one
# parameter_bridge for 45+ minutes across multiple kill_ros.sh runs,
# ending up as a second live publisher on /scan, /odom, /tf, /clock
# racing a freshly-launched one, which is what produced
# robot_state_publisher's "Moved backwards in time" warnings and very
# likely contributed to the swap-exhaustion incident documented in
# 10_docs/setup/web_control_interface_setup.md.
#
# Fix: match "--ros-args" instead of individual binary names. Every
# rclcpp/rclpy node process gets this flag appended by ros2 launch/run
# regardless of what its binary is called, so it's a robust catch-all for
# "this is a ROS2 node" — not just colcon/build tooling (a bare "ros2" or
# "rviz2" pattern would also match colcon build's own log output/paths,
# which --ros-args does not). The remaining explicit terms cover things
# that AREN'T ROS2 nodes and so never carry --ros-args: gz sim itself
# (the Gazebo Harmonic binaries — Classic's gzserver/gzclient names are
# kept too in case an older world/plugin config ever reintroduces them),
# rviz2, and the two watchdog/lifecycle shell scripts. web_ops_node and
# foxglove_bridge are kept explicitly as a safety net for the case where
# they're started with zero extra CLI args and so never got --ros-args
# appended at all.
REMAINING=$(pgrep -f "ros2 launch|--ros-args|rviz2 -d|gzserver|gzclient|gz sim|activate_lifecycle_node.sh|hardware_watchdog.sh|web_ops_node|foxglove_bridge")
if [ -n "$REMAINING" ]; then
  echo "kill_ros.sh: force-killing stragglers: $REMAINING"
  kill -9 $REMAINING 2>/dev/null
else
  echo "kill_ros.sh: clean — nothing left to force-kill"
fi
