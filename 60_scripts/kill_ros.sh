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

# Whatever's still alive after the grace period gets force-killed —
# actual ROS runtime processes only, not colcon/build tooling (a bare
# "ros2" or "rviz2" pattern would also match colcon build's own log
# output/paths, so this matches real executable invocations specifically).
REMAINING=$(pgrep -f "ros2 launch|/install/.*_node |rviz2 -d|gzserver|gzclient|lifecycle_manager |async_slam_toolbox|activate_lifecycle_node.sh|hardware_watchdog.sh")
if [ -n "$REMAINING" ]; then
  echo "kill_ros.sh: force-killing stragglers: $REMAINING"
  kill -9 $REMAINING 2>/dev/null
else
  echo "kill_ros.sh: clean — nothing left to force-kill"
fi
