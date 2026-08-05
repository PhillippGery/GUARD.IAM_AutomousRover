#!/bin/bash
# GUARDIAN — reliably drive a ROS2 lifecycle node through configure ->
# activate, and keep it there for the life of the launch. A persistent
# watchdog, not a one-shot script.
#
# Why this exists, in two parts:
#
# 1. nav2_lifecycle_manager instances added ad hoc in guardian.launch.py
#    (as opposed to the original, always-reliable
#    lifecycle_manager_navigation) proved unreliable — they'd issue
#    Configure then silently never issue Activate. A fixed-delay
#    `ros2 lifecycle set` pair was tried next, but real-hardware startup
#    timing varies too much (LIDAR driver crash/respawn cycles routinely
#    add 10-30+ unpredictable seconds) for any single guessed delay to be
#    reliable — the call would fire before the node even existed yet and
#    just fail outright with no retry.
#
# 2. A one-shot version of this script (retry until success, then exit)
#    fixed that, but missed a second failure mode: some managed nodes
#    (confirmed via an actual GDB backtrace on amcl: a rare segfault
#    inside nav2_amcl's own particle-filter library,
#    uniformPoseGenerator/pf_init_model — third-party code, not ours) can
#    crash AFTER being successfully activated. respawn=True brings the
#    process back, but the fresh instance starts at "unconfigured" and a
#    script that already exited never re-drives it. This loops forever
#    instead, so any respawn gets caught and re-activated automatically.
#
# Usage: activate_lifecycle_node.sh <node_name> [transition_timeout_sec] [poll_interval_sec]

set -u

NODE_NAME="$1"
TRANSITION_TIMEOUT_SEC="${2:-90}"
POLL_INTERVAL_SEC="${3:-3}"

wait_for_transition() {
  local transition="$1"
  local deadline=$(( $(date +%s) + TRANSITION_TIMEOUT_SEC ))
  while [ "$(date +%s)" -lt "$deadline" ]; do
    if ros2 lifecycle set "/$NODE_NAME" "$transition" 2>&1; then
      echo "activate_lifecycle_node.sh: $NODE_NAME -> $transition succeeded"
      return 0
    fi
    sleep 1
  done
  echo "activate_lifecycle_node.sh: $NODE_NAME -> $transition FAILED after ${TRANSITION_TIMEOUT_SEC}s" >&2
  return 1
}

drive_to_active() {
  wait_for_transition configure && wait_for_transition activate
}

# Initial bring-up.
drive_to_active

# Watchdog: for the rest of the launch's life, notice if the node ever
# stops being active (crash+respawn landing back at unconfigured/inactive,
# or the node disappearing entirely during its own respawn window) and
# drive it back to active again.
while true; do
  sleep "$POLL_INTERVAL_SEC"
  state="$(ros2 lifecycle get "/$NODE_NAME" 2>/dev/null)"
  case "$state" in
    active*) ;;  # healthy, nothing to do
    *)
      echo "activate_lifecycle_node.sh: $NODE_NAME not active (state: '${state:-unreachable}') — re-activating"
      drive_to_active
      ;;
  esac
done
