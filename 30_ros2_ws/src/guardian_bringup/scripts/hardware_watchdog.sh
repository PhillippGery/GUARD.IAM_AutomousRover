#!/bin/bash
# GUARDIAN — detect a hardware node that's alive but silently producing no
# data, and force-restart it. A persistent watchdog, not a one-shot script.
#
# Why this exists: respawn=True (set on every node in
# guardian_hardware.launch.py) only helps when a process actually exits —
# ros2 launch has zero visibility into whether the topic it publishes is
# still flowing. Confirmed live on real hardware: sweep_scanner_front
# logged "starting data acquisition" and stayed alive and healthy from
# ros2 launch's point of view, while /scan silently stopped producing any
# data at all — no crash, no exit code, nothing for respawn to catch. Root
# cause (see kernel log): recurring USB control-transfer timeouts (-110/
# ETIMEDOUT — "failed to set flow control", "failed to get modem status")
# on the LIDARs' FTDI serial adapters, a physical USB signal/power issue
# (same class of fault hit the Phidget hub earlier), not an application
# bug — the driver's blocking serial read just silently stalls when that
# happens. Can't fix flaky USB from software, so instead: notice the
# silence and force a clean crash+respawn, which the existing
# infrastructure already knows how to recover from correctly.
#
# Usage: hardware_watchdog.sh <topic> <process_pattern> [stale_timeout_sec] [startup_grace_sec]
#   topic             - topic that should be continuously publishing
#   process_pattern   - pkill -f pattern matching the node's process to
#                        restart (must be specific enough not to also match
#                        some other running node — e.g. "sweep_scanner_front"
#                        not "sweep_scanner", since that would also match
#                        "sweep_scanner_back")
#   stale_timeout_sec - how long with zero messages before considered stale
#                        (default 5s — LIDARs publish at ~10Hz, so this is
#                        several missed cycles, not a single hiccup)
#   startup_grace_sec - how long to wait, both at script start AND after
#                        every restart, before staleness checks resume —
#                        long enough that a genuinely-booting node (LIDAR
#                        startup handshake has taken 5-13s on the bench)
#                        never gets killed mid-boot for simply not having
#                        published yet (default 30s)

set -u

# See activate_lifecycle_node.sh's matching trap for why this exists —
# without it this script's infinite loop (and its long startup-grace
# sleeps) doesn't notice ros2 launch's shutdown signal and is left running
# as an orphan that has to be found and killed by hand afterward.
trap 'echo "hardware_watchdog.sh: $TOPIC watchdog received shutdown signal, exiting"; exit 0' SIGINT SIGTERM

TOPIC="$1"
PROCESS_PATTERN="$2"
STALE_TIMEOUT_SEC="${3:-5}"
STARTUP_GRACE_SEC="${4:-30}"

echo "hardware_watchdog.sh: watching $TOPIC (restarts '$PROCESS_PATTERN' if stale >${STALE_TIMEOUT_SEC}s), ${STARTUP_GRACE_SEC}s startup grace"
sleep "$STARTUP_GRACE_SEC"

while true; do
  if timeout "$STALE_TIMEOUT_SEC" ros2 topic echo "$TOPIC" --once > /dev/null 2>&1; then
    : # healthy — got a message within the timeout
  else
    echo "hardware_watchdog.sh: $TOPIC stale >${STALE_TIMEOUT_SEC}s — restarting '$PROCESS_PATTERN'"
    pkill -f "$PROCESS_PATTERN"
    # Give the freshly-respawned process the same full boot grace before
    # checking again — otherwise a slow-but-healthy restart looks stale
    # too and gets killed again immediately, fighting its own recovery.
    sleep "$STARTUP_GRACE_SEC"
  fi
done
