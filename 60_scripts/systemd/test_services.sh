#!/bin/bash
# MIT License
# GUARDIAN — StarkHacks 2026
# Script: test_services.sh
# Purpose: installs the same systemd --user units as install_services.sh
#   and starts them NOW, so you can verify hardware/foxglove_bridge/
#   web_ops_node actually work correctly on this machine — WITHOUT yet
#   committing to boot-time auto-start. Deliberately skips `enable` and
#   `loginctl enable-linger`: if you reboot right after running this,
#   nothing comes back up, on purpose. Once you've confirmed everything
#   works (motors respond, browser connects, buttons/demo/waypoints all
#   do the real thing), run install_services.sh to lock this in as the
#   actual boot sequence.
#
# Usage (on the robot, after `git pull` + `colcon build --symlink-install`):
#   bash test_services.sh
#   (or the `guardiam_test_boot` alias)

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT_DIR="$HOME/.config/systemd/user"

mkdir -p "$UNIT_DIR"
cp "$SCRIPT_DIR"/guardian-hardware.service \
   "$SCRIPT_DIR"/guardian-stack@.service \
   "$SCRIPT_DIR"/guardian-foxglove-bridge.service \
   "$SCRIPT_DIR"/guardian-ops.service \
   "$UNIT_DIR/"

systemctl --user daemon-reload
# restart (not start) — safe to re-run this script repeatedly while
# iterating, always picks up whatever unit files were just copied above.
systemctl --user restart \
  guardian-hardware.service \
  guardian-foxglove-bridge.service \
  guardian-ops.service

echo ""
echo "Started for testing — NOT enabled at boot yet:"
systemctl --user status --no-pager \
  guardian-hardware.service guardian-foxglove-bridge.service guardian-ops.service \
  | grep -E "●|Active:"
echo ""
echo "Connect from a browser: ws://<this-machine's-ZeroTier-IP>:8765"
echo ""
echo "Once everything checks out (motors respond to teleop, buttons work,"
echo "demo mission runs, waypoints record), lock this in as the real boot"
echo "sequence with:"
echo "  bash $SCRIPT_DIR/install_services.sh"
