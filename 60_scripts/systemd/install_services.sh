#!/bin/bash
# MIT License
# GUARDIAN — StarkHacks 2026
# Script: install_services.sh
# Purpose: one-time setup on the ROBOT's own PC (not a dev/lab machine) —
#   installs the systemd --user units that bring up hardware, the Foxglove
#   browser bridge, and the mode-switch/save-map ops node automatically at
#   boot. User-level units on purpose: no sudo/root needed, since this user
#   already has raw USB/serial device access (dialout/plugdev groups) for
#   everything these units launch. `loginctl enable-linger` is what makes
#   user units start at boot without an active login session — without it
#   they'd only run while this user is logged in (e.g. over SSH).
#
#   guardian-stack@.service (mapping/navigation) is intentionally NOT
#   enabled here — it has no [Install] section. web_ops_node starts
#   whichever instance is requested from the browser; nothing runs the nav
#   stack until you actually ask for it.
#
# Usage (on the robot, after `git pull`): bash install_services.sh

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
systemctl --user enable --now \
  guardian-hardware.service \
  guardian-foxglove-bridge.service \
  guardian-ops.service

# Without this, the units above only run while $USER has an active login
# session (e.g. your SSH connection) — linger keeps systemd --user alive
# across boots with nobody logged in at all.
sudo loginctl enable-linger "$USER"

echo ""
echo "Installed and started:"
systemctl --user status --no-pager guardian-hardware.service guardian-foxglove-bridge.service guardian-ops.service | grep -E "●|Active:"
echo ""
echo "guardian-stack@.service is installed but NOT running — start it via"
echo "the browser interface (web_ops_node's switch_to_mapping/"
echo "switch_to_navigation services), or manually:"
echo "  systemctl --user start guardian-stack@mapping.service"
echo "  systemctl --user start guardian-stack@navigation.service"
