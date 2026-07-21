#!/bin/bash
# MIT License
# GUARDIAN — StarkHacks 2026
# Script: identify_lidar_port.sh
# Purpose: print each connected USB-serial device's stable identifying
#          info (serial number, vendor/product, physical port path) so a
#          udev rule can pin a fixed /dev/lidar_front or /dev/lidar_back
#          symlink to a specific physical unit — instead of the raw
#          /dev/ttyUSB0/1 device names, which are assigned by USB
#          enumeration order and are NOT guaranteed to stay attached to
#          the same physical LIDAR across a reboot or replug.
#
# Usage: plug in ONE LIDAR at a time (unplug the other, or any other
# USB-serial adapter) and run this script — read off the ID_SERIAL_SHORT
# line, label that LIDAR unit physically (e.g. tape marked "F" or "B"),
# and put that serial number into 99-guardian-lidar.rules. Repeat for the
# second unit.

set -u

found=0
for dev in /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$dev" ] || continue
  found=1
  echo "=== $dev ==="
  udevadm info -a -n "$dev" 2>/dev/null | grep -E \
    "ATTRS\{serial\}|ATTRS\{idVendor\}|ATTRS\{idProduct\}|ATTRS\{product\}|ATTRS\{manufacturer\}" \
    | head -5
  echo "  (stable path) KERNELS: $(udevadm info -q path -n "$dev" 2>/dev/null)"
  echo
done

if [ "$found" -eq 0 ]; then
  echo "No /dev/ttyUSB* or /dev/ttyACM* devices found — plug in a LIDAR first."
  exit 1
fi
