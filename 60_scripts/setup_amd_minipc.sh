#!/bin/bash
# MIT License
# GUARDIAN — StarkHacks 2026
# Script: setup_amd_minipc.sh
# Purpose: one-shot install of all system dependencies on Ubuntu 22.04 / 24.04

set -e

echo "========================================"
echo "  GUARDIAN AMD MiniPC Setup"
echo "  StarkHacks 2026"
echo "========================================"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/../30_ros2_ws"
WS_SRC="$WS_DIR/src"

# ─── Detect Ubuntu version → ROS2 distro ─────────────────────────────────────
. /etc/os-release
case "$UBUNTU_CODENAME" in
  jammy)  ROS_DISTRO=humble ;;
  noble)  ROS_DISTRO=jazzy  ;;
  *)
    echo "ERROR: Unsupported Ubuntu release '$UBUNTU_CODENAME'."
    echo "       Supported: 22.04 (jammy) → humble, 24.04 (noble) → jazzy"
    exit 1
    ;;
esac
echo "Detected Ubuntu $UBUNTU_CODENAME → installing ROS2 $ROS_DISTRO"

# ─── ROS2 ─────────────────────────────────────────────────────────────────────
echo "[1/7] Installing ROS2 $ROS_DISTRO..."

sudo apt update
sudo apt install software-properties-common curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install "ros-$ROS_DISTRO-desktop" -y
sudo apt install python3-colcon-common-extensions python3-rosdep -y

sudo rosdep init || true  # ignore error if already initialized
rosdep update

# ─── ROS2 Packages ────────────────────────────────────────────────────────────
echo "[2/7] Installing ROS2 packages..."

sudo apt install -y \
  "ros-$ROS_DISTRO-nav2-bringup" \
  "ros-$ROS_DISTRO-robot-localization" \
  "ros-$ROS_DISTRO-realsense2-camera" \
  "ros-$ROS_DISTRO-teleop-twist-joy"

# ─── Python Packages ──────────────────────────────────────────────────────────
echo "[3/7] Installing Python packages..."

pip3 install pyserial numpy websockets

# ─── External ROS2 source repos ───────────────────────────────────────────────
echo "[4/7] Cloning external ROS2 source repos..."

mkdir -p "$WS_SRC"

if [ ! -d "$WS_SRC/l3xz_sweep_scanner" ]; then
  git clone https://github.com/107-systems/l3xz_sweep_scanner "$WS_SRC/l3xz_sweep_scanner"
else
  echo "  l3xz_sweep_scanner already cloned, skipping"
fi

if [ ! -d "$WS_SRC/lerobot" ]; then
  git clone https://github.com/huggingface/lerobot "$WS_SRC/lerobot"
else
  echo "  lerobot already cloned, skipping"
fi

# ─── LeRobot Python dependencies ─────────────────────────────────────────────
echo "[5/7] Installing LeRobot Python dependencies..."

pip3 install -e "$WS_SRC/lerobot[feetech]"

# ─── Intel RealSense udev rules ───────────────────────────────────────────────
echo "[6/7] Setting up Intel RealSense udev rules..."

sudo apt install -y librealsense2-utils librealsense2-dev || \
  echo "Warning: librealsense2 not found in apt — install manually from Intel repo"

# ─── Serial port permissions ──────────────────────────────────────────────────
echo "[7/7] Adding user to dialout group for serial access..."

sudo usermod -aG dialout "$USER"
echo "Note: log out and back in for dialout group to take effect"

# ─── rosdep workspace deps ────────────────────────────────────────────────────
echo "[+] Resolving workspace rosdep dependencies..."

source "/opt/ros/$ROS_DISTRO/setup.bash"
rosdep install --from-paths "$WS_SRC" --ignore-src -r -y

# ─── ROS2 source in bashrc ────────────────────────────────────────────────────
ROS_SETUP="source /opt/ros/$ROS_DISTRO/setup.bash"
if ! grep -qF "$ROS_SETUP" ~/.bashrc; then
  echo "$ROS_SETUP" >> ~/.bashrc
  echo "Added ROS2 $ROS_DISTRO to ~/.bashrc"
fi

echo ""
echo "========================================"
echo "  Setup complete! (ROS2 $ROS_DISTRO)"
echo "  Next steps:"
echo "  1. Log out and back in (dialout group)"
echo "  2. Run: bash 60_scripts/build_workspace.sh"
echo "  3. Run: bash 60_scripts/run_guardian.sh"
echo "========================================"
