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
  "ros-$ROS_DISTRO-navigation2" \
  "ros-$ROS_DISTRO-nav2-amcl" \
  "ros-$ROS_DISTRO-nav2-map-server" \
  "ros-$ROS_DISTRO-nav2-lifecycle-manager" \
  "ros-$ROS_DISTRO-nav2-controller" \
  "ros-$ROS_DISTRO-nav2-dwb-controller" \
  "ros-$ROS_DISTRO-slam-toolbox" \
  "ros-$ROS_DISTRO-robot-localization" \
  "ros-$ROS_DISTRO-teleop-twist-joy" \
  "ros-$ROS_DISTRO-teleop-twist-keyboard" \
  "ros-$ROS_DISTRO-joy" \
  "ros-$ROS_DISTRO-tf2-tools" \
  "ros-$ROS_DISTRO-tf2-ros" \
  "ros-$ROS_DISTRO-ros-gz" \
  "ros-$ROS_DISTRO-joint-state-publisher" \
  "ros-$ROS_DISTRO-joint-state-publisher-gui" \
  "ros-$ROS_DISTRO-xacro"

# ─── Python Packages ──────────────────────────────────────────────────────────
echo "[3/7] Installing Python packages..."

pip3 install pyserial numpy websockets --break-system-packages

# ─── Phidgets VINT Hub support ────────────────────────────────────────────────
echo "[3b/7] Installing Phidgets VINT Hub support..."

sudo apt install -y libusb-1.0-0-dev
curl -fsSL https://www.phidgets.com/downloads/setup_linux | sudo bash
sudo apt install -y libphidget22 libphidget22-dev

# ─── External ROS2 source repos ───────────────────────────────────────────────
echo "[4/7] Cloning external ROS2 source repos..."

mkdir -p "$WS_SRC"

if [ ! -d "$WS_SRC/l3xz_sweep_scanner" ]; then
  git clone https://github.com/107-systems/l3xz_sweep_scanner "$WS_SRC/l3xz_sweep_scanner"
else
  echo "  l3xz_sweep_scanner already cloned, skipping"
fi

# lerobot and phidgets_drivers are git submodules (see .gitmodules) — pinned
# to a known commit and checked out along with the rest of the repo, not
# cloned ad hoc here.
git -C "$SCRIPT_DIR/.." submodule update --init --recursive \
  30_ros2_ws/src/lerobot \
  30_ros2_ws/src/phidgets_drivers

# ─── LeRobot Python dependencies ─────────────────────────────────────────────
echo "[5/7] Installing LeRobot Python dependencies..."

pip3 install -e "$WS_SRC/lerobot[feetech]" --break-system-packages

# ─── Intel RealSense D415 ─────────────────────────────────────────────────────
# Skipped for now — see chat: librealsense.pgp is ASCII-armored, needs
# `gpg --dearmor` before apt's signed-by= will accept it as a valid keyring.
# Uncomment to re-enable (corrected):
# echo "[6/7] Setting up Intel RealSense D415..."

# sudo mkdir -p /etc/apt/keyrings
# curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
#   | sudo gpg --dearmor --yes -o /etc/apt/keyrings/librealsense.pgp

# echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] \
#   https://librealsense.intel.com/Debian/apt-repo \
#   $(lsb_release -cs) main" \
#   | sudo tee /etc/apt/sources.list.d/librealsense.list > /dev/null

# sudo apt update
# sudo apt install -y librealsense2-dkms librealsense2-utils \
#   librealsense2-dev librealsense2-dbg

# ─── Serial port permissions ──────────────────────────────────────────────────
echo "[7/7] Adding user to dialout group for serial access..."

sudo usermod -aG dialout "$USER"
echo "Note: log out and back in for dialout group to take effect"

# ─── rosdep workspace deps ────────────────────────────────────────────────────
echo "[+] Resolving workspace rosdep dependencies..."

source "/opt/ros/$ROS_DISTRO/setup.bash"
rosdep install --from-paths "$WS_SRC" --ignore-src -r -y

# ─── GUARD.IAM env (ROS2 + workspace + aliases) in bashrc ────────────────────
ENV_SETUP="source $SCRIPT_DIR/guardiam_env.sh"
if ! grep -qF "$ENV_SETUP" ~/.bashrc; then
  echo "$ENV_SETUP" >> ~/.bashrc
  echo "Added GUARD.IAM env (guardiam_env.sh) to ~/.bashrc"
fi

echo ""
echo "========================================"
echo "  Setup complete! (ROS2 $ROS_DISTRO)"
echo "  Next steps:"
echo "  1. Log out and back in (dialout group)"
echo "  2. Run: bash 60_scripts/build_workspace.sh"
echo "  3. Run: bash 60_scripts/run_guardian.sh"
echo "========================================"
