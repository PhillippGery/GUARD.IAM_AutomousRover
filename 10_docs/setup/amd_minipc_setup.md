# AMD Ryzen AI MiniPC Setup

## Prerequisites

- AMD Ryzen AI MiniPC
- USB keyboard + HDMI monitor (for initial OS install)
- USB drive with Ubuntu 22.04 LTS image

## Steps

### 1. Install Ubuntu 22.04 LTS

Flash Ubuntu 22.04 LTS to a USB drive and boot the MiniPC from it. Follow the installer — use the entire disk, create a user named `guardian` for consistency.

### 2. Install ROS2 Humble

```bash
# Add ROS2 apt repository
sudo apt install software-properties-common curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-jazzy-desktop -y
```

### 3. Install colcon and rosdep

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep -y
sudo rosdep init
rosdep update
```

### 4. Clone GUARDIAN Repository

```bash
git clone https://github.com/PhillippGery/GUARDIAN.git ~/GUARDIAN
```

### 5. Run Automated Setup Script

```bash
cd ~/GUARDIAN
bash 60_scripts/setup_amd_minipc.sh
```

This script installs all ROS2 packages, Python dependencies, udev rules, and adds your user to the `dialout` group.

### 6. Connect Intel RealSense T265

Plug the T265 into a **USB 3.0** port (blue connector). Verify detection:

```bash
rs-enumerate-devices
```

### 7. Connect Scanse Sweep LIDAR

Plug the Sweep into a USB port. Confirm it appears:

```bash
ls /dev/ttyUSB*
# Expected: /dev/ttyUSB0
```

### 8. Connect Arduino Mega

Plug the Arduino Mega USB into another port. Confirm:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
# Expected: /dev/ttyACM0 or /dev/ttyUSB1
```

Update `robot_params.yaml` if the port differs from `/dev/ttyUSB0`.

### 9. Connect SO-101 Arms

Plug both SO-101 follower arms into separate USB-C ports. Verify with `lsusb`.

### 10. Launch GUARDIAN

```bash
source /opt/ros/jazzy/setup.bash
source ~/GUARDIAN/30_ros2_ws/install/setup.bash
ros2 launch guardian_bringup guardian.launch.py use_sim:=false
```

Or use the menu script:

```bash
bash ~/GUARDIAN/60_scripts/run_guardian.sh
```
