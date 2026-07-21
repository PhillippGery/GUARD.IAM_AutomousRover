# GUARDIAN ROS2 Workspace

ROS2 Jazzy workspace — Ubuntu 24.04, pure Python (ament_python).

## Packages

| Package | Owner | Purpose |
|---------|-------|---------|
| [guardian_bringup](src/guardian_bringup/) | Phillipp | Launch files, EKF/Nav2/RViz config — no nodes of its own |
| [guardian_description](src/guardian_description/) | Phillipp | URDF/xacro robot model (CAD-derived meshes) + Gazebo sim world |
| [guardian_drive](src/guardian_drive/) | Phillipp/Vedant | Mecanum kinematics + Phidgets DCC1120 motor bridge (`phidget_bridge_node`, direct VINT/Phidget22 API — `serial_bridge_node` is an older serial-port-based fallback) |
| [guardian_localization](src/guardian_localization/) | Phillipp | LIDAR republisher (self-occlusion masking) + front/back scan merger, feeds Nav2 |
| [guardian_manipulation](src/guardian_manipulation/) | Dipam | Dual SO-101 arm management + LeRobot bridge |
| [guardian_navigation](src/guardian_navigation/) | Phillipp | Automatic AMCL localization bootstrap on launch |
| [guardian_teleop](src/guardian_teleop/) | Victor | Quest 3 + joystick + keyboard (evdev, real press/release) teleoperation |

---

## Build

```bash
cd ~/AutomousRover_StarkHacks/30_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Build a single package:
```bash
colcon build --packages-select <package_name>
```

---

## Keyboard Teleop (WASD)

Reads the keyboard directly via evdev (real press/release, holds a direction
only while the key is down, multiple keys combine for diagonal driving) — see
`guardian_teleop`'s README for one-time `input`-group setup.

```bash
cd ~/GUARD.IAM_AutomousRover/30_ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run guardian_teleop keyboard_teleop_node
```

Controls:
| Key | Action |
|-----|--------|
| `w` | Forward |
| `s` | Backward |
| `a` | Strafe left |
| `d` | Strafe right |
| `q` | Rotate left |
| `e` | Rotate right |
| `space` / `k` | Stop |
| `+` / `-` | Speed up/down |
| `x` / `Esc` | Quit |

---

## LIDAR

> Two Scanse Sweep units (front + back), each on a stable udev-assigned path —
> see `60_scripts/README.md` for the one-time `/dev/lidar_front`/`/dev/lidar_back`
> setup (do not hardcode `/dev/ttyUSB0`/`1`, which shift with USB plug order).

Brought up automatically by `guardian.launch.py` (both units → merged, masked
`/scan_filtered`). To check a single raw unit directly:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 topic echo /scan --once        # front, raw
ros2 topic echo /scan_back --once   # back, raw
ros2 topic echo /scan_filtered --once  # merged + masked, what Nav2/SLAM use
```

---

## Check Connected Hardware

```bash
# All USB devices
lsusb

# Serial ports (LIDARs — see udev setup above for stable /dev/lidar_front/back)
ls /dev/ttyUSB* /dev/ttyACM*

# Camera
ls /dev/video*
```

---

## Hardware Setup

### Drive Motors — Phidgets DCC1120 / DCM4109

Four Phidgets DCC1120 BLDC motor controllers on one VINT hub, addressed by hub
port (all four controllers report the hub's own serial number, not their own —
see `phidget_bridge_node.py`'s header comment for the full addressing model).
`20_hardware/phidgets/enumerate_phidgets.py` maps hub ports to controllers;
`20_hardware/phidgets/spin_test.py` bench-tests individual wheels.

| Parameter | Value |
|-----------|-------|
| Motor | DCM4109, NEMA23 BLDC, 22.667:1 gearbox |
| Rated max speed | 170 RPM (wheel output) |
| Encoder | Built-in hall sensors — 4-pole x 6 states/electrical-rev x gearbox = 272 ticks/wheel-rev (confirmed on the real robot) |
| Wheel | VEX Pro 6" mecanum, radius 0.0775 m (CAD-measured) |

All of the above live in one place — `guardian_bringup/config/robot_params.yaml`'s
`/**` wildcard block — not scattered per-node.

### Lidar Reset (if node crashes on startup)

The Scanse Sweep segfaults if left in data-streaming mode. Reset before launching
(replace the port with whichever raw `/dev/ttyUSBn` that unit is currently on):

```bash
stty -F /dev/ttyUSB0 115200 && printf 'DX\n' > /dev/ttyUSB0 && sleep 1 && printf 'RR\n' > /dev/ttyUSB0 && sleep 3
```

---

## Launch Files

One unified launch file covers every combination — see
`30_ros2_ws/src/guardian_bringup/README.md` for the full argument list.

| Launch | Purpose | Hardware needed |
|--------|---------|-----------------|
| `guardian.launch.py mode:=mapping` | Drive + LIDAR + SLAM mapping | Motors + Sweep (or none, in sim) |
| `guardian.launch.py` (default) | Full Nav2 stack, autonomous navigation | Motors + Sweep (or none, in sim) |
| `guardian.launch.py use_sim:=false` | Real hardware instead of Gazebo | Motors + Sweep |

`ros2 run guardian_teleop keyboard_teleop_node` drives with keyboard only, standalone.

---

## Mapping (Real Robot)

> Requires the Phidgets DCC1120 motor controllers (see `guardian_drive`) and the
> Scanse Sweep LIDAR(s) on their udev-assigned `/dev/lidar_front`/`/dev/lidar_back`
> ports (see `60_scripts/README.md` for one-time setup).

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch guardian_bringup guardian.launch.py use_sim:=false mode:=mapping
```

Drive around with the keyboard (teleop auto-enables in mapping mode). Map builds
in RViz automatically. SLAM params: `minimum_travel_distance: 0.0` — map updates
even when stationary.

---

## Gazebo Simulation + Autonomous Navigation

> No hardware required — full sim with SLAM, Nav2, and keyboard teleop

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch guardian_bringup guardian.launch.py
```

### What launches
- Gazebo Harmonic world, robot spawns after ~3s
- Front + back GPU lidars merged into `/scan_filtered`; `/odom` + `/tf` from the
  MecanumDrive plugin (holonomic — DiffDrive can't strafe and isn't used)
- SLAM Toolbox (mapping mode) or AMCL + a saved map (navigation mode, default)
- Nav2 lifecycle auto-starts (`autostart: true` — no manual "Startup" click needed)
- RViz opens by default (`rviz:=true`) with the map, costmaps, and global/local path displays
- Teleop keyboard reads the keyboard directly via evdev (real press/release,
  multi-key diagonal driving) — auto-enabled in mapping mode

### Build the map first
Drive around with the keyboard to build the SLAM map before sending Nav2 goals
(`mode:=mapping`, then `30_ros2_ws/src/guardian_bringup/scripts/save_map.sh`).

### Send an autonomous navigation goal
1. Click the **Nav2 Goal** tool (green arrow) in the RViz toolbar
2. Click a position on the map and drag to set heading
3. Release — robot drives autonomously, avoiding obstacles

Verify:
```bash
ros2 topic hz /scan    # ~5Hz
ros2 topic hz /odom    # ~50Hz
ros2 topic hz /map     # ~1Hz
ros2 topic echo /plan --once | head -5   # path published when goal set
ros2 topic echo /cmd_vel | head -5       # velocities while navigating
```

### Nav2 config notes
- `autostart: true` — Nav2 lifecycle starts automatically on launch
- `bond_timeout: 0.0` — disables heartbeat bonds (sim clock vs wall clock mismatch)
- `wait_for_service_timeout: 3000` — gives behavior_server 3s to come up before bt_navigator connects
- Behavior plugin named `backup` (no underscore) — matches what default BT XML expects (`/backup` action server)
- `behavior_plugins: ['spin', 'backup', 'drive_on_heading', 'wait', 'assisted_teleop']`

---

## SLAM Toolbox Key Parameters

Configured in `src/guardian_bringup/config/nav2_params.yaml`:

| Parameter | Value | Why |
|-----------|-------|-----|
| `minimum_travel_distance` | 0.0 | Map updates even when stationary |
| `minimum_travel_heading` | 0.0 | Map updates without rotation |
| `map_update_interval` | 1.0 s | Fast map refresh |
| `max_laser_range` | 8.0 m | Matches Scanse Sweep range |

**Important:** The SLAM launch argument must be `slam_params_file` (not `params_file`) — this is the key that `online_async_launch.py` actually reads.

---

## Useful ROS2 Debug Commands

```bash
# List all active topics
ros2 topic list

# Monitor cmd_vel
ros2 topic echo /cmd_vel

# Monitor wheel RPM
ros2 topic echo /wheel_rpm

# Monitor odometry
ros2 topic echo /odom

# List active nodes
ros2 node list
```
