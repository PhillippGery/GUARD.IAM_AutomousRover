# GUARD.IAM ROS2 Workspace

## Project Overview
Autonomous confined space intervention rover with dual SO-101 arms.
Purdue University ICON + MARS Lab. Supervisor: Prof. Yu She.

## Hardware
- AMD Ryzen AI MiniPC (ROS2 Jazzy)
- 4x Phidgets DCM4109 BLDC motors + DCC1120 drivers via VINT Hub
- VEX Pro 6" mecanum wheels
- Scanse Sweep V1 LIDAR
- Intel RealSense D415
- SO-101 LeRobot arms x2
- 6S 22.2V LiPo batteries x2 in parallel

## Package Structure
- `guardian_bringup` — launch files + YAML/RViz/BT configs only, no nodes
- `guardian_description` — URDF/xacro robot model + Gazebo worlds
- `guardian_drive` — mecanum kinematics, serial bridge, dummy odom (sim/bench)
- `guardian_localization` — LIDAR republisher feeding Nav2 + EKF
- `guardian_manipulation` — dual SO-101 arm control + Quest arm-teleop bridge (LeRobot)
- `guardian_navigation` — high-level autonomous mission behaviors (Nav2 simple commander)
- `guardian_teleop` — keyboard/joystick/Quest base teleoperation
- `l3xz_sweep_scanner` — third-party sensor driver package (vendored, do not restructure)

## Key ROS2 Topics
- `/wheel_rpm` — `std_msgs/Float32MultiArray` [FL,FR,BL,BR] RPM — `guardian_drive` kinematics → serial bridge
- `/odom` — `nav_msgs/Odometry` — from `guardian_drive` (real serial bridge or dummy_odom_node in sim)
- `/scan` — `sensor_msgs/LaserScan` — from `guardian_localization`'s LIDAR republisher, feeds Nav2 costmaps
- `/cmd_vel` — `geometry_msgs/Twist` — Nav2 / teleop velocity commands into `guardian_drive`

## Known Working Fixes
- `bt_navigator`: set `wait_for_service_timeout: 3000` in `nav2_params*.yaml`
- `behavior_server` must be listed before `bt_navigator` in the lifecycle manager's `node_names`
- `autostart: False` in Nav2 params — start the Nav2 lifecycle manually (click "Startup" in RViz) after launch

## Current Status
- Sim working with Nav2 autonomous navigation
- Real robot: motors ordered (Phidgets DCM4109), chassis being built
- `guardian_manipulation` and `guardian_navigation` nodes (`arm_manager_node`, `teleop_bridge_node`, `demo_mission_node`) are stubs — TODOs pending real implementation
- Workspace just reorganized into per-responsibility packages (bringup no longer contains any Python nodes)

## Next Steps
- Implement `arm_manager_node` / `teleop_bridge_node` against the LeRobot API once arms arrive
- Wire `guardian_manipulation` into a `guardian_bringup` launch file once arm nodes are functional
- Replace `guardian_description` placeholder geometry with real CAD-derived meshes
