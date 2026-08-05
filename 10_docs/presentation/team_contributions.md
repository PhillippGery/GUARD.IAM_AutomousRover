# GUARD.IAM — Team Contributions (Corrected)

## Team

| Member | Role | Affiliation |
|--------|------|-------------|
| **Phillipp Gery** | Lead — ROS2, Systems, Electrical | MS ACS, Purdue — Fulbright Scholar |
| **Vedant Patkar** | Drive hardware, LeRobot pipeline | MS ACS, Purdue |
| **Victor Hu** | Mechanical design, CAD, 3D printing | MS ACS, Purdue |
| **Pier** | Quest 3 / overhead camera, LeRobot ACT | MS ACS, Purdue |

---

## Phillipp — Lead, ROS2 Architecture, Electrical

- Full ROS2 system architecture (all packages, launch system, topic graph)
- Nav2 autonomous navigation stack — behavior trees, costmaps, planner config
- SLAM — slam_toolbox integration, mapping pipeline
- EKF sensor fusion — robot_localization, T265 + encoders + IMU
- Mecanum inverse kinematics node
- LIDAR integration — lidar_republisher_node (QoS bridge, frame_id, timestamp fix)
- Web control interface — Foxglove layout, web_ops_node, service call buttons (Start Mapping / Navigation / Save Map / Start Demo)
- Boot automation — systemd services, full stack auto-launch on power-on
- Simulation — Gazebo integration, URDF, sim↔real parity (same launch file, use_sim flag)
- Waypoint system — demo_mission_node, set_waypoint_node, browser recording
- **Electrical system** — full electrical planning and execution: power architecture (24V LiPo, buck converters, fuse boxes, rails), wiring, cable management

---

## Vedant — Drive Hardware, LeRobot Pipeline

- Phidgets BLDC bridge node (phidget_bridge_node) — 4× DCC1120 over VINT/USB, hall-effect velocity feedback
  - **Status: laggy motor response, PID tuning in progress**
- LeRobot data collection and training pipeline (together with Pier)

---

## Victor — Mechanical Design

- Full chassis CAD design and planning (SolidWorks)
- Hardware layout and component placement
- 3D printing — print planning, validation, fitting on robot (together with Phillipp)
- **Not responsible for electrical — that was Phillipp**

---

## Pier — Quest 3, Overhead Camera, ACT

- Meta Quest 3 integration — head tracking, live camera feed
- Overhead 3-DOF camera system (PTZ)
- LeRobot ACT policy training and inference pipeline (together with Vedant)
- Electrical schematic documentation (in progress)

---

## Notes

- **Dipam** — was listed in earlier docs as manipulation owner. No longer on the team. All manipulation/LeRobot work is Vedant + Pier.
- System_overview.md still references Arduino Mega / L298N / ESP32 for motor control — **outdated**, replaced by Phidgets DCC1120 + VINT hub.
