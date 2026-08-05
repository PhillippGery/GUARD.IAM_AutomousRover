# GUARD.IAM — Professor Handoff Presentation Brief

**Date:** 2026-08-06
**Audience:** Prof. Yu She (MARS Lab) + Prof. Shaoshuai Mou (ICON)
**Goal:** Final project handoff — show what was built, where the money went, what the robot can do

---

## Financials

### Lab Budget (Prof. She)
| Category | Amount |
|----------|--------|
| Phidgets invoices (4 invoices, incl. tax/import) | $1,611.14 |
| Amazon orders (power, cables, structure) | ~$350 |
| Tools (crimpers, Home Depot) | ~$80 |
| Misc contingency | ~$230 |
| **Total lab spend** | **~$2,295** |

### Sponsored Value
| Sponsor | What | Value |
|---------|------|-------|
| AMD | Ryzen AI MiniPC (8845HS) + SO-101 arms ×2 + GPU cloud | ~$1,100 |
| Espressif | ESP32-P4-EYE ×2 | ~$50 |
| Qualcomm | Rubik Pi 3 | $179 |
| CubeMars | AKA10-9 actuators ×4 (20% discount) | ~$640 |
| StepperOnline | 57BLR50 motors ×4 (20% discount) | ~$250 |
| Ouster | OS0 3D LIDAR (active — Duncan engaged) | ~$4,500 |
| **Total sponsored** | | **~$6,700** |

**Total platform value: ~$9,000**

---

## Capabilities

### ✅ Working Now
- Holonomic mecanum drive (omnidirectional — vx, vy, yaw simultaneously)
- Phidgets BLDC motor control — 4× DCM4109 + DCC1120, direct USB via VINT hub
- SLAM mapping — Scanse Sweep + slam_toolbox
- Autonomous navigation — Nav2 full stack, obstacle avoidance
- Waypoint missions — pre-defined + live recording from browser
- Xbox controller teleoperation
- **Web interface** (Foxglove over ZeroTier) — browser anywhere on network:
  - 3D robot view with live URDF mesh
  - Live occupancy map + planned path
  - One-click: Start Mapping / Start Navigation / Save Map / Start Demo
  - Nav status + readiness indicator
  - Waypoint recording from browser
- Boot-to-ready — full stack auto-launches on power-on (systemd)
- Simulation — Gazebo, full URDF, same launch file as real robot (use_sim flag)
- Quest 3 MR teleoperation — head tracking → PTZ camera, live camera feed
- Bimanual SO-101 arm teleoperation (leader/follower)
- LeRobot data collection — 3 cameras (overhead 3-DOF + 2 wrist)
- ACT policy training and inference — validated on AMD MiniPC

### 🟡 In Progress
- Phidgets motor PID tuning — laggy response, Vedant fixing today
- Electrical schematic documentation — Pier drawing

### ⚠️ Known Weak Point
- **Scanse Sweep LIDAR** — 2D only, 5 Hz, 40m range — works but limiting
- Ouster OS0 3D LIDAR (sponsor engaged, not arrived)

---

## Full Component List (for replication)

### Purchased (Lab Budget)
See: `10_docs/hardware/Componet_List.xlsx` — full Phidgets + Amazon order list

### Sponsored
- AMD Ryzen AI MiniPC (8845HS) — compute + AI inference
- SO-101 follower arms ×2 — manipulation
- Espressif ESP32-P4-EYE ×2
- Qualcomm Rubik Pi 3
- CubeMars AKA10-9 KV60 ×4 (backup actuators)

### From Lab
- VEX Pro 6" mecanum wheels ×4
- Scanse Sweep V1 LIDAR
- Intel RealSense D415 depth camera
- Intel RealSense T265 tracking camera
- Turnigy Multistar 6S 22.2V 20Ah LiPo ×2
- Aluminum extrusion 1010 (chassis frame)

---

## Team Contributions
See: `10_docs/presentation/team_contributions.md`

---

## Path Forward
- Formal ICON Student Team Project — Prof. She + Prof. Mou
- Ouster OS0 3D LIDAR integration when hardware arrives
- Phidgets PID tuning
- ICRA/IROS 2027 target
