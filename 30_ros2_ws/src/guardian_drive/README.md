# guardian_drive

**Owner: Phillipp**

Mecanum wheel kinematics, the Phidgets motor bridge, and a dummy odometry source for driving GUARDIAN without hardware.

## Nodes

| Node | Purpose |
|------|---------|
| `mecanum_kinematics_node` | Converts `/cmd_vel` Twist → 4x wheel RPM targets |
| `phidget_bridge_node` (owner: Vedant) | Drives the 4x Phidget DCC1120 controllers over VINT/USB, reads hall-based wheel velocity back, publishes `/odom` + TF. This is the node actually wired into `guardian.launch.py` for real hardware. |
| `serial_bridge_node` (alias `guardian_drive_node`) | Pre-Phidgets ESP32-over-serial bridge. Kept in the codebase but **not launched anywhere** — the robot no longer has ESP32 boards. |
| `dummy_odom_node` | Integrates `/cmd_vel` into fake `/odom` + TF, for testing without a connected drive base |

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/cmd_vel` | `geometry_msgs/Twist` | Subscribed (all nodes) |
| `/wheel_rpm` | `std_msgs/Float32MultiArray` | Published by `mecanum_kinematics_node`, subscribed by `phidget_bridge_node` (FL, FR, BL, BR) |
| `/odom` | `nav_msgs/Odometry` | Published by `phidget_bridge_node` (real) or `dummy_odom_node` (sim/bench test) |

## How to Launch or Run

Normally brought up as part of `guardian_bringup` (`guardian.launch.py`, any mode). To run standalone:

```bash
ros2 run guardian_drive mecanum_kinematics_node
ros2 run guardian_drive phidget_bridge_node
# or, without hardware attached:
ros2 run guardian_drive dummy_odom_node
```

`phidget_bridge_node` degrades gracefully with no Phidget hub attached — it logs
"no controller on hub port N" per wheel and keeps running (motors just no-op,
`/odom` publishes zero), so the ROS-level wiring (params, `/wheel_rpm` in,
`/odom` out) can be sanity-checked on a bench with no hardware connected.

## Known Issues / Dependencies

- `phidget_bridge_node` addresses wheels by **VINT hub port**, not serial number — all 4 DCC1120 controllers share the hub's own serial (`hub_serial`, default `782265`), see the node's file header for the port map and the two bench calibration steps (`rescale_factor`, `wheel_directions`).
- `dummy_odom_node` and `phidget_bridge_node` both publish `/odom` — never run both at once.
- Depends on `tf2_ros` for the odom→base_link transform, and the `Phidget22` Python package + `libphidget22` for `phidget_bridge_node`.
