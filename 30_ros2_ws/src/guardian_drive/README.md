# guardian_drive

**Owner: Phillipp**

Mecanum wheel kinematics, Arduino serial bridge, and a dummy odometry source for driving GUARDIAN without hardware.

## Nodes

| Node | Purpose |
|------|---------|
| `mecanum_kinematics_node` | Converts `/cmd_vel` Twist → 4x wheel RPM targets |
| `serial_bridge_node` (alias `guardian_drive_node`) | Sends RPM targets to Arduino, reads encoder feedback, publishes `/odom` |
| `dummy_odom_node` | Integrates `/cmd_vel` into fake `/odom` + TF, for testing without a connected drive base |

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/cmd_vel` | `geometry_msgs/Twist` | Subscribed (all nodes) |
| `/wheel_rpm` | `std_msgs/Float32MultiArray` | Published by `mecanum_kinematics_node`, subscribed by `serial_bridge_node` (FL, FR, BL, BR) |
| `/odom` | `nav_msgs/Odometry` | Published by `serial_bridge_node` (real) or `dummy_odom_node` (sim/bench test) |

## How to Launch or Run

Normally brought up as part of `guardian_bringup` (e.g. `guardian_nav.launch.py`, `guardian_real.launch.py`). To run standalone:

```bash
ros2 run guardian_drive mecanum_kinematics_node
ros2 run guardian_drive serial_bridge_node
# or, without hardware attached:
ros2 run guardian_drive dummy_odom_node
```

## Known Issues / Dependencies

- `serial_bridge_node` expects an Arduino/Phidgets-side firmware speaking the same wheel RPM protocol; `serial_port_left`/`serial_port_right` default to `/dev/ttyACM0`/`/dev/ttyACM1`.
- `dummy_odom_node` and `serial_bridge_node` both publish `/odom` — never run both at once.
- Depends on `tf2_ros` for the odom→base_link transform.
