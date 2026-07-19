# guardian_manipulation

**Owner: Dipam**

Dual SO-101 LeRobot arm management — follower arm control, LeRobot policy inference, and the Quest teleop bridge.

## Nodes

| Node | Purpose |
|------|---------|
| `arm_manager_node` | Manages both SO-101 follower arms, coordinates dual-arm operation (stub — LeRobot init TODO) |
| `teleop_bridge_node` | Bridges Quest hand tracking input to LeRobot arm commands (stub — subscriber/publisher TODO) |

## Scripts

| Script | Purpose |
|--------|---------|
| `scripts/collect_training_data.py` | Records leader arm demonstrations for LeRobot imitation learning (stub) |

## Topics

Both nodes are currently stubs with all pub/sub commented out pending LeRobot integration. Planned:

| Topic | Type | Direction |
|-------|------|-----------|
| `/quest/hand_poses` | `geometry_msgs/PoseArray` | Subscribed by `teleop_bridge_node` (planned) |
| `/arm_commands` | `std_msgs/Float32MultiArray` | Published by `teleop_bridge_node`, subscribed by `arm_manager_node` (planned) |
| `/arm_joint_states` | `sensor_msgs/JointState` | Published by `arm_manager_node` (planned) |

## How to Launch or Run

Not yet wired into a `guardian_bringup` launch file. Run standalone:

```bash
ros2 run guardian_manipulation arm_manager_node
ros2 run guardian_manipulation teleop_bridge_node
python3 src/guardian_manipulation/scripts/collect_training_data.py
```

## Known Issues / Dependencies

- Both nodes are unimplemented stubs (see inline `TODO`s) — no actual arm I/O yet.
- Requires the `lerobot` Python package (`pip install -e lerobot[feetech]`, not a rosdep-resolvable package — install manually, see `60_scripts/setup_amd_minipc.sh`).
- Hardware: 2x SO-101 follower arms over USB-C to the AMD MiniPC.
- See [40_lerobot/README.md](../../../40_lerobot/README.md) and [10_docs/setup/dependencies.md](../../../10_docs/setup/dependencies.md) for LeRobot setup.
