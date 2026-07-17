# guardian_arms

**Owner: Dipam**

Dual SO-101 LeRobot arm management — follower arm control, LeRobot policy inference, and teleop bridge.

## Nodes

| Node | Purpose |
|------|---------|
| `arm_manager_node` | Manages both SO-101 follower arms, coordinates dual-arm operation |
| `teleop_bridge_node` | Bridges Quest hand tracking input to LeRobot arm commands |

## Scripts

| Script | Purpose |
|--------|---------|
| `collect_training_data.py` | Records leader arm demonstrations for LeRobot imitation learning |

## Hardware

- 2× SO-101 follower arms (USB-C to AMD MiniPC)
- LeRobot library for policy inference

## LeRobot Setup

See [40_lerobot/README.md](../../../40_lerobot/README.md) and [10_docs/setup/dependencies.md](../../../10_docs/setup/dependencies.md).
