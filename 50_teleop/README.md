# Teleop

**Owner: Victor**

Meta Quest 3 WebSocket bridge for mixed reality teleoperation.

## Subfolders

| Folder | Contents |
|--------|----------|
| [quest_bridge/](quest_bridge/) | WebSocket server that bridges Quest 3 data to ROS2 |

## How It Works

1. A companion app runs on the Meta Quest 3 and streams head pose + hand tracking data over WebSocket.
2. `websocket_bridge.py` runs on the AMD MiniPC, receives the WebSocket data, and forwards it to ROS2 topics.
3. `quest_bridge_node` (in `guardian_teleop`) subscribes to those topics and publishes standard ROS2 messages.

## Setup

```bash
pip install websockets
python 50_teleop/quest_bridge/websocket_bridge.py
```

Then run `ros2 run guardian_teleop keyboard_teleop_node` (or launch `guardian.launch.py`, which auto-enables teleop in mapping mode).
