# guardian_teleop

**Owner: Victor**

Meta Quest 3 mixed-reality teleoperation, keyboard, and joystick fallback control for driving GUARDIAN.

## Nodes

| Node | Purpose |
|------|---------|
| `keyboard_teleop_node` | Reads keyboard input, publishes `/cmd_vel` for manual drive |
| `joystick_fallback_node` | Maps joystick `/joy` input to `/cmd_vel` for manual rover drive |
| `quest_bridge_node` | Receives Quest head/hand tracking via WebSocket, publishes to ROS2 topics (stub — pub/sub TODO) |

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/cmd_vel` | `geometry_msgs/Twist` | Published by `keyboard_teleop_node` and `joystick_fallback_node` |
| `/joy` | `sensor_msgs/Joy` | Subscribed by `joystick_fallback_node` |
| `/quest/head_pose` | `geometry_msgs/PoseStamped` | Published by `quest_bridge_node` (planned) |
| `/quest/hand_poses` | `geometry_msgs/PoseArray` | Published by `quest_bridge_node` (planned) |

## How to Launch or Run

```bash
# auto-enabled in guardian.launch.py's mapping mode, or standalone:
ros2 run guardian_teleop keyboard_teleop_node
ros2 run guardian_teleop joystick_fallback_node
```

`keyboard_teleop_node` needs one-time permission to read the keyboard
directly (see below) before it will run.

### `keyboard_teleop_node` — why it reads the keyboard directly

WASD drives only while a key is physically held, and holding several keys
together drives at an angle (e.g. `w`+`a`). Getting genuine press/release
state for that requires reading the keyboard device directly via
[evdev](https://python-evdev.readthedocs.io/) — a plain terminal
(`termios` raw mode) has no key-up event at all, and relying on OS key
auto-repeat to approximate "still held" breaks as soon as a second key is
pressed (standard X11/terminal auto-repeat only re-sends *one* key at a
time, so a two-key combo would silently stop after ~1s).

**One-time setup** — reading `/dev/input/eventN` needs the `input` group:

```bash
sudo apt install python3-evdev   # or: pip install --break-system-packages evdev
sudo usermod -aG input $USER
# then log out and back in (or `newgrp input` for just the current shell)
```

Without this, the node exits immediately with a clear error rather than
silently failing.

Parameters: `linear_speed`, `angular_speed`, `device_path` (auto-detected
if unset), `grab_device` (default `false` — set `true` to exclusively
capture the keyboard so keystrokes don't also reach whatever window has
focus; while grabbed, your keyboard is unavailable to everything else).

## Known Issues / Dependencies

- `keyboard_teleop_node` and `joystick_fallback_node` both publish `/cmd_vel` — don't run both at once, or drive commands will race.
- `keyboard_teleop_node` requires `input` group membership to read `/dev/input` — see above.
- `quest_bridge_node` is an unimplemented stub; the WebSocket bridge server itself lives outside this package — see [50_teleop/quest_bridge/](../../../50_teleop/quest_bridge/).
- Depends on `teleop_twist_keyboard` (system package) and a `sensor_msgs/Joy` publisher (e.g. a `joy` node) for `joystick_fallback_node`.
