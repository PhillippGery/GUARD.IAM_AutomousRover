# guardian_bringup

**Owner: Phillipp**

Launch files and YAML/RViz/BT configuration that assemble the other GUARDIAN packages into runnable robot modes — no nodes of its own.

## Nodes

None. This package only contains launch files and config.

## Topics

None published or subscribed directly — see the launch files below for which packages/nodes are brought up.

## Launch Files

One unified launch file covers every sim/real × mapping/navigation combination —
the separate `guardian_sim`/`guardian_nav`/`guardian_mapping`/`guardian_real`/
`guardian_full`/`guardian_teleop` launch files that used to exist here have been
removed; everything they did is now an argument on `guardian.launch.py`.

| File | Purpose |
|------|---------|
| `guardian.launch.py` | Every mode: sim or real hardware, mapping (SLAM) or navigation (AMCL + saved map) |

```bash
# Sim, autonomous navigation, RViz open (defaults)
ros2 launch guardian_bringup guardian.launch.py

# Real hardware, build a new map (mapping a sim world isn't useful)
ros2 launch guardian_bringup guardian.launch.py use_sim:=false mode:=mapping

# Real hardware, navigation
ros2 launch guardian_bringup guardian.launch.py use_sim:=false

# Headless (no RViz) — e.g. autonomous/unattended startup
ros2 launch guardian_bringup guardian.launch.py rviz:=false

# Real hardware navigation mode, but without teleop (pure autonomous)
ros2 launch guardian_bringup guardian.launch.py use_sim:=false teleop:=false

# Sim mapping mode, but with manual teleop override enabled too
ros2 launch guardian_bringup guardian.launch.py teleop:=true
```

Key arguments: `use_sim` (`true`/`false`, default `true`), `mode` (`mapping`/`navigation`,
default `navigation`), `rviz` (default `true`), `teleop` (`true`/`false` explicitly
overrides in either mode; left empty it auto-selects: **always on for real
hardware regardless of mode** — the Xbox controller can override autonomous
behavior the instant the robot is powered on — and mode-based for sim, on for
mapping, off for navigation), `map` (map yaml to load in navigation mode), `known_pose`
(auto: known spawn point in sim, AMCL global localization on real hardware),
`world`/`spawn_x`/`spawn_y`/`spawn_z` (sim only). See `guardian.launch.py`'s
`DeclareLaunchArgument` calls for the full list and descriptions.

The aliases in `60_scripts/guardiam_env.sh` (`guardiam_sim`, `guardiam_map`,
`guardiam_real`) wrap the common cases.

## Xbox Controller Controls

Teleop is an Xbox controller via the standard `joy` + `teleop_twist_joy`
packages (config: `xbox_teleop.yaml`), not GUARDIAN's own keyboard node.
Connect over USB or Bluetooth — both work, USB is more reliable.

| Control | Action |
|---------|--------|
| **LB** (hold) | Deadman switch — nothing moves unless this is held, matching the old keyboard teleop's "only while held" safety |
| Left stick vertical | Forward / backward |
| Left stick horizontal | Strafe left / right |
| Right stick horizontal | Rotate |

Button/axis numbers were verified empirically (`buttons`/`axes` indices vary
by controller and driver, don't trust the "standard" xpad numbering blindly):
`enable_button: 6` (LB), `axis_linear.x: 1` (left stick Y), `axis_linear.y: 0`
(left stick X), `axis_angular.yaw: 2` (right stick X).

**No turbo currently** — RB was meant to be the turbo button
(`enable_turbo_button`), but on this controller/driver combo RB's button
press reaches the kernel fine (raw evdev shows `BTN_TR` firing) yet never
appears in `joy_node`'s published `/joy` message at all. Root cause not
found yet; `enable_turbo_button` is set to `-1` (disabled) rather than
pointing at a button that silently never fires. Speed scales in
`xbox_teleop.yaml` (`scale_linear`, `scale_angular`) are a fixed ~0.55 of
the real 170 RPM-rated max — bump those if turbo gets fixed, or just to
drive faster in the meantime.

## Config Files

| File | Purpose |
|------|---------|
| `ekf_params.yaml` / `ekf_params_real.yaml` | robot_localization EKF configuration (sim / real) |
| `nav2_params.yaml` | Nav2 navigation stack parameters — one file for both sim and real (a separate `nav2_params_real.yaml` used to exist but was never wired into the launch file and had drifted stale; deleted) |
| `robot_params.yaml` | Robot geometry, serial port, hardware params (single source of truth via a `/**` wildcard block) |
| `lidar_filter_params.yaml` | Per-sensor LIDAR self-occlusion masking (front/back) |
| `xbox_teleop.yaml` | `teleop_twist_joy` axis/button mapping and speed scales for the Xbox controller |
| `navigate_to_pose.xml` | Nav2 behavior tree for `navigate_to_pose` |
| `guardian.rviz` | RViz display config — one file for both mapping and navigation mode, since Nav2 core runs in both |

## Known Issues / Dependencies

- `bt_navigator` needs `wait_for_service_timeout: 3000` in `nav2_params*.yaml` or it fails to find `behavior_server` on slower hardware.
- `behavior_server` must be listed before `bt_navigator` in the `node_names` array of the lifecycle manager config.
- `autostart: true` in the Nav2 params — the Nav2 lifecycle starts automatically on launch, no manual "Startup" click needed.
- Depends on `guardian_description`, `guardian_drive`, `guardian_localization`, `guardian_manipulation`, `guardian_navigation`, plus `joy`, `teleop_twist_joy`, `nav2_bringup`, `robot_localization`, `slam_toolbox`, `l3xz_sweep_scanner`, `rviz2`, `xacro`.
- Teleop is Xbox controller based (`joy` + `teleop_twist_joy`, config in `xbox_teleop.yaml`) — GUARDIAN's own `guardian_teleop` keyboard node is no longer wired into this launch file (kept in the codebase, just unused here).
- RealSense D415 camera node removed (was `realsense2_camera` in the real-hardware launch branch) — the pinned realsense-ros release doesn't support ROS_DISTRO=jazzy and broke full-workspace builds. Re-add once a Jazzy-compatible release exists upstream.
