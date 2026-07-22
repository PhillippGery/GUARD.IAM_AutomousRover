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

# Navigation mode, but with manual teleop override enabled too
ros2 launch guardian_bringup guardian.launch.py use_sim:=false teleop:=true

# Mapping mode, but without teleop (pure autonomous SLAM exploration)
ros2 launch guardian_bringup guardian.launch.py use_sim:=false mode:=mapping teleop:=false
```

Key arguments: `use_sim` (`true`/`false`, default `true`), `mode` (`mapping`/`navigation`,
default `navigation`), `rviz` (default `true`), `teleop` (`true`/`false` explicitly
overrides in either mode; left empty it auto-selects: on for mapping, off for
navigation), `map` (map yaml to load in navigation mode), `known_pose`
(auto: known spawn point in sim, AMCL global localization on real hardware),
`world`/`spawn_x`/`spawn_y`/`spawn_z` (sim only). See `guardian.launch.py`'s
`DeclareLaunchArgument` calls for the full list and descriptions.

The aliases in `60_scripts/guardiam_env.sh` (`guardiam_sim`, `guardiam_map`,
`guardiam_real`) wrap the common cases.

## Config Files

| File | Purpose |
|------|---------|
| `ekf_params.yaml` / `ekf_params_real.yaml` | robot_localization EKF configuration (sim / real) |
| `nav2_params.yaml` / `nav2_params_real.yaml` | Nav2 navigation stack parameters (sim / real) |
| `robot_params.yaml` | Robot geometry, serial port, hardware params (single source of truth via a `/**` wildcard block) |
| `lidar_filter_params.yaml` | Per-sensor LIDAR self-occlusion masking (front/back) |
| `navigate_to_pose.xml` | Nav2 behavior tree for `navigate_to_pose` |
| `guardian.rviz` | RViz display config — one file for both mapping and navigation mode, since Nav2 core runs in both |

## Known Issues / Dependencies

- `bt_navigator` needs `wait_for_service_timeout: 3000` in `nav2_params*.yaml` or it fails to find `behavior_server` on slower hardware.
- `behavior_server` must be listed before `bt_navigator` in the `node_names` array of the lifecycle manager config.
- `autostart: true` in the Nav2 params — the Nav2 lifecycle starts automatically on launch, no manual "Startup" click needed.
- Depends on `guardian_description`, `guardian_drive`, `guardian_localization`, `guardian_manipulation`, `guardian_navigation`, `guardian_teleop`, plus `nav2_bringup`, `robot_localization`, `slam_toolbox`, `l3xz_sweep_scanner`, `rviz2`, `xacro`.
- RealSense D415 camera node removed (was `realsense2_camera` in the real-hardware launch branch) — the pinned realsense-ros release doesn't support ROS_DISTRO=jazzy and broke full-workspace builds. Re-add once a Jazzy-compatible release exists upstream.
