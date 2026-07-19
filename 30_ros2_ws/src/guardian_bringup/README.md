# guardian_bringup

**Owner: Phillipp**

Launch files and YAML/RViz/BT configuration that assemble the other GUARDIAN packages into runnable robot modes — no nodes of its own.

## Nodes

None. This package only contains launch files and config.

## Topics

None published or subscribed directly — see the launch files below for which packages/nodes are brought up.

## Launch Files

| File | Purpose |
|------|---------|
| `guardian_full.launch.py` | Full autonomous mode — all nodes |
| `guardian_sim.launch.py` | Gazebo simulation with Nav2 |
| `guardian_nav.launch.py` | Navigation only — no arms, no Quest |
| `guardian_mapping.launch.py` | SLAM mapping mode (slam_toolbox) |
| `guardian_real.launch.py` | Real-robot bringup (sensors + Nav2, no sim) |
| `guardian_teleop.launch.py` | Teleop fallback — keyboard/joystick + drive |

```bash
ros2 launch guardian_bringup guardian_sim.launch.py
ros2 launch guardian_bringup guardian_nav.launch.py
```

## Config Files

| File | Purpose |
|------|---------|
| `ekf_params.yaml` / `ekf_params_real.yaml` | robot_localization EKF configuration (sim / real) |
| `nav2_params.yaml` / `nav2_params_real.yaml` | Nav2 navigation stack parameters (sim / real) |
| `robot_params.yaml` | Robot geometry, serial port, hardware params |
| `navigate_to_pose.xml` | Nav2 behavior tree for `navigate_to_pose` |
| `guardian.rviz`, `guardian_nav.rviz`, `guardian_mapping.rviz`, `guardian_real.rviz` | RViz display configs per mode |

## Known Issues / Dependencies

- `bt_navigator` needs `wait_for_service_timeout: 3000` in `nav2_params*.yaml` or it fails to find `behavior_server` on slower hardware.
- `behavior_server` must be listed before `bt_navigator` in the `node_names` array of the lifecycle manager config.
- `autostart` is `False` in the Nav2 params — the Nav2 lifecycle must be started manually (click "Startup" in RViz) after launch.
- Depends on `guardian_description`, `guardian_drive`, `guardian_localization`, `guardian_manipulation`, `guardian_navigation`, `guardian_teleop`, plus `nav2_bringup`, `robot_localization`, `slam_toolbox`, `realsense2_camera`, `l3xz_sweep_scanner`, `rviz2`, `xacro`.
