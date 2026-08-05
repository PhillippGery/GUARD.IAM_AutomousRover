# guardian_navigation

**Owner: Phillipp**

High-level autonomous navigation behaviors for GUARDIAN, built on the Nav2 simple-commander API.

## Nodes

| Node | Purpose |
|------|---------|
| `localization_bootstrap_node` | Actively used — automatic AMCL bootstrap on launch. Fast path if the start pose is known (`known_pose:=true`, e.g. sim's deterministic spawn point); otherwise drives AMCL's global localization + a `/spin` action until covariance converges, retrying on failure. Wired into `guardian.launch.py`'s navigation mode. |
| `demo_mission_node` | Unused example/stub — a hardcoded two-waypoint `BasicNavigator` demo mission. Not part of the active launch pipeline; kept only as a reference for the Nav2 simple-commander API. |

## Topics / Services / Actions (`localization_bootstrap_node`)

- Publishes `/localized` (`Bool`) and `/initialpose` (`PoseWithCovarianceStamped`)
- Subscribes `/amcl_pose` (`PoseWithCovarianceStamped`) to check localization covariance
- Calls the `/reinitialize_global_localization` service (`std_srvs/Empty`)
- Uses the `/spin` action (`nav2_msgs/action/Spin`) to rotate in place while localizing

## How to Launch or Run

Brought up automatically by `guardian_bringup`'s `guardian.launch.py` in navigation
mode — not normally run standalone. Key parameters (see the node for the full
list): `known_pose`, `known_pose_x`/`known_pose_y`/`known_pose_yaw`,
`covariance_threshold`, `max_attempts`, `retry_delay_sec`.

## Known Issues / Dependencies

- Depends on Nav2 (`nav2_msgs`, `std_srvs`) and AMCL being active.
- Pure-rotation global localization converges position but is weak at
  disambiguating yaw in large, sparse/repetitive spaces — translational motion
  helps more than spinning in place there. A known limitation, not yet addressed.
- `demo_mission_node` depends on `nav2_simple_commander` and `nav2_bringup` but
  is not exercised by any current launch file.
