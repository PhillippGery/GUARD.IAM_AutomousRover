# guardian_description

Xacro/URDF robot model for GUARDIAN (no meshes checked in yet — placeholder geometry only).

## Nodes

None. This package only contains URDF/xacro and Gazebo world files.

## Topics

None directly — `robot_state_publisher` (launched from `guardian_bringup`) consumes this package's URDF to publish `/robot_description` and TF.

## Files

| File | Purpose |
|------|---------|
| `urdf/guardian.urdf.xacro` | Real-robot model stub (fill in with CAD dimensions) |
| `urdf/guardian_sim.urdf.xacro` | Simulation model with Gazebo plugins |
| `worlds/guardian_world.sdf` | Gazebo simulation world |

## How to Use

Not run directly — included by `guardian_bringup` launch files via `xacro`:

```bash
ros2 launch guardian_bringup guardian_sim.launch.py
```

## Known Issues / Dependencies

- Wheel base and wheel radius in the xacro must match `guardian_bringup/config/robot_params.yaml` after the physical build.
- No mesh files yet — visual/collision geometry is primitive shapes only.
- Must include T265 camera frame, LIDAR frame, and arm base frames for a correct TF tree once CAD is finalized.
- Depends on `xacro` for macro expansion at launch time.
