# guardian_localization

**Owner: Phillipp**

LIDAR republishing and sensor preprocessing to feed Nav2 costmaps and the EKF.

## Nodes

| Node | Purpose |
|------|---------|
| `lidar_republisher_node` | Republishes Scanse Sweep scan data onto the topic/frame Nav2 expects, with a small timestamp offset correction |

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/sweep/scan` | `sensor_msgs/LaserScan` | Subscribed (default `input_topic`, from `l3xz_sweep_scanner`) |
| `/scan_filtered` | `sensor_msgs/LaserScan` | Published (default `output_topic`, consumed by Nav2 costmaps, SLAM Toolbox, and AMCL) |

## How to Launch or Run

Brought up as part of `guardian_bringup` (`guardian.launch.py`, any mode). To run standalone:

```bash
ros2 run guardian_localization lidar_republisher_node
```

### Self-occlusion masking (`mask_angle_ranges`)

If the LIDAR's mount permanently blocks part of its rotation (e.g. a wall or
post directly behind it), that sector reads a constant near-zero range —
left alone, SLAM/AMCL/costmaps would see it as a real, robot-attached
obstacle. Set `mask_angle_ranges` to blank those beams out (published as
`NaN`, which downstream consumers ignore rather than treat as an obstacle
or as clear space):

```yaml
lidar_republisher_node:
  ros__parameters:
    mask_angle_ranges: "2.356,-2.356"   # "lo1,hi1,lo2,hi2,..." in radians,
                                        # in the scan's own angle_min/angle_max
                                        # frame. Wraps across +-pi. Above
                                        # example blanks a ~90deg wedge
                                        # centered directly behind the sensor.
```

Empty string (default) = no masking, i.e. today's behavior is unchanged.

Because every parameter here (topics, frame_id, the mask) is per node
*instance*, a second physical LIDAR is just another `lidar_republisher_node`
launched with its own `name=`, `input_topic`/`output_topic`, and
`mask_angle_ranges` — no code changes needed.

## Known Issues / Dependencies

- `input_topic`, `output_topic`, `frame_id`, and `timestamp_offset_sec` are all parameters — check `guardian_bringup/config/robot_params.yaml` for the values actually used at launch.
- `mask_angle_ranges` is a comma-separated string, not a native double array — ROS2/rclpy (observed on Jazzy) mis-types a `double_array` parameter declared with an empty-list default when a non-empty override is applied later (`InvalidParameterTypeException`, override read back as `BYTE_ARRAY`), regardless of an explicit `ParameterDescriptor`. The string workaround sidesteps it.
- Depends on `l3xz_sweep_scanner` publishing `/sweep/scan` upstream.
