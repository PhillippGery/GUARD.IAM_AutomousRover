# MIT License
# GUARDIAN — StarkHacks 2026
# Launch: guardian
# Purpose: single unified entry point for GUARDIAN — sim/real x mapping/navigation
#
#   use_sim:=true/false   — Gazebo simulation vs real sensors/drive
#   mode:=mapping/navigation — SLAM Toolbox live mapping vs AMCL on a saved map

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             IncludeLaunchDescription, OpaqueFunction,
                             RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def launch_setup(context, *args, **kwargs):
    bringup_dir     = get_package_share_directory('guardian_bringup')
    description_dir = get_package_share_directory('guardian_description')
    slam_dir        = get_package_share_directory('slam_toolbox')

    nav2_params       = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    slam_params       = os.path.join(bringup_dir, 'config', 'slam_params.yaml')
    amcl_params       = os.path.join(bringup_dir, 'config', 'amcl_params.yaml')
    map_server_params = os.path.join(bringup_dir, 'config', 'map_server_params.yaml')
    robot_params      = os.path.join(bringup_dir, 'config', 'robot_params.yaml')

    use_sim  = LaunchConfiguration('use_sim').perform(context).lower() == 'true'
    mode     = LaunchConfiguration('mode').perform(context).lower()
    map_yaml = LaunchConfiguration('map').perform(context)
    rviz     = LaunchConfiguration('rviz').perform(context).lower() == 'true'

    # teleop: '' (default) means "auto" — on for mapping (drive while
    # building the map, existing behavior), off for navigation (autonomous,
    # opt in with teleop:=true for a manual driving test).
    teleop_arg = LaunchConfiguration('teleop').perform(context)
    teleop = (teleop_arg.lower() == 'true') if teleop_arg else (mode == 'mapping')

    # known_pose: '' (default) means "auto" — true in sim (deterministic
    # spawn point), false in real (unknown start, use global localization).
    known_pose_arg = LaunchConfiguration('known_pose').perform(context)
    known_pose = (known_pose_arg.lower() == 'true') if known_pose_arg else use_sim
    start_x   = float(LaunchConfiguration('start_x').perform(context))
    start_y   = float(LaunchConfiguration('start_y').perform(context))
    start_yaw = float(LaunchConfiguration('start_yaw').perform(context))

    # Overridable for testing against a known world+map pair (e.g. Nav2's
    # bundled depot/warehouse demo environments) instead of GUARDIAN's own
    # world — useful for isolating "is the localization logic wrong" from
    # "is this particular map too sparse/symmetric to localize in".
    world_arg = LaunchConfiguration('world').perform(context)
    spawn_x   = LaunchConfiguration('spawn_x').perform(context)
    spawn_y   = LaunchConfiguration('spawn_y').perform(context)
    spawn_z   = LaunchConfiguration('spawn_z').perform(context)

    use_sim_time = {'use_sim_time': use_sim}
    rviz_config  = os.path.join(bringup_dir, 'config', 'guardian.rviz')
    lidar_filter_params = os.path.join(
        bringup_dir, 'config', 'lidar_filter_params.yaml')

    # One shared xacro file for sim and real — its <gazebo> blocks
    # (MecanumDrive plugin, gpu_lidar sensors) are only ever acted on by
    # Gazebo; robot_state_publisher and RViz safely ignore them otherwise,
    # so there's no need to conditionally strip them for real hardware.
    # Two near-duplicate files used to exist here; the real-hardware copy
    # silently never got any of the CAD-mesh/dual-LIDAR updates made to
    # the sim copy.
    xacro_file = os.path.join(description_dir, 'urdf', 'guardian.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    actions = [
        # Global use_sim_time for every node launched below (real hardware
        # still needs this explicitly False — no /clock source without sim).
        SetParameter('use_sim_time', use_sim),

        # ── Always: robot description ──────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}, use_sim_time],
        ),
    ]

    # ── use_sim:=true — Gazebo + gz bridge ──────────────────────────────────
    if use_sim:
        gz_sim_dir = get_package_share_directory('ros_gz_sim')
        world_file = (world_arg if world_arg else
                      os.path.join(description_dir, 'worlds', 'guardian_world.sdf'))

        actions += [
            ExecuteProcess(
                cmd=['bash', '-c',
                     'pkill -9 -f "controller_server|planner_server|bt_navigator|'
                     'behavior_server|smoother_server|waypoint_follower|'
                     'velocity_smoother|lifecycle_manager|async_slam_toolbox|'
                     'amcl|map_server" || true'],
                output='screen',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gz_sim_dir, 'launch', 'gz_sim.launch.py')
                ),
                launch_arguments={'gz_args': f'-r {world_file}'}.items(),
            ),
            TimerAction(period=3.0, actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=['-name', 'guardian', '-string', robot_desc,
                               '-x', spawn_x, '-y', spawn_y, '-z', spawn_z],
                    output='screen',
                ),
            ]),
            TimerAction(period=4.0, actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='gz_bridge',
                    arguments=[
                        # GZ-side sensor topic is hardcoded '/scan' in
                        # guardian.urdf.xacro's lidar macro; parameter_bridge's
                        # simple CLI form requires the same name on both sides, so
                        # this stays '/scan' and gets relayed to
                        # '/scan_filtered' below via lidar_republisher_node.
                        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                        '/scan_back@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    ],
                    parameters=[use_sim_time],
                    output='screen',
                ),
            ]),
            TimerAction(period=4.5, actions=[
                # Each raw sensor topic gets frame-fixed (and, later,
                # self-occlusion masked via mask_angle_ranges) on its own
                # intermediate topic before merging — masking must happen
                # per-sensor, in that sensor's own angle_min/max frame,
                # not after re-binning into the merged scan.
                Node(
                    package='guardian_localization',
                    executable='lidar_republisher_node',
                    name='lidar_front_republisher_node',
                    parameters=[{
                        'input_topic':  '/scan',
                        'output_topic': '/scan_front_filtered',
                        'frame_id':     'laser',
                    }, lidar_filter_params, use_sim_time],
                ),
                Node(
                    package='guardian_localization',
                    executable='lidar_republisher_node',
                    name='lidar_back_republisher_node',
                    parameters=[{
                        'input_topic':  '/scan_back',
                        'output_topic': '/scan_back_filtered',
                        'frame_id':     'laser_back',
                    }, lidar_filter_params, use_sim_time],
                ),
            ]),
            TimerAction(period=5.0, actions=[
                # Combines both filtered scans into one virtual 360° scan
                # on /scan_filtered — the topic Nav2/SLAM actually consume.
                # front_x/y/yaw and back_x/y/yaw must match the `lidar`
                # xacro macro instantiations in guardian.urdf.xacro
                # (lidar_front_*/lidar_back_* come from dimensions.xacro,
                # auto-generated off the CAD — update these to match if
                # that ever changes).
                Node(
                    package='guardian_localization',
                    executable='lidar_merger_node',
                    name='lidar_merger_node',
                    parameters=[{
                        'front_input_topic': '/scan_front_filtered',
                        'back_input_topic':  '/scan_back_filtered',
                        'output_topic':      '/scan_filtered',
                        'frame_id':          'base_link',
                        'front_x': 0.319650, 'front_y': 0.0, 'front_yaw': 0.0,
                        'back_x': -0.319650, 'back_y': 0.0,
                        'back_yaw': 3.14159265,
                    }, use_sim_time],
                ),
            ]),
        ]
        nav2_delay = 8.0

    # ── use_sim:=false — real sensors + drive ───────────────────────────────
    else:
        # Defined ahead of the actions list: sweep_scanner_front_node is
        # both launched below and used as the target of the
        # RegisterEventHandler that starts sweep_scanner_back once it's
        # actually ready (see the comment further down).
        sweep_scanner_front_node = Node(
            package='l3xz_sweep_scanner',
            executable='l3xz_sweep_scanner_node',
            name='sweep_scanner_front',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'serial_port': '/dev/lidar_front',
                'topic': 'scan',
                'frame_id': 'laser',
                'rotation_speed': 5,
            }],
        )
        sweep_scanner_back_node = Node(
            package='l3xz_sweep_scanner',
            executable='l3xz_sweep_scanner_node',
            name='sweep_scanner_back',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'serial_port': '/dev/lidar_back',
                'topic': 'scan_back',
                'frame_id': 'laser_back',
                'rotation_speed': 5,
            }],
        )
        _front_lidar_ready = {'done': False}

        def _start_back_lidar_once_front_ready(event):
            # Fires on every stdout line from sweep_scanner_front,
            # including after a respawn=True restart — only actually
            # launch the back unit the first time, never again.
            if _front_lidar_ready['done']:
                return None
            text = event.text.decode(errors='replace')
            if 'starting data acquisition' in text:
                _front_lidar_ready['done'] = True
                return [sweep_scanner_back_node]
            return None

        actions += [
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                parameters=[{'robot_description': robot_desc}],
            ),

            # ── Drive chain ──────────────────────────────────────────────
            Node(
                package='guardian_drive',
                executable='mecanum_kinematics_node',
                name='mecanum_kinematics_node',
                parameters=[robot_params],
            ),
            Node(
                package='guardian_drive',
                executable='serial_bridge_node',
                name='serial_bridge_node',
                parameters=[robot_params, {'sim_mode': False}],
            ),
            # TODO: Phidgets VINT Hub motor driver node (Vedant) — once the
            # DCC1120 driver integration lands, it replaces/augments
            # serial_bridge_node above for real-hardware motor control.

            # ── Scanse Sweep LIDARs (front + back) → merged /scan_filtered ──
            # serial_port uses /dev/lidar_front and /dev/lidar_back, NOT raw
            # /dev/ttyUSB0/1 — those numbers are assigned by USB enumeration
            # order and are not guaranteed to stay attached to the same
            # physical unit across a reboot or replug. /dev/lidar_front and
            # /dev/lidar_back are stable udev symlinks keyed on each USB
            # adapter's own serial number — see
            # 60_scripts/99-guardian-lidar.rules and its README section for
            # how to identify each unit and set this up on the real robot.
            #
            # Topic names ('scan'/'scan_back') deliberately match the sim
            # branch's raw gz-bridge topic names (/scan, /scan_back) —
            # one consistent naming convention for both, feeding the same
            # lidar_republisher_node -> lidar_merger_node pipeline.
            #
            # sweep_scanner_back's startup is chained off sweep_scanner_front
            # actually being ready (RegisterEventHandler below), not a fixed
            # delay: starting both l3xz_sweep_scanner_node processes at the
            # same instant segfaults one or both of them — a USB-serial
            # open/configure race in the underlying libsweep driver. How
            # long _front takes to reach "starting data acquisition" varies
            # (seen 5-13s on the bench), so a fixed timer either wastes time
            # or isn't long enough; waiting for the actual ready line is
            # both faster on average and more reliable. respawn=True on
            # both is a safety net for this driver's occasional crash
            # regardless of startup timing — ros2 launch restarts it rather
            # than leaving the LIDAR pipeline dead until a manual relaunch.
            sweep_scanner_front_node,
            Node(
                package='guardian_localization',
                executable='lidar_republisher_node',
                name='lidar_front_republisher_node',
                parameters=[{
                    'input_topic':  '/scan',
                    'output_topic': '/scan_front_filtered',
                    'frame_id':     'laser',
                }, lidar_filter_params],
            ),
            Node(
                package='guardian_localization',
                executable='lidar_republisher_node',
                name='lidar_back_republisher_node',
                parameters=[{
                    'input_topic':  '/scan_back',
                    'output_topic': '/scan_back_filtered',
                    'frame_id':     'laser_back',
                }, lidar_filter_params],
            ),
            Node(
                package='guardian_localization',
                executable='lidar_merger_node',
                name='lidar_merger_node',
                parameters=[{
                    'front_input_topic': '/scan_front_filtered',
                    'back_input_topic':  '/scan_back_filtered',
                    'output_topic':      '/scan_filtered',
                    'frame_id':          'base_link',
                    'front_x': 0.319650, 'front_y': 0.0, 'front_yaw': 0.0,
                    'back_x': -0.319650, 'back_y': 0.0,
                    'back_yaw': 3.14159265,
                }],
            ),

            # ── Intel RealSense D415 ─────────────────────────────────────
            # Removed: the pinned realsense-ros release (4.54.1, the
            # newest tag at the time) hard-rejects ROS_DISTRO=jazzy in its
            # CMakeLists.txt (Unsupported ROS Distribution error), which
            # broke every full-workspace colcon build. Re-add once a
            # Jazzy-compatible realsense-ros release exists upstream.
        ]
        # See the comment above sweep_scanner_front_node — starts
        # sweep_scanner_back the moment _front actually reports ready,
        # rather than guessing at a fixed delay.
        actions.append(RegisterEventHandler(OnProcessIO(
            target_action=sweep_scanner_front_node,
            on_stdout=_start_back_lidar_once_front_ready,
            on_stderr=_start_back_lidar_once_front_ready,
        )))
        nav2_delay = 0.0

    # ── rviz:=true only — RViz2 (pure visualization, no effect on nav) ──────
    if rviz:
        actions.append(TimerAction(period=nav2_delay, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
                parameters=[use_sim_time],
            ),
        ]))

    # ── Always: Nav2 core (controller=DWB via nav2_params.yaml) ─────────────
    actions.append(TimerAction(period=nav2_delay, actions=[
        Node(package='nav2_controller',   executable='controller_server',
             name='controller_server',   output='screen',
             parameters=[nav2_params, use_sim_time],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_smoother',     executable='smoother_server',
             name='smoother_server',     output='screen',
             parameters=[nav2_params, use_sim_time]),
        Node(package='nav2_planner',      executable='planner_server',
             name='planner_server',      output='screen',
             parameters=[nav2_params, use_sim_time]),
        Node(package='nav2_behaviors',    executable='behavior_server',
             name='behavior_server',     output='screen',
             parameters=[nav2_params, use_sim_time]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator',        output='screen',
             parameters=[nav2_params, use_sim_time]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower',   output='screen',
             parameters=[nav2_params, use_sim_time]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother',   output='screen',
             parameters=[nav2_params, use_sim_time],
             remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[use_sim_time, {
                 'autostart': True,
                 'bond_timeout': 0.0,
                 'node_names': [
                     'controller_server', 'smoother_server',
                     'planner_server', 'behavior_server',
                     'bt_navigator', 'waypoint_follower',
                     'velocity_smoother']}]),
    ]))

    # ── mode:=mapping — SLAM Toolbox live mapping ───────────────────────────
    if mode == 'mapping':
        actions.append(TimerAction(period=nav2_delay + 1.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_dir, 'launch', 'online_async_launch.py')
                ),
                launch_arguments={
                    'slam_params_file': slam_params,
                    'use_sim_time': 'true' if use_sim else 'false',
                }.items(),
            ),
        ]))

    # ── mode:=navigation — AMCL on saved map ────────────────────────────────
    else:
        actions.append(TimerAction(period=nav2_delay + 1.0, actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                # map_server_params.yaml sets yaml_filename under the specific
                # 'map_server:' node name; Node's own parameters= mechanism
                # puts its auto-generated --params-file block *after*
                # arguments=, so a plain arguments=['-p', ...] override still
                # loses (later --ros-args wins). Building one explicit
                # --ros-args block here, with the override last, avoids that.
                arguments=[
                    '--ros-args',
                    '--params-file', map_server_params,
                    '-p', f"use_sim_time:={'true' if use_sim else 'false'}",
                    '-p', f'yaml_filename:={map_yaml}',
                ],
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[amcl_params, use_sim_time],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[use_sim_time, {
                    'autostart': True,
                    'node_names': ['map_server', 'amcl'],
                }],
            ),
        ]))

        # Autonomous localization — no RViz "2D Pose Estimate" needed. Seeds
        # a known pose (sim's deterministic spawn point, or a real
        # deployment that always starts from the same spot) or runs AMCL's
        # global localization + a Spin behavior + covariance polling for an
        # unknown real-world start. Started once map_server/amcl/
        # lifecycle_manager above have had time to activate.
        actions.append(TimerAction(period=nav2_delay + 3.0, actions=[
            Node(
                package='guardian_navigation',
                executable='localization_bootstrap_node',
                name='localization_bootstrap_node',
                output='screen',
                parameters=[use_sim_time, {
                    'known_pose': known_pose,
                    'known_pose_x': start_x,
                    'known_pose_y': start_y,
                    'known_pose_yaw': start_yaw,
                }],
            ),
        ]))

    # ── teleop:=true (or mode:=mapping default) — GUARDIAN's own keyboard
    # teleop node, the same one used on real hardware, so a sim test here
    # exercises the exact code path that runs on the rover. It reads
    # keyboard input directly via evdev (/dev/input/eventN), not terminal
    # raw-mode, so it does NOT need its own TTY — no terminal wrapper here.
    # (An earlier revision wrapped it in `gnome-terminal --wait --`, which
    # looked attached but wasn't: gnome-terminal hands the actual process
    # off to the separate, persistent gnome-terminal-server daemon over
    # D-Bus, so ros2 launch's shutdown signal only reached the thin wrapper
    # it spawned — the real node orphaned and kept running, still holding
    # /dev/input and still publishing to /cmd_vel, stacking up across every
    # session that didn't get killed at the exact same instant as the
    # wrapper. Plain output='screen', same as every other node, avoids all
    # of that and also works headless over SSH.)
    if teleop:
        actions.append(Node(
            package='guardian_teleop',
            executable='keyboard_teleop_node',
            name='keyboard_teleop_node',
            output='screen',
        ))

    return actions


def generate_launch_description():
    bringup_dir = get_package_share_directory('guardian_bringup')
    default_map = os.path.join(bringup_dir, 'maps', 'guardian_map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim', default_value='true',
            description='Use Gazebo simulation (true) or real robot (false)'),
        DeclareLaunchArgument(
            'mode', default_value='navigation',
            description='mapping = SLAM Toolbox | navigation = AMCL on saved map'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Map yaml to load in navigation mode (see save_map.sh)'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz2 (visualization only, no effect on '
                         'navigation) — on by default so mapping/nav goals '
                         'can be set interactively; pass rviz:=false for '
                         'headless/autonomous startup'),
        DeclareLaunchArgument(
            'known_pose', default_value='',
            description="'true'/'false' — whether the robot's start pose "
                         "(start_x/start_y/start_yaw) is known and can be "
                         "seeded directly. Empty (default) auto-selects: "
                         "true in sim (deterministic spawn point), false "
                         "in real (use AMCL global localization instead)."),
        DeclareLaunchArgument(
            'start_x', default_value='0.0',
            description='Known start pose x (m, map frame) — only used '
                         'when known_pose resolves true'),
        DeclareLaunchArgument(
            'start_y', default_value='0.0',
            description='Known start pose y (m, map frame) — only used '
                         'when known_pose resolves true'),
        DeclareLaunchArgument(
            'start_yaw', default_value='0.0',
            description='Known start pose yaw (rad, map frame) — only used '
                         'when known_pose resolves true'),
        DeclareLaunchArgument(
            'world', default_value='',
            description='Absolute path to a Gazebo .sdf world (use_sim '
                         'only). Empty (default) = guardian_world.sdf. '
                         'Override to test against a known world+map pair, '
                         "e.g. Nav2's bundled depot/warehouse demos."),
        DeclareLaunchArgument(
            'spawn_x', default_value='0',
            description='Gazebo spawn x (use_sim only)'),
        DeclareLaunchArgument(
            'spawn_y', default_value='0',
            description='Gazebo spawn y (use_sim only)'),
        DeclareLaunchArgument(
            'spawn_z', default_value='0.15',
            description='Gazebo spawn z (use_sim only)'),
        DeclareLaunchArgument(
            'teleop', default_value='',
            description="'true'/'false' — launch GUARDIAN's own keyboard "
                         'teleop node. Empty (default) auto-selects: on '
                         'for mode:=mapping, off for mode:=navigation.'),
        OpaqueFunction(function=launch_setup),
    ])
