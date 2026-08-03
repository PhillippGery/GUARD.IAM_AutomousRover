# MIT License
# GUARDIAN — StarkHacks 2026
# Launch: guardian
# Purpose: the software stack — Nav2, SLAM/AMCL, RViz — for sim or real
#          hardware. On real hardware (use_sim:=false) this assumes
#          guardian_hardware.launch.py is ALREADY running separately,
#          providing the drive chain, both LIDARs, and Xbox teleop. In sim
#          (use_sim:=true), it's the same split via guardian_sim.launch.py
#          — include_sim_env:=true (default) brings that up here too for
#          one-shot convenience (guardiam_sim), or include_sim_env:=false
#          assumes it's already running separately, e.g. so the mode-switch
#          buttons in the browser control interface can restart just the
#          nav stack without resetting Gazebo/robot/clock underneath it.
#          Either way, this file itself can be killed and relaunched freely
#          while iterating on nav2/SLAM params without ever dropping the
#          environment/hardware connections underneath it.
#
#   use_sim:=true/false   — Gazebo simulation vs real hardware (see above)
#   mode:=mapping/navigation — SLAM Toolbox live mapping vs AMCL on a saved map

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             IncludeLaunchDescription, OpaqueFunction,
                             TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('guardian_bringup')
    slam_dir    = get_package_share_directory('slam_toolbox')

    nav2_params       = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    slam_params       = os.path.join(bringup_dir, 'config', 'slam_params.yaml')
    amcl_params       = os.path.join(bringup_dir, 'config', 'amcl_params.yaml')
    map_server_params = os.path.join(bringup_dir, 'config', 'map_server_params.yaml')

    use_sim  = LaunchConfiguration('use_sim').perform(context).lower() == 'true'
    mode     = LaunchConfiguration('mode').perform(context).lower()
    map_yaml = LaunchConfiguration('map').perform(context)
    rviz     = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    include_sim_env = LaunchConfiguration('include_sim_env').perform(context).lower() == 'true'

    # teleop is sim-only here — on real hardware it's always on, owned
    # entirely by guardian_hardware.launch.py (no flag needed, it's just
    # always running alongside this file), so launching it here too would
    # spawn a second, conflicting joy_node/teleop_twist_joy_node pair.
    # '' (default) auto-selects mode-based (on for mapping, off for
    # navigation) so scripted/automated sim tests aren't affected;
    # teleop:=true/false always overrides.
    teleop_arg = LaunchConfiguration('teleop').perform(context)
    teleop = use_sim and (
        (teleop_arg.lower() == 'true') if teleop_arg else (mode == 'mapping'))

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
    xbox_teleop_params = os.path.join(
        bringup_dir, 'config', 'xbox_teleop.yaml')
    activate_lifecycle_node = os.path.join(
        bringup_dir, 'scripts', 'activate_lifecycle_node.sh')

    actions = [
        # Global use_sim_time for every node launched below (real hardware
        # still needs this explicitly False — no /clock source without sim).
        SetParameter('use_sim_time', use_sim),
    ]

    # ── use_sim:=true — Gazebo + gz bridge ──────────────────────────────────
    if use_sim:
        # Nav2's own stale processes need clearing regardless of whether
        # the sim environment itself is included here or running
        # separately — this is about killing leftover Nav2 core from a
        # previous run, not about the sim environment.
        actions.append(ExecuteProcess(
            cmd=['bash', '-c',
                 'pkill -9 -f "controller_server|planner_server|bt_navigator|'
                 'behavior_server|smoother_server|waypoint_follower|'
                 'velocity_smoother|lifecycle_manager|async_slam_toolbox|'
                 'amcl|map_server" || true'],
            output='screen',
        ))

        if include_sim_env:
            # Self-contained one-shot convenience path (guardiam_sim /
            # guardiam_sim_foxglove) — no separately-running environment to
            # assume, so bring up Gazebo/robot/bridge/lidar-merge here.
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'guardian_sim.launch.py')
                ),
                launch_arguments={
                    'world': world_arg, 'spawn_x': spawn_x,
                    'spawn_y': spawn_y, 'spawn_z': spawn_z,
                }.items(),
            ))
            nav2_delay = 8.0
        else:
            # include_sim_env:=false — guardian_sim.launch.py is assumed to
            # already be running separately (started once, left running
            # across mode switches). This is what the mode-switch buttons
            # use: restarting the whole sim environment on every mode
            # switch reset /clock back near zero and respawned the robot
            # at the origin each time, which made Foxglove's (and RViz's)
            # TF buffer treat the "new" but earlier-than-previously-seen
            # transforms as stale and drop them — the rendered robot pose
            # visibly froze and the URDF mesh stopped updating even though
            # data was still flowing underneath. Keeping the environment
            # running avoids the discontinuity entirely.
            #
            # Not 0.0: the pkill ExecuteProcess above and this delay's
            # Nav2 TimerAction are both just appended to the same launch
            # actions list, which ros2 launch schedules concurrently, not
            # sequentially — a period of 0.0 gave the pkill zero
            # guaranteed head start. Confirmed happening live: pkill's
            # process-name-pattern scan (controller_server|planner_server
            # |...) sometimes still caught the *freshly spawned*
            # controller_server/planner_server/smoother_server from this
            # exact launch and killed them (exit code -9) within the same
            # second they started, which then left
            # lifecycle_manager_navigation stuck forever on "Waiting for
            # service controller_server/get_state..." — Nav2 silently up
            # but never actually able to drive the robot. Worse under
            # memory pressure (a loaded pkill/proc scan takes longer),
            # which is exactly the condition this system was under when
            # it was caught. 1.5s is comfortably more than pkill -9 -f
            # ever needs even under load; still near-instant for a mode
            # switch compared to the 8.0s full-sim-env path above.
            nav2_delay = 1.5

    # ── use_sim:=false — real hardware ───────────────────────────────────────
    # Nothing physical is launched here anymore — robot_state_publisher,
    # joint_state_publisher, the drive chain (mecanum_kinematics_node,
    # phidget_bridge_node), both Sweep LIDARs, the lidar_republisher/merger
    # chain, and Xbox teleop all moved to guardian_hardware.launch.py, a
    # separate, standalone, self-healing launch file meant to be started
    # once and left running for the session (every node in it has
    # respawn=True). Run it first:
    #     ros2 launch guardian_bringup guardian_hardware.launch.py
    # This file then only owns the software stack on top (Nav2, SLAM/AMCL,
    # RViz) — it can be killed and relaunched freely while iterating on nav2
    # params or SLAM tuning without ever dropping the LIDAR/motor/teleop
    # connections that guardian_hardware.launch.py maintains independently.
    else:
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
        # Re-publishes NavigateToPose's action feedback (distance_remaining,
        # estimated_time_remaining — Nav2 already computes both) as a plain
        # /guardian/nav_status String, since action feedback topics are
        # hidden by default and awkward to add as a Foxglove panel
        # directly. Depends on bt_navigator's action server existing, so
        # it belongs in this same TimerAction alongside it.
        Node(package='guardian_navigation', executable='nav_status_node',
             name='nav_status_node', parameters=[use_sim_time]),
        # Persistent listener mode (no 'index' param passed) — see the
        # node's own docstring. Lets the browser control interface record
        # waypoints the same way `set_waypoint <index>` already does from
        # a terminal, without needing a fresh `ros2 run` per waypoint.
        Node(package='guardian_navigation', executable='set_waypoint_node',
             name='set_waypoint_node', parameters=[use_sim_time]),
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
                    # online_async_launch.py's own default (autostart=true,
                    # use_lifecycle_manager=false) self-activates via an
                    # internal configure->OnStateTransition->activate event
                    # chain, which doesn't fire reliably once nested inside
                    # our own TimerAction/IncludeLaunchDescription wrapper —
                    # confirmed empirically: `ros2 lifecycle get
                    # /slam_toolbox` sits at "inactive" forever. A
                    # nav2_lifecycle_manager instance aimed at it also
                    # proved unreliable (its first Configure call races
                    # slam_toolbox's own startup). A single fixed-delay
                    # `ros2 lifecycle set` pair was tried next but ALSO
                    # proved unreliable on real hardware — LIDAR driver
                    # crash/respawn cycles routinely add 10-30+
                    # unpredictable seconds to startup, so any fixed delay
                    # can fire before the node even exists yet and just
                    # fail outright with no retry (confirmed: `ros2
                    # lifecycle set /slam_toolbox activate` died with exit
                    # code 1 on a run where LIDAR startup ran long).
                    # activate_lifecycle_node.sh retries every 1s until it
                    # actually succeeds, so it's correct regardless of how
                    # long LIDAR startup actually takes.
                    'use_lifecycle_manager': 'false',
                    'autostart': 'false',
                }.items(),
            ),
            ExecuteProcess(
                cmd=['bash', activate_lifecycle_node, 'slam_toolbox', '90'],
                output='screen'),
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
                # Confirmed via an actual GDB backtrace on a captured core
                # dump: nav2_amcl can segfault inside its own particle-filter
                # library during startup —
                # nav2_amcl::AmclNode::uniformPoseGenerator (libamcl_core.so)
                # called from pf_init_model (libpf_lib.so), while spreading
                # the initial particle cloud across the map. This is inside
                # the prebuilt ros-jazzy-nav2-amcl package, not our code, and
                # didn't reproduce on an immediate standalone retry — a rare,
                # intermittent crash in third-party code we can't patch
                # directly. respawn mirrors the same mitigation already used
                # for the flaky LIDAR driver.
                respawn=True,
                respawn_delay=2.0,
            ),
            # nav2_lifecycle_manager instances added in this launch file
            # (as opposed to the pre-existing lifecycle_manager_navigation
            # further up, which has always worked reliably) have shown a
            # repeatable bug: they issue Configure to every managed node
            # successfully, then stall forever before ever issuing
            # Activate. A fixed-delay `ros2 lifecycle set` pair isn't
            # reliable either — real-hardware startup timing varies too
            # much (LIDAR driver crash/respawn cycles) for any guessed
            # delay to consistently land after the node actually exists.
            # activate_lifecycle_node.sh retries until each transition
            # actually succeeds instead of gambling on a fixed wait.
            ExecuteProcess(
                cmd=['bash', activate_lifecycle_node, 'map_server', '90'],
                output='screen'),
            ExecuteProcess(
                cmd=['bash', activate_lifecycle_node, 'amcl', '90'],
                output='screen'),
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

    # ── teleop:=true (or mode:=mapping default) — Xbox controller teleop,
    # using the standard ROS2 joy + teleop_twist_joy packages rather than
    # GUARDIAN's own keyboard_teleop_node. joy_node reads the raw
    # /dev/input/jsN device and publishes sensor_msgs/Joy; teleop_twist_joy
    # converts that into geometry_msgs/Twist on /cmd_vel using
    # xbox_teleop.yaml's holonomic axis mapping (left stick =
    # forward/back + strafe, right stick X = rotate, LB = deadman hold,
    # RB = turbo) — same /cmd_vel contract as the old keyboard node, so
    # nothing downstream (mecanum_kinematics_node onward) changes.
    if teleop:
        actions += [
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                output='screen',
                parameters=[{
                    'deadzone': 0.15,
                    'autorepeat_rate': 20.0,
                }],
            ),
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_twist_joy_node',
                output='screen',
                parameters=[xbox_teleop_params],
            ),
        ]

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
            'include_sim_env', default_value='true',
            description='use_sim only. true (default): bring up Gazebo/'
                         'robot/bridge/lidar-merge here too (one-shot '
                         'convenience, e.g. guardiam_sim). false: assume '
                         'guardian_sim.launch.py is already running '
                         'separately — used when switching Nav2 mode '
                         'on demand, so the sim environment (and /clock) '
                         "doesn't restart on every switch."),
        DeclareLaunchArgument(
            'teleop', default_value='',
            description="'true'/'false' — launch Xbox controller teleop "
                         '(joy + teleop_twist_joy). Empty (default) '
                         'auto-selects: always on for real hardware '
                         '(any mode), mode-based for sim (on for mapping, '
                         'off for navigation).'),
        OpaqueFunction(function=launch_setup),
    ])
