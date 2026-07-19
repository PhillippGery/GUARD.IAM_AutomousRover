# MIT License
# GUARDIAN — StarkHacks 2026
# Launch: guardian_sim
# Purpose: Gazebo Harmonic simulation with SLAM + Nav2 autonomous navigation

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir     = get_package_share_directory('guardian_bringup')
    description_dir = get_package_share_directory('guardian_description')
    slam_dir        = get_package_share_directory('slam_toolbox')
    gz_sim_dir      = get_package_share_directory('ros_gz_sim')

    world_file   = os.path.join(description_dir, 'worlds', 'guardian_world.sdf')
    nav2_params  = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    rviz_config  = os.path.join(bringup_dir, 'config', 'guardian_nav.rviz')
    robot_desc = xacro.process_file(
        os.path.join(description_dir, 'urdf', 'guardian_sim.urdf.xacro')).toxml()

    return LaunchDescription([

        # ── 0. Kill stale nav2/gz processes from previous runs ────────────────
        ExecuteProcess(
            cmd=['bash', '-c',
                 'pkill -9 -f "controller_server|planner_server|bt_navigator|'
                 'behavior_server|smoother_server|waypoint_follower|'
                 'velocity_smoother|lifecycle_manager" || true'],
            output='screen',
        ),

        # ── 1. Gazebo Harmonic ────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_sim_dir, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        ),

        # ── 2. Robot state publisher ──────────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        ),

        # ── 3. Spawn robot in Gazebo ──────────────────────────────────────────
        TimerAction(period=3.0, actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-name', 'guardian', '-string', robot_desc,
                           '-x', '0', '-y', '0', '-z', '0.15'],
                output='screen',
            ),
        ]),

        # ── 4. ROS-GZ bridge ──────────────────────────────────────────────────
        # GZ-side sensor topic is hardcoded '/scan' in guardian_sim.urdf.xacro;
        # relayed to '/scan_filtered' below since nav2_params.yaml's costmaps
        # and slam_toolbox now read from '/scan_filtered'.
        TimerAction(period=4.0, actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='gz_bridge',
                arguments=[
                    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                    '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                ],
                parameters=[{'use_sim_time': True}],
                output='screen',
            ),
        ]),
        TimerAction(period=4.5, actions=[
            Node(
                package='guardian_localization',
                executable='lidar_republisher_node',
                name='lidar_republisher_node',
                parameters=[{
                    'input_topic':  '/scan',
                    'output_topic': '/scan_filtered',
                    'frame_id':     'laser',
                }, {'use_sim_time': True}],
            ),
        ]),

        # ── 5. SLAM Toolbox (live mapping) ────────────────────────────────────
        TimerAction(period=5.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_dir, 'launch', 'online_async_launch.py')
                ),
                launch_arguments={
                    'slam_params_file': nav2_params,
                    'use_sim_time': 'true',
                }.items(),
            ),
        ]),

        # ── 6. Nav2 — launch only the essential nodes directly ────────────────
        TimerAction(period=10.0, actions=[
            Node(package='nav2_controller',   executable='controller_server',
                 name='controller_server',   output='screen',
                 parameters=[nav2_params, {'use_sim_time': True}],
                 remappings=[('cmd_vel', 'cmd_vel_nav')]),
            Node(package='nav2_smoother',     executable='smoother_server',
                 name='smoother_server',     output='screen',
                 parameters=[nav2_params, {'use_sim_time': True}]),
            Node(package='nav2_planner',      executable='planner_server',
                 name='planner_server',      output='screen',
                 parameters=[nav2_params, {'use_sim_time': True}]),
            Node(package='nav2_behaviors',    executable='behavior_server',
                 name='behavior_server',     output='screen',
                 parameters=[nav2_params, {'use_sim_time': True}]),
            Node(package='nav2_bt_navigator', executable='bt_navigator',
                 name='bt_navigator',        output='screen',
                 parameters=[nav2_params, {'use_sim_time': True}]),
            Node(package='nav2_waypoint_follower', executable='waypoint_follower',
                 name='waypoint_follower',   output='screen',
                 parameters=[nav2_params, {'use_sim_time': True}]),
            Node(package='nav2_velocity_smoother', executable='velocity_smoother',
                 name='velocity_smoother',   output='screen',
                 parameters=[nav2_params, {'use_sim_time': True}],
                 remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', output='screen',
                 parameters=[{'use_sim_time': True,
                               'autostart': True,
                               'bond_timeout': 0.0,
                               'node_names': [
                                   'controller_server', 'smoother_server',
                                   'planner_server', 'behavior_server',
                                   'bt_navigator', 'waypoint_follower',
                                   'velocity_smoother']}]),
        ]),

        # ── 7. RViz2 with Nav2 tools ──────────────────────────────────────────
        TimerAction(period=10.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
            ),
        ]),

        # ── 8. Keyboard teleop (for map building phase) ───────────────────────
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            prefix='xterm -e',
        ),

    ])
