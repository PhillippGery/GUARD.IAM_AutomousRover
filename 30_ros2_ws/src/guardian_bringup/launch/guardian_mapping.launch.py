# MIT License
# GUARDIAN — StarkHacks 2026
# Launch: guardian_mapping
# Purpose: drive + lidar + SLAM toolbox + RViz for live mapping

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir     = get_package_share_directory('guardian_bringup')
    description_dir = get_package_share_directory('guardian_description')
    slam_dir        = get_package_share_directory('slam_toolbox')

    robot_params = os.path.join(bringup_dir, 'config', 'robot_params.yaml')
    nav2_params  = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    rviz_config  = os.path.join(bringup_dir, 'config', 'guardian_mapping.rviz')

    robot_desc = xacro.process_file(
        os.path.join(description_dir, 'urdf', 'guardian.urdf.xacro')).toxml()

    return LaunchDescription([

        # ── Robot description ─────────────────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'robot_description': robot_desc}],
        ),

        # ── Drive chain ───────────────────────────────────────────────────────
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
            parameters=[robot_params],
        ),

        # ── Keyboard teleop (own window) ──────────────────────────────────────
        Node(
            package='guardian_teleop',
            executable='keyboard_teleop_node',
            name='keyboard_teleop_node',
            output='screen',
            prefix='xterm -e',
        ),

        # ── Scanse Sweep LIDAR ────────────────────────────────────────────────
        # Reset lidar state before node connects (stops any active data stream)
        ExecuteProcess(
            cmd=['bash', '-c',
                 'stty -F /dev/ttyUSB0 115200 && '
                 'printf "DX\n" > /dev/ttyUSB0 && sleep 1 && '
                 'printf "RR\n" > /dev/ttyUSB0'],
            output='screen',
        ),
        TimerAction(
            period=4.0,
            actions=[Node(
                package='l3xz_sweep_scanner',
                executable='l3xz_sweep_scanner_node',
                name='sweep_scanner',
                parameters=[{
                    'serial_port': '/dev/ttyUSB0',
                    'topic':       '/sweep/scan',
                    'frame_id':    'laser',
                    'rotation_speed': 10,
                    'sample_rate': 1000
                }],
            )],

        ),
        Node(
            package='guardian_localization',
            executable='lidar_republisher_node',
            name='lidar_republisher_node',
            parameters=[{
                'input_topic':  '/sweep/scan',
                'output_topic': '/scan_filtered',
                'frame_id':     'laser',
            }],
        ),

        # ── SLAM Toolbox (online async mapping) ───────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': nav2_params,
                'use_sim_time': 'false',
            }.items(),
        ),

        # ── RViz2 ─────────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
