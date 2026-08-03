# MIT License
# GUARDIAN — StarkHacks 2026
# Launch: guardian_sim
# Purpose: the sim-only equivalent of guardian_hardware.launch.py — Gazebo,
#   the spawned robot, the gz bridge, and the dual-LIDAR merge pipeline.
#   No Nav2/SLAM/AMCL here — see guardian.launch.py for that.
#
#   Split out of guardian.launch.py's sim branch specifically so the sim
#   environment can stay running continuously across Nav2 mode switches.
#   Before this split, switching mode:=mapping/navigation restarted the
#   ENTIRE guardian.launch.py — including this Gazebo/spawn/bridge block —
#   which reset /clock back near zero and respawned the robot at the
#   origin every time. Foxglove (and RViz) both get confused when a live
#   connection's timestamps jump backward like that: TF2's buffer treats
#   the "new" (but earlier-than-previously-seen) transforms as stale and
#   drops them, so the rendered robot pose visibly freezes and the URDF
#   mesh stops updating even though data is still flowing underneath.
#   Keeping this process running across mode switches avoids the
#   discontinuity entirely.
#
#   guardian.launch.py's own sim branch still includes this file by
#   default (via include_sim_env:=true) for one-shot convenience — e.g.
#   guardiam_sim / guardiam_sim_foxglove, where there's no separate
#   "already running" environment to assume. Pass include_sim_env:=false
#   there (as the sim-test systemd unit does) when this is started
#   separately and left running, same relationship guardian_hardware
#   .launch.py has with guardian.launch.py on the real robot.
#
#   Usage: ros2 launch guardian_bringup guardian_sim.launch.py

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def launch_setup(context, *args, **kwargs):
    bringup_dir     = get_package_share_directory('guardian_bringup')
    description_dir = get_package_share_directory('guardian_description')

    world_arg = LaunchConfiguration('world').perform(context)
    spawn_x   = LaunchConfiguration('spawn_x').perform(context)
    spawn_y   = LaunchConfiguration('spawn_y').perform(context)
    spawn_z   = LaunchConfiguration('spawn_z').perform(context)

    lidar_filter_params = os.path.join(
        bringup_dir, 'config', 'lidar_filter_params.yaml')

    # Same shared xacro file guardian.launch.py uses in its real branch —
    # its <gazebo> blocks (MecanumDrive plugin, gpu_lidar sensors) are
    # only ever acted on by Gazebo; robot_state_publisher safely ignores
    # them otherwise, so there's no need for a separate real-hardware copy.
    xacro_file = os.path.join(description_dir, 'urdf', 'guardian.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    use_sim_time = {'use_sim_time': True}
    gz_sim_dir = get_package_share_directory('ros_gz_sim')
    world_file = (world_arg if world_arg else
                  os.path.join(description_dir, 'worlds', 'guardian_world.sdf'))

    return [
        SetParameter('use_sim_time', True),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}, use_sim_time],
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
                    # simple CLI form requires the same name on both sides,
                    # so this stays '/scan' and gets relayed to
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
            # per-sensor, in that sensor's own angle_min/max frame, not
            # after re-binning into the merged scan.
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
            # Combines both filtered scans into one virtual 360° scan on
            # /scan_filtered — the topic Nav2/SLAM actually consume.
            # front_x/y/yaw and back_x/y/yaw must match the `lidar` xacro
            # macro instantiations in guardian.urdf.xacro
            # (lidar_front_*/lidar_back_* come from dimensions.xacro,
            # auto-generated off the CAD — update these to match if that
            # ever changes).
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


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='',
            description='Absolute path to a Gazebo .sdf world. Empty '
                         '(default) = guardian_world.sdf.'),
        DeclareLaunchArgument(
            'spawn_x', default_value='0',
            description='Gazebo spawn x'),
        DeclareLaunchArgument(
            'spawn_y', default_value='0',
            description='Gazebo spawn y'),
        DeclareLaunchArgument(
            'spawn_z', default_value='0.15',
            description='Gazebo spawn z'),
        OpaqueFunction(function=launch_setup),
    ])
