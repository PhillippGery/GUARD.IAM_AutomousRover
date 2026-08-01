# MIT License
# GUARDIAN — StarkHacks 2026
# Launch: guardian_hardware
# Purpose: everything physically attached to the real robot — drive motors,
#          LIDARs, Xbox teleop — as one standalone, self-healing launch.
#
# Run this FIRST and leave it running for the life of a session. It owns
# nothing SLAM/Nav2/AMCL related — that's guardian.launch.py's job, launched
# separately on top of this. The split exists so the software stack (nav2
# params, SLAM tuning, map, etc.) can be freely killed and relaunched while
# testing without ever dropping the LIDAR/motor/teleop connections, and so a
# single crashed hardware node doesn't require restarting the whole stack —
# every node here has respawn=True, so this script is meant to just be
# started once and left alone; if something upstream (USB bus glitch, flaky
# LIDAR connector) kills a node, ros2 launch brings it back on its own
# without you needing to notice or intervene.
#
# Usage: ros2 launch guardian_bringup guardian_hardware.launch.py

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch_ros.actions import Node, SetParameter


def launch_setup(context, *args, **kwargs):
    bringup_dir     = get_package_share_directory('guardian_bringup')
    description_dir = get_package_share_directory('guardian_description')

    robot_params         = os.path.join(bringup_dir, 'config', 'robot_params.yaml')
    lidar_filter_params  = os.path.join(bringup_dir, 'config', 'lidar_filter_params.yaml')
    xbox_teleop_params   = os.path.join(bringup_dir, 'config', 'xbox_teleop.yaml')
    hardware_watchdog    = os.path.join(bringup_dir, 'scripts', 'hardware_watchdog.sh')

    # Same shared xacro file guardian.launch.py uses — one robot_description
    # for every launch file, not a separate real-hardware copy to drift out
    # of sync with the sim/CAD one.
    xacro_file = os.path.join(description_dir, 'urdf', 'guardian.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # Defined ahead of the actions list: sweep_scanner_front_node is both
    # launched below and used as the target of the RegisterEventHandler
    # that starts sweep_scanner_back once it's actually ready.
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
            # Sweep protocol only accepts 500/750/1000 Hz. 1000Hz was tried
            # for denser scans but the serial link is a fixed 115200 baud
            # regardless of this setting and each packet is 7 bytes, so
            # 1000Hz needs ~61% of the link's bandwidth vs 500Hz's ~30% —
            # far less margin for the background reader thread under real
            # CPU load; crash frequency increased noticeably at 1000Hz.
            'sample_rate': 500,
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
            'sample_rate': 500,
        }],
    )
    _front_lidar_ready = {'done': False}

    def _start_back_lidar_once_front_ready(event):
        # Fires on every stdout line from sweep_scanner_front, including
        # after a respawn=True restart — only actually launch the back unit
        # the first time, never again.
        if _front_lidar_ready['done']:
            return None
        text = event.text.decode(errors='replace')
        if 'starting data acquisition' in text:
            _front_lidar_ready['done'] = True
            return [sweep_scanner_back_node]
        return None

    actions = [
        SetParameter('use_sim_time', False),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            respawn=True,
            respawn_delay=2.0,
        ),

        # ── Drive chain ──────────────────────────────────────────────────
        Node(
            package='guardian_drive',
            executable='mecanum_kinematics_node',
            name='mecanum_kinematics_node',
            parameters=[robot_params],
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='guardian_drive',
            executable='phidget_bridge_node',
            name='phidget_bridge_node',
            parameters=[robot_params],
            # libphidget22 can fail to enumerate the VINT hub at startup
            # (confirmed live: real USB bus instability, unrelated to any
            # of our code — raw lsusb sees the device fine but libphidget22
            # still reports "No Phidgets were detected at all") — without
            # respawn the node just exits and never comes back.
            respawn=True,
            respawn_delay=2.0,
        ),

        # ── Scanse Sweep LIDARs (front + back) → merged /scan_filtered ────
        # serial_port uses /dev/lidar_front and /dev/lidar_back, NOT raw
        # /dev/ttyUSB0/1 — those numbers are assigned by USB enumeration
        # order and aren't guaranteed to stay attached to the same physical
        # unit across a reboot or replug. See 60_scripts/99-guardian-lidar.rules.
        #
        # sweep_scanner_back's startup is chained off sweep_scanner_front
        # actually being ready, not a fixed delay: starting both
        # l3xz_sweep_scanner_node processes at the same instant segfaults
        # one or both of them — a USB-serial open/configure race in the
        # underlying libsweep driver.
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
            respawn=True,
            respawn_delay=2.0,
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
            respawn=True,
            respawn_delay=2.0,
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
            respawn=True,
            respawn_delay=2.0,
        ),

        # ── Xbox controller teleop — always on, no flag needed. Reads
        # /dev/input/jsN via joy_node, converts to /cmd_vel via
        # teleop_twist_joy using xbox_teleop.yaml's holonomic axis mapping
        # (left stick = forward/back + strafe, right stick X = rotate,
        # LB = deadman hold). The moment this script is up, the controller
        # can drive the robot — no separate teleop:=true flag to remember.
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'deadzone': 0.15,
                'autorepeat_rate': 20.0,
            }],
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[xbox_teleop_params],
            respawn=True,
            respawn_delay=2.0,
        ),

        # ── Hardware watchdogs — catch a node that's alive but silently
        # producing no data (respawn=True above only catches an actual
        # process exit; confirmed live: sweep_scanner_front can stay
        # running and "healthy" while /scan silently stops flowing — see
        # hardware_watchdog.sh's header for the full root-cause writeup).
        # __node:=X in the pkill pattern (not just "sweep_scanner_front")
        # is deliberately specific so it can never also match a different
        # node — an overly broad kill pattern has bitten this project
        # before. 30s startup grace covers the LIDAR handshake, which has
        # taken 5-13s on the bench; back gets 45s since it only starts
        # once front signals ready, so its own boot begins later.
        ExecuteProcess(
            cmd=['bash', hardware_watchdog, '/scan',
                 '__node:=sweep_scanner_front', '5', '30'],
            output='screen'),
        ExecuteProcess(
            cmd=['bash', hardware_watchdog, '/scan_back',
                 '__node:=sweep_scanner_back', '5', '45'],
            output='screen'),
    ]

    actions.append(RegisterEventHandler(OnProcessIO(
        target_action=sweep_scanner_front_node,
        on_stdout=_start_back_lidar_once_front_ready,
        on_stderr=_start_back_lidar_once_front_ready,
    )))

    return actions


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
