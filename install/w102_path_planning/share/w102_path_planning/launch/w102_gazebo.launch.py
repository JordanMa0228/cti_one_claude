"""
W102 Gazebo + RViz2 simulation launch file.

Architecture (same pattern as ros2_ws):
  1. Gazebo Sim   — physics server-only (headless, reliable in WSL)
  2. robot_state_publisher — W102 TF tree from URDF
  3. ros_gz_bridge  — /cmd_vel, /odom, /clock, /joint_states
  4. w102_viz_markers — publishes RViz2 marker array (room, chair, waypoints, trail)
  5. RViz2          — 3D visualisation window (WSLg / X11)
  6. w102_gazebo_nav — waypoint P-controller

Usage:
  ros2 launch w102_path_planning w102_gazebo.launch.py
  ros2 launch w102_path_planning w102_gazebo.launch.py rviz:=false   # headless only
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_DISPLAY     = os.environ.get('DISPLAY', ':0')
_WAYLAND     = os.environ.get('WAYLAND_DISPLAY', 'wayland-0')
_XDG_RUNTIME = os.environ.get('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}')

# Gazebo (OGRE1) needs X11/GLX.  Use xcb platform so Qt opens via XWayland,
# which gives OGRE1 a proper GLX context.  D3D12 Mesa driver handles the GPU.
GZ_ENV = {
    'GZ_IP':                          '127.0.0.1',
    'DISPLAY':                        _DISPLAY,
    'QT_QPA_PLATFORM':                'xcb',
    'MESA_D3D12_DEFAULT_ADAPTER_NAME': 'NVIDIA',
    'XDG_RUNTIME_DIR':                _XDG_RUNTIME,
}

# RViz2 / other Qt tools use Wayland natively (more reliable in WSLg)
GUI_ENV = {
    'DISPLAY':          _DISPLAY,
    'WAYLAND_DISPLAY':  _WAYLAND,
    'XDG_RUNTIME_DIR':  _XDG_RUNTIME,
    'QT_QPA_PLATFORM':  'xcb',
}


def generate_launch_description():
    pkg_share  = get_package_share_directory('w102_path_planning')
    world_file = os.path.join(pkg_share, 'worlds',  'living_room.sdf')
    bridge_cfg = os.path.join(pkg_share, 'config',  'ros_gz_bridge.yaml')
    urdf_xacro = os.path.join(pkg_share, 'urdf',    'w102_robot.urdf.xacro')
    rviz_cfg   = os.path.join(pkg_share, 'rviz',    'w102_sim.rviz')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 for 3D visualisation')

    rviz_on = LaunchConfiguration('rviz')

    # ── 1. Gazebo Sim — starts PAUSED so user can orient camera then press Play ─
    # Remove -r so simulation is paused on open.  Press the ▶ Play button in
    # the Gazebo window to start the physics.  The nav node waits for /odom
    # (only published once running) so the robot will not move until Play.
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        additional_env=GZ_ENV,
    )

    # ── 2. robot_state_publisher (URDF → TF, same as ros2_ws pattern) ───────
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_xacro]),
        value_type=str,
    )
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # ── 3. ROS ↔ GZ bridge (4 s delay — let Gazebo finish loading) ──────────
    bridge = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                output='screen',
                parameters=[{'config_file': bridge_cfg}],
                additional_env=GZ_ENV,
            )
        ],
    )

    # ── 4. Visualisation marker publisher (5 s delay — needs bridge up) ─────
    viz_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='w102_path_planning',
                executable='w102_viz_markers',
                name='w102_viz_markers',
                output='screen',
                parameters=[{'use_sim_time': True}],
                additional_env=GZ_ENV,
            )
        ],
    )

    # ── 5. RViz2 (5 s delay — latched markers will load on connect) ─────────
    rviz = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_cfg],
                output='screen',
                parameters=[{'use_sim_time': True}],
                additional_env=GUI_ENV,
                condition=IfCondition(rviz_on),
            )
        ],
    )

    # ── 6. W102 navigation node (7 s delay — bridge must be ready) ──────────
    nav_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='w102_path_planning',
                executable='w102_gazebo_nav',
                name='w102_gazebo_nav',
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': True}],
                additional_env=GZ_ENV,
            )
        ],
    )

    return LaunchDescription([
        rviz_arg,
        gz_server,
        rsp,
        bridge,
        viz_node,
        rviz,
        nav_node,
    ])
