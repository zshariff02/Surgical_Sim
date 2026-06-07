"""
surgical_sim.launch.py
======================
Launches the full surgical instrument positioning simulation:
  - Gazebo with surgical_table.world
  - UR5e robot_state_publisher + joint_state_publisher
  - MoveIt2 move_group
  - SurgicalController node
  - MetricsLogger node

Usage:
    ros2 launch surgical_sim surgical_sim.launch.py
    ros2 launch surgical_sim surgical_sim.launch.py gui:=false
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_surgical = FindPackageShare('surgical_sim')
    pkg_ur       = FindPackageShare('ur_moveit_config')

    # --- Arguments ---
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch Gazebo with GUI'
    )
    ur_type_arg = DeclareLaunchArgument(
        'ur_type', default_value='ur5e',
        description='UR robot type (ur3, ur5, ur5e, ur10, ur10e, ur16e)'
    )

    gui     = LaunchConfiguration('gui')
    ur_type = LaunchConfiguration('ur_type')

    # --- Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([pkg_surgical, 'worlds', 'surgical_table.world']),
            'gui':   gui,
        }.items(),
    )

    # --- UR5e + MoveIt2 (delayed so Gazebo has time to start) ---
    ur_moveit = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([pkg_ur, 'launch', 'ur_moveit.launch.py'])
                ]),
                launch_arguments={
                    'ur_type':       ur_type,
                    'use_sim_time':  'true',
                    'launch_rviz':   'false',
                }.items(),
            )
        ],
    )

    # --- Surgical controller (delayed until MoveIt2 is up) ---
    controller = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='surgical_sim',
                executable='surgical_controller',
                name='surgical_controller',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ],
    )

    # --- Metrics logger ---
    logger = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='surgical_sim',
                executable='metrics_logger',
                name='metrics_logger',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ],
    )

    return LaunchDescription([
        gui_arg,
        ur_type_arg,
        gazebo,
        ur_moveit,
        controller,
        logger,
    ])
