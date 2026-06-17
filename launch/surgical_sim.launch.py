"""
surgical_sim.launch.py
======================
Launches the full surgical instrument positioning simulation:
  - Gazebo with surgical_table.world
  - UR5e robot_state_publisher
  - UR5e spawned into Gazebo
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
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_surgical = FindPackageShare('surgical_sim')
    pkg_ur       = FindPackageShare('ur_moveit_config')
    pkg_ur_desc  = FindPackageShare('ur_description')

    # --- Arguments ---
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch Gazebo with GUI'
    )
    ur_type_arg = DeclareLaunchArgument(
        'ur_type', default_value='ur5e',
        description='UR robot type'
    )

    gui     = LaunchConfiguration('gui')
    ur_type = LaunchConfiguration('ur_type')

    # --- Robot description ---
    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([pkg_ur_desc, 'urdf', 'ur.urdf.xacro']),
            ' ur_type:=', ur_type,
            ' name:=ur',
        ]),
        value_type=str
    )

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

    # --- Robot state publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # --- Spawn UR5e into Gazebo (delayed so Gazebo is ready) ---
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'ur5e',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.0',
                ],
                output='screen',
            )
        ],
    )

    # --- UR5e + MoveIt2 (delayed so Gazebo and robot are ready) ---
    ur_moveit = TimerAction(
        period=8.0,
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

    # --- Surgical controller ---
    controller = TimerAction(
        period=15.0,
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
        period=15.0,
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
        robot_state_publisher,
        spawn_robot,
        ur_moveit,
        controller,
        logger,
    ])

