from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_surgical = FindPackageShare('surgical_sim')
    pkg_ur_desc = FindPackageShare('ur_description')

    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    ur_type_arg = DeclareLaunchArgument('ur_type', default_value='ur5e')
    gui = LaunchConfiguration('gui')
    ur_type = LaunchConfiguration('ur_type')

    sim_controllers = PathJoinSubstitution([pkg_surgical, 'config', 'ur_sim_controllers.yaml'])

    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([pkg_ur_desc, 'urdf', 'ur.urdf.xacro']),
            ' ur_type:=', ur_type,
            ' name:=ur',
            ' sim_gazebo:=true',
            ' simulation_controllers:=', sim_controllers,
        ]),
        value_type=str
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([pkg_surgical, 'worlds', 'surgical_table.world']),
            'gui': gui,
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    spawn_robot = TimerAction(period=8.0, actions=[
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'ur5e',
                       '-x', '0.0', '-y', '0.4', '-z', '0.9'],
            output='screen',
        )
    ])

    return LaunchDescription([
        gui_arg,
        ur_type_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])
