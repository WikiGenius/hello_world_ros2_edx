from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from my_python_utils import some_utility


def generate_launch_description():

    # Path to Gazebo launch file
    # gazebo_launch_path = PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
    # gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch_path))

    # Declare the launch arguments
    args = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in [
            ('workcell_prefix', 'workcell_'),
            ('workcell_parent_name', 'world'),
            ('robot1_pedestal_prefix', 'robot1_pedestal_'),
            ('robot2_pedestal_prefix', 'robot2_pedestal_'),
            ('robot1_pedestal_height', '0.95'),
            ('robot2_pedestal_height', '0.695'),
            ('break_beam_prefix', 'break_beam_'),
            ('bin_1_prefix', 'bin_1_'),
            ('logical_camera1_prefix', 'logical_camera1_'),
            ('logical_camera2_prefix', 'logical_camera2_'),
        ]
    ]

    # Generate descriptions
    workcell_description = some_utility.generate_description_command(
        'hrwros_support', 'workcell/workcell.xacro',
        'workcell_parent_name:=', LaunchConfiguration('workcell_parent_name')
    )
    break_beam_description = some_utility.generate_description_command(
        'hrwros_support', 'break_beam/break_beam.xacro',
        'prefix:=', LaunchConfiguration('break_beam_prefix'), ' ',
        'break_beam_parent:=', LaunchConfiguration('workcell_parent_name')
    )
    bin_1_description = some_utility.generate_description_command(
        'hrwros_support', 'bin/bin.xacro',
        'prefix:=', LaunchConfiguration('bin_1_prefix'), ' ',
        'bin_parent:=', LaunchConfiguration('workcell_parent_name')
    )
    logical_camera1_description = some_utility.generate_description_command(
        'hrwros_support', 'logical_camera/logical_camera1.xacro',
        'prefix:=', LaunchConfiguration('logical_camera1_prefix'), ' ',
        'logical_camera_parent:=', LaunchConfiguration('workcell_parent_name')
    )
    logical_camera2_description = some_utility.generate_description_command(
        'hrwros_support', 'logical_camera/logical_camera2.xacro',
        'prefix:=', LaunchConfiguration('logical_camera2_prefix'), ' ',
        'logical_camera_parent:=', LaunchConfiguration('workcell_parent_name')
    )
    robot1_pedestal_description = some_utility.generate_description_command(
        'hrwros_support', 'robot_pedestal/robot1_pedestal.xacro',
        'pedestal_prefix:=', LaunchConfiguration(
            'robot1_pedestal_prefix'), ' ',
        'pedestal_parent:=', LaunchConfiguration('workcell_parent_name'), ' ',
        'pedestal_height:=', LaunchConfiguration('robot1_pedestal_height')
    )
    robot2_pedestal_description = some_utility.generate_description_command(
        'hrwros_support', 'robot_pedestal/robot2_pedestal.xacro',
        'pedestal_prefix:=', LaunchConfiguration(
            'robot2_pedestal_prefix'), ' ',
        'pedestal_parent:=', LaunchConfiguration('workcell_parent_name'), ' ',
        'pedestal_height:=', LaunchConfiguration('robot2_pedestal_height')
    )

    # Generate robot state publishers
    workcell_state_publisher = some_utility.generate_robot_state_publisher(
        'workcell', workcell_description)
    break_beam_state_publisher = some_utility.generate_robot_state_publisher(
        'break_beam', break_beam_description)
    bin_1_state_publisher = some_utility.generate_robot_state_publisher(
        'bin_1', bin_1_description)
    logical_camera1_state_publisher = some_utility.generate_robot_state_publisher(
        'logical_camera1', logical_camera1_description)
    logical_camera2_state_publisher = some_utility.generate_robot_state_publisher(
        'logical_camera2', logical_camera2_description)
    robot1_pedestal_state_publisher = some_utility.generate_robot_state_publisher(
        'robot1_pedestal', robot1_pedestal_description)
    robot2_pedestal_state_publisher = some_utility.generate_robot_state_publisher(
        'robot2_pedestal', robot2_pedestal_description)

    # Generate spawner nodes
    break_beam_spawner = some_utility.generate_spawner_node(
        'break_beam', '/break_beam_description')
    bin_1_spawner = some_utility.generate_spawner_node(
        'bin_1', '/bin_1_description', x=-8.0, y=-2.2)
    logical_camera1_spawner = some_utility.generate_spawner_node(
        'logical_camera1', '/logical_camera1_description')
    logical_camera2_spawner = some_utility.generate_spawner_node(
        'logical_camera2', '/logical_camera2_description')
    robot1_pedestal_spawner = some_utility.generate_spawner_node(
        'robot1_pedestal', '/robot1_pedestal_description')
    robot2_pedestal_spawner = some_utility.generate_spawner_node(
        'robot2_pedestal', '/robot2_pedestal_description')
    workcell_spawner = some_utility.generate_spawner_node(
        'workcell', '/workcell_description')
    state_publisher_list = [
        workcell_state_publisher,
        robot1_pedestal_state_publisher,
        robot2_pedestal_state_publisher,
        bin_1_state_publisher,
        break_beam_state_publisher,
        logical_camera1_state_publisher,
        logical_camera2_state_publisher
    ]
    spawn_list = [
        workcell_spawner,
        robot1_pedestal_spawner,
        robot2_pedestal_spawner,
        bin_1_spawner,
        break_beam_spawner,
        logical_camera1_spawner,
        logical_camera2_spawner
    ]
    return LaunchDescription(
        args + [
            *state_publisher_list,
            *spawn_list
        ]
    )
