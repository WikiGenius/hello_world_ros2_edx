from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_description_command(package, urdf_file, *args):
    return Command([
        'xacro ',
        PathJoinSubstitution([FindPackageShare(package), 'urdf', urdf_file]),
        ' ',
        *args
    ])


def generate_robot_state_publisher(name, description_command):
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name=name + '_state_publisher',
        parameters=[{'robot_description': description_command}],
        remappings=[('/robot_description', '/' + name + '_description')]
    )


def generate_spawner_node(name, entity_name, topic, x=None, y=None):
    arguments = ['-entity', entity_name, '-topic', topic]
    if x is not None and y is not None:
        arguments.extend(['-x', str(x), '-y', str(y)])
    return Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=arguments
    )


def generate_launch_description():
    # Declare the launch arguments
    args = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in [
            ('workcell', 'workcell_'),
            ('workcell_parent_name', 'world_interface'),
            ('robot1_prefix', 'robot1_'),
            ('robot2_prefix', 'robot2_'),
            ('robot1_pedestal', 'robot1_pedestal_'),
            ('robot2_pedestal', 'robot2_pedestal_'),
            ('robot1_pedestal_height', '0.95'),
            ('robot2_pedestal_height', '0.695'),
            ('vacuum_gripper1_prefix', 'vacuum_gripper1_'),
            ('vacuum_gripper2_prefix', 'vacuum_gripper2_'),
            ('break_beam', 'break_beam_'),
            ('bin_1', 'bin_1_')
        ]
    ]

    # Generate descriptions
    workcell_description = generate_description_command(
        'hrwros_support', 'workcell/workcell.xacro',
        'workcell_parent_name:=', LaunchConfiguration('workcell_parent_name')
    )
    break_beam_description = generate_description_command(
        'hrwros_support', 'break_beam/break_beam.xacro',
        'prefix:=', LaunchConfiguration('break_beam')
    )
    bin_1_description = generate_description_command(
        'hrwros_support', 'bin/bin.xacro',
        'prefix:=', LaunchConfiguration('bin_1')
    )
    robot1_pedestal_description = generate_description_command(
        'hrwros_support', 'robot_pedestal/robot1_pedestal.xacro',
        'pedestal_prefix:=', LaunchConfiguration('robot1_pedestal'), ' ',
        'pedestal_parent:=', LaunchConfiguration('workcell_parent_name'), ' ',
        'pedestal_height:=', LaunchConfiguration('robot1_pedestal_height')
    )
    robot2_pedestal_description = generate_description_command(
        'hrwros_support', 'robot_pedestal/robot2_pedestal.xacro',
        'pedestal_prefix:=', LaunchConfiguration('robot2_pedestal'), ' ',
        'pedestal_parent:=', LaunchConfiguration('workcell_parent_name'), ' ',
        'pedestal_height:=', LaunchConfiguration('robot2_pedestal_height')
    )

    # Path to Gazebo launch file
    gazebo_launch_path = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    ])

    # Generate robot state publishers
    workcell_state_publisher = generate_robot_state_publisher(
        'workcell', workcell_description)
    break_beam_state_publisher = generate_robot_state_publisher(
        'break_beam', break_beam_description)
    bin_1_state_publisher = generate_robot_state_publisher(
        'bin_1', bin_1_description)
    robot1_pedestal_state_publisher = generate_robot_state_publisher(
        'robot1_pedestal', robot1_pedestal_description)
    robot2_pedestal_state_publisher = generate_robot_state_publisher(
        'robot2_pedestal', robot2_pedestal_description)

    # Generate spawner nodes
    break_beam_spawner = generate_spawner_node(
        'break_beam', 'break_beam_', '/break_beam_description')
    bin_1_spawner = generate_spawner_node(
        'bin_1', 'bin_1_', '/bin_1_description', x=0.0, y=0.0)
    robot1_pedestal_spawner = generate_spawner_node(
        'robot1_pedestal', 'robot1_pedestal_', '/robot1_pedestal_description')
    robot2_pedestal_spawner = generate_spawner_node(
        'robot2_pedestal', 'robot2_pedestal_', '/robot2_pedestal_description')
    workcell_spawner = generate_spawner_node(
        'workcell', 'workcell_', '/workcell_description')

    return LaunchDescription(
        args + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch_path)
            ),
            # workcell_state_publisher,
            break_beam_state_publisher,
            # bin_1_state_publisher,
            # robot1_pedestal_state_publisher,
            # robot2_pedestal_state_publisher,
            # TimerAction(
            #     period=10.0,  # Wait 10 second to ensure Gazebo is fully started
            #     actions=[
            #         workcell_spawner
            #     ]
            # ),
            # TimerAction(
            #     period=1.0,  # Wait 20 second to ensure Gazebo is fully started
            #     actions=[
            #         robot1_pedestal_spawner
            #     ]
            # ),
            # TimerAction(
            #     period=30.0,  # Wait 30 second to ensure Gazebo is fully started
            #     actions=[
            #         robot2_pedestal_spawner
            #     ]
            # ),
            # TimerAction(
            #     period=4.0,  # Wait 40 second to ensure Gazebo is fully started
            #     actions=[
            #         bin_1_spawner
            #     ]
            # ),
            TimerAction(
                period=1.0,  # Wait 50 second to ensure Gazebo is fully started
                actions=[
                    break_beam_spawner
                ]
            ),

        ]
    )
