from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_arguments():
    return [
        DeclareLaunchArgument(
            'map_file',
            default_value='',
            description='Full path to the map file'
        ),
        DeclareLaunchArgument(
            '3d_sensor',
            default_value=EnvironmentVariable('TURTLEBOT_3D_SENSOR'),
            description='Type of 3D sensor (r200, kinect, asus_xtion_pro)'
        ),
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='0.0',
            description='Initial x position of the robot'
        ),
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='0.0',
            description='Initial y position of the robot'
        ),
        DeclareLaunchArgument(
            'initial_pose_a',
            default_value='0.0',
            description='Initial orientation angle of the robot'
        ),
        DeclareLaunchArgument(
            'custom_amcl_launch_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('turtlebot_navigation'),
                'launch',
                'includes',
                'amcl',
                LaunchConfiguration('3d_sensor') + '_amcl.launch.py'
            ]),
            description='Path to the custom AMCL launch file'
        ),
        DeclareLaunchArgument(
            'move_base_launch_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('turtlebot_navigation'),
                'launch',
                'includes',
                'move_base.launch.py'
            ]),
            description='Path to the move_base launch file'
        )
    ]


def generate_map_server_node():
    return Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map_file')}]
    )


def generate_amcl_launch_include():
    return IncludeLaunchDescription(
        # Specifies the source of the included launch file
        PythonLaunchDescriptionSource(
            # The path to the launch file is set dynamically via a launch configuration
            LaunchConfiguration('custom_amcl_launch_file')
        ),
        # Launch arguments to pass to the included launch file
        launch_arguments={
            # Initial pose x-coordinate
            'initial_pose_x': LaunchConfiguration('initial_pose_x'),
            # Initial pose y-coordinate
            'initial_pose_y': LaunchConfiguration('initial_pose_y'),
            # Initial pose orientation angle
            'initial_pose_a': LaunchConfiguration('initial_pose_a')
        }.items()  # Converts the dictionary to a list of key-value pairs
    )


def generate_move_base_launch_include():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            LaunchConfiguration('move_base_launch_file')
        ])
    )


def generate_launch_description():
    launch_args = generate_launch_arguments()
    map_server_node = generate_map_server_node()
    amcl_launch_include = generate_amcl_launch_include()
    move_base_launch_include = generate_move_base_launch_include()

    return LaunchDescription(launch_args + [map_server_node, amcl_launch_include, move_base_launch_include])
