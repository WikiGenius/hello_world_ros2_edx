import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def load_xacro_file(xacro_file_path):
    """
    Load the xacro file and return the command to process it.
    """
    return Command(['xacro', ' ', xacro_file_path])


def turtlebot_state_publisher(urdf_content):
    """
    Create a node to publish the state of the TurtleBot3.
    """
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='turtlebot_state_publisher',
        parameters=[{'robot_description': urdf_content}],
        remappings=[('robot_description', 'turtlebot_description')],
        output='screen'
    )


def spawn_turtlebot():
    """
    Create a node to spawn the TurtleBot3 in Gazebo.
    """
    return Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', 'mobile_base',
            '-topic', 'turtlebot_description',
            '-x', '-4.0', '-y', '-0.2', '-z', '0.0',
            '-unpause'
        ],
        output='screen'
    )


def generate_launch_description():
    """
    Generate the launch description for launching the TurtleBot3.
    """
    # Ensure the environment variable is set
    turtlebot_model = os.getenv('TURTLEBOT3_MODEL')
    if not turtlebot_model:
        raise EnvironmentError(
            "TURTLEBOT3_MODEL environment variable is not set. Please export TURTLEBOT3_MODEL in your bashrc.")

    # Construct the file path to the URDF file
    xacro_file_path = PathJoinSubstitution([
        FindPackageShare('turtlebot3_description'),
        'urdf',
        f'turtlebot3_{turtlebot_model}.urdf'
    ])
    urdf_content = load_xacro_file(xacro_file_path)

    # Path to Gazebo launch file
    gazebo_launch_path = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    ])

    # Generate launch description
    return LaunchDescription([
        turtlebot_state_publisher(urdf_content),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path)
        ),
        TimerAction(
            period=1.0,  # Wait 1 second to ensure Gazebo is fully started
            actions=[spawn_turtlebot()]
        )
    ])


if __name__ == '__main__':
    generate_launch_description()
