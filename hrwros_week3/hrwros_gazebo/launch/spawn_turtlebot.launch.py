from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def declare_launch_arguments():
    return [
        DeclareLaunchArgument(
            'turtlebot_system',
            default_value='turtlebot3_waffle.urdf',
            description='Turtlebot3 system type'
        )
    ]

def load_xacro_file(xacro_file_path):
    return Command([
        'xacro', ' ',xacro_file_path
    ])

def turtlebot_state_publisher(urdf_content):

    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='turtlebot_state_publisher',
        parameters=[{
            'robot_description': urdf_content
        }],
        remappings=[('robot_description', 'turtlebot_description')],
        output='screen'
    )


def spawn_turtlebot(xacro_file_path):
    return Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', 'mobile_base',
            '-file', xacro_file_path,
            '-x', '-4.0', '-y', '-0.2', '-z', '0.0',
            '-unpause'
        ],
        output='screen'
    )


def generate_launch_description():
    # Construct the file path to the URDF file
    xacro_file_path = PathJoinSubstitution([
        FindPackageShare('turtlebot3_description'),
        'urdf',
        LaunchConfiguration('turtlebot_system')
    ])
    urdf_content = load_xacro_file(xacro_file_path)

    gazebo_launch_path = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    ])

    return LaunchDescription([
        *declare_launch_arguments(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path)
        ),
        turtlebot_state_publisher(urdf_content),
        TimerAction(
            period=1.0,  # Wait 1 seconds to ensure Gazebo is fully started
            actions=[spawn_turtlebot(xacro_file_path)]
        )
    ])


if __name__ == '__main__':
    generate_launch_description()
