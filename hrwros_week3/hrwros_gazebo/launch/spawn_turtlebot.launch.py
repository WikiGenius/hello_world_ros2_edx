import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def declare_launch_arguments():
    return [
        DeclareLaunchArgument('turtlebot_system', default_value='turtlebot3'),
        DeclareLaunchArgument('base', default_value=EnvironmentVariable('TURTLEBOT_BASE', default_value='kobuki')),
        DeclareLaunchArgument('stacks', default_value=EnvironmentVariable('TURTLEBOT_STACKS', default_value='hexagons')),
        DeclareLaunchArgument('3d_sensor', default_value=EnvironmentVariable('TURTLEBOT_3D_SENSOR', default_value='kinect'))
    ]

def load_xacro_file():
    return Node(
        package='xacro',
        executable='xacro',
        arguments=[
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_description'),
                'urdf',
                PathJoinSubstitution([
                    LaunchConfiguration('base'),
                    TextSubstitution(text='_'),
                    LaunchConfiguration('stacks'),
                    TextSubstitution(text='_'),
                    LaunchConfiguration('3d_sensor'),
                    TextSubstitution(text='.urdf.xacro')
                ])
            ])
        ],
        output='screen',
        parameters=[{
            'robot_description': PathJoinSubstitution([
                LaunchConfiguration('turtlebot_system'),
                TextSubstitution(text='_description')
            ])
        }]
    )

def spawn_turtlebot():
    return Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=LaunchConfiguration('turtlebot_system'),
        arguments=[
            '-x', '-4.0', '-y', '-0.2', '-z', '0.1',
            '-urdf', '-unpause',
            '-param', PathJoinSubstitution([
                LaunchConfiguration('turtlebot_system'),
                TextSubstitution(text='_description')
            ]),
            '-entity', 'mobile_base'
        ],
        output='screen'
    )

def turtlebot_state_publisher():
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='turtlebot_state_publisher',
        parameters=[{'publish_frequency': 50.0}],
        remappings=[('robot_description', 'robot_description')]
    )

def generate_launch_description():
    return LaunchDescription([
        *declare_launch_arguments(),
        load_xacro_file(),
        spawn_turtlebot(),
        turtlebot_state_publisher()
    ])

if __name__ == '__main__':
    generate_launch_description()
