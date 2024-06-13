import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare

def declare_launch_arguments():
    return [
        DeclareLaunchArgument('turtlebot_system', default_value='turtlebot_'),
        DeclareLaunchArgument('base', default_value=EnvironmentVariable('TURTLEBOT_BASE', default_value='kobuki')),
        DeclareLaunchArgument('stacks', default_value=EnvironmentVariable('TURTLEBOT_STACKS', default_value='hexagons')),
        DeclareLaunchArgument('3d_sensor', default_value=EnvironmentVariable('TURTLEBOT_3D_SENSOR', default_value='kinect'))
    ]

def load_xacro_file(turtlebot_system, base, stacks, sensor):
    return Node(
        package='xacro',
        executable='xacro',
        arguments=[
            os.path.join(
                FindPackageShare('turtlebot_description'), 'robots', 
                f'{LaunchConfiguration(base)}_{LaunchConfiguration(stacks)}_{LaunchConfiguration(sensor)}.urdf.xacro'
            )
        ],
        output='screen',
        parameters=[{'name': LaunchConfiguration(turtlebot_system) + 'description'}]
    )

def spawn_turtlebot(turtlebot_system):
    return Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=f'{LaunchConfiguration(turtlebot_system)}spawner',
        arguments=[
            '-x', '-4.0', '-y', '-0.2', '-z', '0.1',
            '-urdf', '-unpause',
            '-param', f'{LaunchConfiguration(turtlebot_system)}description',
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
        remappings=[('robot_description', 'turtlebot_description')]
    )

def generate_launch_description():
    return LaunchDescription([
        *declare_launch_arguments(),
        load_xacro_file('turtlebot_system', 'base', 'stacks', '3d_sensor'),
        spawn_turtlebot('turtlebot_system'),
        turtlebot_state_publisher()
    ])

if __name__ == '__main__':
    generate_launch_description()
