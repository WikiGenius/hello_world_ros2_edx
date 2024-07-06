from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_arguments():
    return [
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('hrwros_week3_assignment'),
                'rviz',
                'hrwros_navigation.rviz'
            ]),
            description='Full path to the RViz config file'
        ),
    ]

def generate_rviz_node():
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_file')]
    )

def generate_launch_description():
    return LaunchDescription(
        generate_launch_arguments() + [
            generate_rviz_node(),
        ]
    )

