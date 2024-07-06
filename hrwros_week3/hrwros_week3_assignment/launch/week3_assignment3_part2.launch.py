from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Navigate to target2
        Node(
            package='hrwros_week3_assignment',
            executable='<update_your_code_here>',
            name='move_turtlebot_target2',
            output='screen'
        ),
        # Add Unknown obstacles
        Node(
            package='hrwros_week3',
            executable='create_unknown_obstacles.py',
            name='spawn_unknown_obstacles',
            output='screen'
        )
    ])
