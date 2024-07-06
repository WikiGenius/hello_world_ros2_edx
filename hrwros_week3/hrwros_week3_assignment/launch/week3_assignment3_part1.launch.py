from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hrwros_week3_assignment',
            executable='<update_your_code_here>',
            name='move_turtlebot_target1',
            output='screen'
        ),
    ])
