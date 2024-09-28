from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    args = [
        DeclareLaunchArgument('goal_x', default_value='-4.168'),
        DeclareLaunchArgument('goal_y', default_value='-1.112')
    ]
    return LaunchDescription([
        *args,
        Node(
            package='hrwros_week3_assignment',
            executable='move_mobile_robot',
            name='move_mobile_robot_target2',
            output='screen',
            parameters=[{'goal_x': goal_x}, {'goal_y': goal_y}]
        ),
        # Add Unknown obstacles
        Node(
            package='hrwros_week3_assignment',
            executable='create_unknown_obstacles',
            name='spawn_unknown_obstacles',
            output='screen'
        )
    ])
