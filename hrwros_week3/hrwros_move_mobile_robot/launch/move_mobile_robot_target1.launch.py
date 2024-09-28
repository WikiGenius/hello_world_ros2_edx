from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    args = [
        DeclareLaunchArgument('goal_x', default_value = '3.75'),
        DeclareLaunchArgument('goal_y', default_value = '2.0')
    ]
    return LaunchDescription([
        *args,
        Node(
            package='hrwros_move_mobile_robot',
            executable='move_mobile_robot',
            name='move_mobile_robot_target1',
            output='screen',
            parameters=[{'goal_x': goal_x}, {'goal_y': goal_y}]
        ),
    ])
