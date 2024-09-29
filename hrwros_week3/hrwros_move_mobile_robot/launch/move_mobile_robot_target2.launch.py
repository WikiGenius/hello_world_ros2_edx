from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

DELAY_SPAWN = 4


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
            package='hrwros_move_mobile_robot',
            executable='move_mobile_robot',
            name='move_mobile_robot_target2',
            output='screen',
            parameters=[{'goal_x': goal_x}, {'goal_y': goal_y}]
        ),
        # Wait for a couple of seconds to prevent the unknown obstacle from
        # being considered as a part of the map.
        # Add Unknown obstacles

        ExecuteProcess(
            cmd=["ros2", "run", "hrwros_gazebo", "timed_ros2_run.sh",
                 str(DELAY_SPAWN),
                 "hrwros_move_mobile_robot", "create_unknown_obstacles"],
            output="screen")
    ])
