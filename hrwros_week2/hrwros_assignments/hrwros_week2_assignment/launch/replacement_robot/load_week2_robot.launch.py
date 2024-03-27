from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    #  List to hold all declarations for cleaner assembly
    declare_arguments = [
        DeclareLaunchArgument(
            'robot_description',
            default_value=Command([
                FindExecutable(name='xacro'),
                ' ',
                PathJoinSubstitution([
                    get_package_share_directory('hrwros_week2_assignment'),
                    'urdf',
                    'robot',
                    'lrmate200ic.xacro'
                ]),
            ]),
            description='Full robot description'
        ),
    ]

    return LaunchDescription(declare_arguments)
