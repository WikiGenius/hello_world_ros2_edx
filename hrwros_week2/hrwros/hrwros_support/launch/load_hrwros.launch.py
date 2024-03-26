from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    #  List to hold all declarations for cleaner assembly
    declare_arguments = [
        DeclareLaunchArgument(
            'name',
            default_value='robot1',
            description='Name of the arm robot'
        ),
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur10',
            description='Type of the arm robot'
        ),
        DeclareLaunchArgument(
            'robot_description',
            default_value=Command([
                FindExecutable(name='xacro'),
                ' ',
                PathJoinSubstitution([
                    get_package_share_directory('hrwros_support'),
                    'urdf',
                    'hrwros.xacro'
                ]),
                ' ',
                'name:=', LaunchConfiguration('name'),
                ' ',
                'ur_type:=', LaunchConfiguration('ur_type')
            ]),
            description='Full robot description'
        ),
    ]

    return LaunchDescription(declare_arguments)
