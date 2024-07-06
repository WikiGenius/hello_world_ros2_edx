from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def get_declare_arguments():
    return [
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
            ]),
            description='Full robot description'
        ),
    ]

def generate_launch_description():
    # Call the function to get the list of declare arguments
    declare_arguments = get_declare_arguments()
    return LaunchDescription(declare_arguments)

