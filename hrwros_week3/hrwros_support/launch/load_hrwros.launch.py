from my_python_utils import some_utility
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def get_declare_arguments():
    hrwros_description = some_utility.generate_description_command(
        'hrwros_support', 'hrwros.xacro')
    return [
        DeclareLaunchArgument(
            'robot_description', default_value=hrwros_description, description='Full robot description'
        ),
    ]


def generate_launch_description():
    # Call the function to get the list of declare arguments
    declare_arguments = get_declare_arguments()
    return LaunchDescription(declare_arguments)
