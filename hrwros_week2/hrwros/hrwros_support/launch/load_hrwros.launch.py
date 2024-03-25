from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('hrwros_support')

    # Define the xacro file path
    xacro_file = PathJoinSubstitution([pkg_dir, 'urdf', 'hrwros.xacro'])
    
    robot_description_param = DeclareLaunchArgument(
        name='robot_description',
        default_value=Command([
            FindExecutable(name='xacro'), ' ', xacro_file
        ]),
        description='Full robot description'
    )
    return LaunchDescription([

        robot_description_param
    ])
