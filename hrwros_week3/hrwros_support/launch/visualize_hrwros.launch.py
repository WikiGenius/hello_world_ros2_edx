from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from my_python_utils import some_utility


def get_declare_arguments():
    return [
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Flag to enable GUI for joint state publisher.'
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution(
                [get_package_share_directory('hrwros_support'), 'rviz', 'hrwros.rviz']),
            description='Full path to the RViz config file'
        )
    ]


def include_robot_description_launch():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('hrwros_support'),
                'launch',
                'load_hrwros.launch.py'
            ])
        ])
    )


def generate_launch_description():

    declare_arguments = get_declare_arguments()

    description_command = ParameterValue(
        LaunchConfiguration('robot_description'),
        value_type=str
    )
    robot_name = 'hrwros'
    # Include the reusable robot description launch file
    robot_description_launch = include_robot_description_launch()
    robot_state_publisher_node = \
        some_utility.generate_robot_state_publisher(robot_name,
                                                    description_command)
    joint_state_publisher_nodes = \
        some_utility.get_joint_state_publisher_nodes(robot_name,
                                                     'hrwros_support',
                                                     'joint_states.yaml',
                                                     LaunchConfiguration('gui'))
    rviz_node = some_utility.get_rviz_node(
        LaunchConfiguration('rviz_config_file'))

    launch_description_elements = declare_arguments + [
        robot_description_launch,
        robot_state_publisher_node,
        *joint_state_publisher_nodes,
        rviz_node
    ]

    return LaunchDescription(launch_description_elements)
