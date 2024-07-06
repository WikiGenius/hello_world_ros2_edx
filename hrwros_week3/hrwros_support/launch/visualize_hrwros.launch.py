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


def get_robot_state_publisher_node():
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(
                LaunchConfiguration('robot_description'), value_type=str)},
        ]
    )


def get_joint_state_publisher_nodes():
    joint_state_publisher_params = PathJoinSubstitution([
        FindPackageShare('hrwros_support'),
        'config',
        'joint_states.yaml'
    ])
    gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        parameters=[joint_state_publisher_params],
        condition=IfCondition(LaunchConfiguration('gui')),
    )
    non_gui_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[joint_state_publisher_params],
        condition=UnlessCondition(LaunchConfiguration('gui')),
    )
    return [gui_node, non_gui_node]


def get_rviz_node():
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        output='screen',
    )


def generate_launch_description():
    declare_arguments = get_declare_arguments()
    # Include the reusable robot description launch file
    robot_description_launch = include_robot_description_launch()
    robot_state_publisher_node = get_robot_state_publisher_node()
    joint_state_publisher_nodes = get_joint_state_publisher_nodes()
    rviz_node = get_rviz_node()

    launch_description_elements = declare_arguments + [
        robot_description_launch,
        robot_state_publisher_node,
        *joint_state_publisher_nodes,
        rviz_node
    ]

    return LaunchDescription(launch_description_elements)
