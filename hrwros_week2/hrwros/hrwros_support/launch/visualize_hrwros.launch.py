from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # List to hold all declarations for cleaner assembly
    declare_arguments = [
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Flag to enable GUI for joint state publisher.'
        ),
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

    # # Nodes definition
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': LaunchConfiguration('robot_description')}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        condition=IfCondition(LaunchConfiguration('gui')),
        parameters=[PathJoinSubstitution([
            get_package_share_directory('hrwros_support'),
            'config',
            'zeros.yaml'
        ])]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui')),
        parameters=[{
            'zeros': LaunchConfiguration('zeros')
        }],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution(
            [get_package_share_directory('hrwros_support'), 'config', 'hrwros.rviz'])],
        output='screen',
    )

    launch_description_elements = declare_arguments + [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rviz_node
    ]

    return LaunchDescription(launch_description_elements)
