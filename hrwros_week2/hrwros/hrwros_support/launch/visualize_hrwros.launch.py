from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node


def generate_launch_description():
    # Define arguments
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Flag to enable GUI for joint state publisher.')

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

    # Launch the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': LaunchConfiguration('robot_description')}]
    )

    # Launch the joint_state_publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui' if LaunchConfiguration(
            'gui') == 'true' else 'joint_state_publisher',
        executable='joint_state_publisher_gui' if LaunchConfiguration(
            'gui') == 'true' else 'joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_gui': LaunchConfiguration('gui'),
            'robot_description': LaunchConfiguration('robot_description'),
            'zeros': {
                'robot1_shoulder_pan_joint': 0.6207787083493841,
                'robot1_shoulder_lift_joint': -1.004681330618082,
                'robot1_elbow_joint': 1.6983449885307538,
                'robot1_wrist_1_joint': -2.301530778020034,
                'robot1_wrist_2_joint': -1.625460038967466,
                'robot1_wrist_3_joint': 0.0,
                'robot2_shoulder_pan_joint': 1.5707963267950005,
                'robot2_shoulder_lift_joint': -1.5525750894041779,
                'robot2_elbow_joint': 1.5525750894041783,
                'robot2_wrist_1_joint': -1.570796326795,
                'robot2_wrist_2_joint': -1.534353852013356,
                'robot2_wrist_3_joint': 0.0,
            }
        }]
    )
    # Define the xacro file path
    rviz_file = PathJoinSubstitution([pkg_dir, 'config', 'hrwros.rviz'])
    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_file],
        output='screen',
    )
    return LaunchDescription([
        gui_arg,
        robot_description_param,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
