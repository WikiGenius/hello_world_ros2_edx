from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from my_python_utils import some_utility


def generate_args():
    args = [

        DeclareLaunchArgument('robot_parent', default_value='world'),
        DeclareLaunchArgument('robot1_name', default_value='robot1'),
        DeclareLaunchArgument('robot2_name', default_value='robot2'),
        DeclareLaunchArgument('robot1_type', default_value='ur10'),
        DeclareLaunchArgument('robot2_type', default_value='ur5'),
        DeclareLaunchArgument('vacuum_gripper1_prefix',
                              default_value='vacuum_gripper1_'),
        DeclareLaunchArgument('vacuum_gripper2_prefix',
                              default_value='vacuum_gripper2_'),
        DeclareLaunchArgument('gripper1_plugin_name',
                              default_value='gripper1'),
        DeclareLaunchArgument('gripper2_plugin_name',
                              default_value='gripper2'),
    ]
    return args


def generate_robot_group(robot_name, robot_type, robot_parent, urdf_file,
                         vacuum_gripper_prefix, gripper_plugin_name,
                         x, y, z, yaw=None, joints=None):
    """Generate a group of nodes for a single robot."""
    robot_prefix = robot_name
    description_command = some_utility.generate_description_command(
        'hrwros_support',
        urdf_file,
        f'robot_type:={robot_type} ',
        f'robot_prefix:={robot_prefix} ',
        f'vacuum_gripper_prefix:={vacuum_gripper_prefix} ',
        f'gripper_plugin_name:={gripper_plugin_name} ',
        f'robot_parent:={robot_parent} '
    )

    return GroupAction(
        actions=[
            PushRosNamespace(robot_name),
            some_utility.generate_robot_state_publisher(
                robot_name, description_command, ns_robot=robot_name),
            some_utility.generate_spawner_node(
                entity_name=robot_name,
                topic=f'/{robot_name}/robot_description',
                x=x, y=y, z=z, yaw=yaw,
            ),
            # *generate_controller_spawner_node(robot_prefix, description_command)
        ])


def generate_launch_description():
    """Generate the launch description for the robots."""

    return LaunchDescription([
        # Declare arguments

        *generate_args(),
        # Group for Robot 1
        generate_robot_group(
            robot_name='robot1',
            robot_type='ur10',
            robot_parent='world',
            urdf_file='robot_system/robot_system.xacro',
            vacuum_gripper_prefix='vacuum_gripper1_',
            gripper_plugin_name='gripper1',
            x=0.5, y=1.8, z=0.95,
            joints={
                'robot1_elbow_joint': 1.57,
                'robot1_shoulder_lift_joint': -1.57,
                'robot1_shoulder_pan_joint': 1.24,
                'robot1_wrist_1_joint': -1.57,
                'robot1_wrist_2_joint': -1.57,
            }
        ),

        # Group for Robot 2
        generate_robot_group(
            robot_name='robot2',
            robot_type='ur5',
            robot_parent='world',
            urdf_file='robot_system/robot_system.xacro',
            vacuum_gripper_prefix='vacuum_gripper2_',
            gripper_plugin_name='gripper2',
            x=-7.8, y=-1.5, z=0.7, yaw=1.57,
            joints={
                'robot2_elbow_joint': 1.57,
                'robot2_shoulder_lift_joint': -1.57,
                'robot2_shoulder_pan_joint': 1.24,
                'robot2_wrist_1_joint': -1.57,
                'robot2_wrist_2_joint': -1.57,
            }
        ),
    ])
