from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_description_command(package, urdf_file, *args):
    """Generate a command to process the robot's xacro file."""
    return Command([
        'xacro ',
        PathJoinSubstitution([FindPackageShare(package), 'urdf', urdf_file]),
        ' ',
        *args
    ])

def generate_robot_state_publisher(robot_prefix, description_command):
    """Create a robot_state_publisher node."""
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name=f'{robot_prefix}_state_publisher',
        parameters=[{'robot_description': description_command, 'tf_prefix': robot_prefix}],
        remappings=[('/robot_description', f'/{robot_prefix}_description')]
    )


def generate_spawner_node(name, entity_name, topic, x=None, y=None, z=None, yaw=None, joints=None):
    """Spawn the robot model in Gazebo."""
    arguments = ['-entity', entity_name, '-topic', topic]
    if x is not None and y is not None:
        arguments.extend(['-x', str(x), '-y', str(y)])
    if z is not None:
        arguments.extend(['-z', str(z)])
    if yaw is not None:
        arguments.extend(['-Y', str(yaw)])
    if joints:
        for joint, position in joints.items():
            arguments.extend(['-J', joint, str(position)])
    return Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=arguments
    )


def generate_controller_spawner_node(robot_prefix):
    """Spawn the controller for the robot."""
    return Node(
        package='controller_manager',
        executable='spawner',
        name=f'{robot_prefix}_controller_spawner',
        arguments=[f'{robot_prefix}_joint_state_controller',
                   f'{robot_prefix}_controller']
    )


def generate_robot_group(robot_prefix, robot_type, urdf_file, vacuum_gripper_prefix, gripper_plugin_name, x, y, z, yaw=None, joints=None):
    """Generate a group of nodes for a single robot."""
    description_command = generate_description_command(
        'hrwros_support',
        urdf_file,
        f'robot_type:={robot_type}',
        f'robot_prefix:={robot_prefix}_',
        f'vacuum_gripper_prefix:={vacuum_gripper_prefix}',
        f'robot_param:=/{robot_prefix}_description',
        f'gripper_plugin_name:={gripper_plugin_name}'
    )

    return GroupAction([
        PushRosNamespace(robot_prefix),
        generate_robot_state_publisher(robot_prefix, description_command),
        generate_spawner_node(
            name=robot_prefix,
            entity_name=robot_prefix,
            topic=f'/{robot_prefix}_description',
            x=x, y=y, z=z, yaw=yaw,
            joints=joints
        ),
        generate_controller_spawner_node(robot_prefix)
    ])


def generate_launch_description():
    """Generate the launch description for the robots."""
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('robot1_prefix', default_value='robot1_'),
        DeclareLaunchArgument('robot2_prefix', default_value='robot2_'),
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

        # Group for Robot 1
        generate_robot_group(
            robot_prefix=LaunchConfiguration('robot1_prefix'),
            robot_type=LaunchConfiguration('robot1_type'),
            urdf_file='robot_system/robot_system.xacro',
            vacuum_gripper_prefix=LaunchConfiguration(
                'vacuum_gripper1_prefix'),
            gripper_plugin_name=LaunchConfiguration('gripper1_plugin_name'),
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
            robot_prefix=LaunchConfiguration('robot2_prefix'),
            robot_type=LaunchConfiguration('robot2_type'),
            urdf_file='robot_system/robot_system.xacro',
            vacuum_gripper_prefix=LaunchConfiguration(
                'vacuum_gripper2_prefix'),
            gripper_plugin_name=LaunchConfiguration('gripper2_plugin_name'),
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
