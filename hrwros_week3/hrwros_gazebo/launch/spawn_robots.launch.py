import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def declare_launch_arguments():
    return [
        DeclareLaunchArgument('robot1_prefix', default_value='robot1_'),
        DeclareLaunchArgument('robot2_prefix', default_value='robot2_'),
        DeclareLaunchArgument('robot1_type', default_value='ur10'),
        DeclareLaunchArgument('robot2_type', default_value='ur5'),
        DeclareLaunchArgument('vacuum_gripper1_prefix', default_value='vacuum_gripper1_'),
        DeclareLaunchArgument('vacuum_gripper2_prefix', default_value='vacuum_gripper2_'),
        DeclareLaunchArgument('gripper1_plugin_name', default_value='gripper1'),
        DeclareLaunchArgument('gripper2_plugin_name', default_value='gripper2')
    ]

def load_xacro_file(robot_type, robot_prefix, vacuum_gripper_prefix, gripper_plugin_name, robot_namespace):
    return Node(
        package='xacro',
        executable='xacro',
        arguments=[
            os.path.join(FindPackageShare('hrwros_support'), 'urdf', 'robot_system', 'robot_system.xacro'),
            'robot_type:=', LaunchConfiguration(robot_type),
            'robot_prefix:=', LaunchConfiguration(robot_prefix),
            'vacuum_gripper_prefix:=', LaunchConfiguration(vacuum_gripper_prefix),
            'robot_param:=/', robot_namespace, '/', LaunchConfiguration(robot_prefix), 'description',
            'gripper_plugin_name:=', LaunchConfiguration(gripper_plugin_name)
        ],
        output='screen'
    )

def spawn_robot(robot_name, x, y, z, yaw, prefix):
    return Node(
        package='gazebo_ros',
        executable='spawn_model',
        name=f'{robot_name}_spawner',
        arguments=[
            '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw),
            '-urdf', '-model', LaunchConfiguration(prefix),
            '-param', f'{LaunchConfiguration(prefix)}description',
            '-J', f'{LaunchConfiguration(prefix)}elbow_joint', '1.57',
            '-J', f'{LaunchConfiguration(prefix)}shoulder_lift_joint', '-1.57',
            '-J', f'{LaunchConfiguration(prefix)}shoulder_pan_joint', '1.24',
            '-J', f'{LaunchConfiguration(prefix)}wrist_1_joint', '-1.57',
            '-J', f'{LaunchConfiguration(prefix)}wrist_2_joint', '-1.57'
        ],
        output='screen'
    )

def spawn_controllers(controller_namespace, joint_state_controller, robot_controller):
    return Node(
        package='controller_manager',
        executable='spawner',
        name=f'{controller_namespace}_controller_spawner',
        arguments=[joint_state_controller, robot_controller],
        output='screen'
    )

def include_rosparam_files(controller_namespace):
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                FindPackageShare('hrwros_gazebo'), 'config', f'{controller_namespace}_joint_state_controller.yaml'
            )])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                FindPackageShare('hrwros_gazebo'), 'config', f'{controller_namespace}_controller.yaml'
            )])
        )
    ]

def generate_robot_group(robot_namespace, robot_type, robot_prefix, vacuum_gripper_prefix, gripper_plugin_name, x, y, z, yaw, controller_namespace, joint_state_controller, robot_controller):
    return GroupAction([
        PushRosNamespace(robot_namespace),
        load_xacro_file(robot_type, robot_prefix, vacuum_gripper_prefix, gripper_plugin_name, robot_namespace),
        spawn_robot(robot_namespace, x, y, z, yaw, robot_prefix),
        spawn_controllers(controller_namespace, joint_state_controller, robot_controller),
        *include_rosparam_files(controller_namespace)
    ])

def generate_launch_description():
    return LaunchDescription([
        *declare_launch_arguments(),
        generate_robot_group('robot1', 'robot1_type', 'robot1_prefix', 'vacuum_gripper1_prefix', 'gripper1_plugin_name', 0.5, 1.8, 0.95, 0, 'r1', 'r1_joint_state_controller', 'robot1_controller'),
        generate_robot_group('robot2', 'robot2_type', 'robot2_prefix', 'vacuum_gripper2_prefix', 'gripper2_plugin_name', -7.8, -1.5, 0.7, 1.57, 'r2', 'r2_joint_state_controller', 'robot2_controller')
    ])

if __name__ == '__main__':
    generate_launch_description()
