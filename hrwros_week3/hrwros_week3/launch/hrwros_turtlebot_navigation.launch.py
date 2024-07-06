import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def declare_launch_arguments():
    return [
        DeclareLaunchArgument('paused', default_value='true', description='Pause simulation on start'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('gui', default_value='true', description='Run GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Run RViz'),
        DeclareLaunchArgument('headless', default_value='false', description='Run headless'),
        DeclareLaunchArgument('debug', default_value='false', description='Run in debug mode'),
        DeclareLaunchArgument('extra_gazebo_args', default_value='--verbose', description='Extra arguments for Gazebo')
    ]

def include_launch_description(package_name, launch_file_name, launch_arguments):
    package_share_directory = get_package_share_directory(package_name)
    launch_file_path = os.path.join(package_share_directory, 'launch', launch_file_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_path),
        launch_arguments=launch_arguments.items()
    )

def create_execute_process():
    return ExecuteProcess(
        cmd=['timed_roslaunch.sh', '7', 'hrwros_gazebo', 'spawn_turtlebot.launch'],
        output='screen'
    )

def create_nodelet_manager_nodes(hrwros_gazebo_share):
    return [
        Node(
            package='nodelet',
            executable='nodelet',
            name='mobile_base_nodelet_manager',
            arguments=['manager']
        ),
        Node(
            package='nodelet',
            executable='nodelet',
            name='cmd_vel_mux',
            arguments=['load', 'yocs_cmd_vel_mux/CmdVelMuxNodelet', 'mobile_base_nodelet_manager'],
            parameters=[{'yaml_cfg_file': os.path.join(hrwros_gazebo_share, 'param', 'mux.yaml')}],
            remappings=[
                ('cmd_vel_mux/output', 'mobile_base/commands/velocity')
            ]
        ),
        Node(
            package='nodelet',
            executable='nodelet',
            name='laserscan_nodelet_manager',
            arguments=['manager']
        ),
        Node(
            package='nodelet',
            executable='nodelet',
            name='depthimage_to_laserscan',
            arguments=['load', 'depthimage_to_laserscan/DepthImageToLaserScanNodelet', 'laserscan_nodelet_manager'],
            parameters=[
                {'scan_height': 10},
                {'output_frame_id': 'camera_depth_frame'},
                {'range_min': 0.45}
            ],
            remappings=[
                ('image', '/camera/depth/image_raw'),
                ('scan', '/scan')
            ]
        )
    ]

def create_static_transform_nodes():
    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_target1',
            arguments=['0.13', '1.44', '0', '0', '0', '0', '1', 'map', 'turtlebot_target1']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_target2',
            arguments=['-8.1', '-1.43', '0', '0', '0', '0', '1', 'map', 'turtlebot_target2']
        )
    ]

def generate_launch_description():
    paused = LaunchConfiguration('paused')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    extra_gazebo_args = LaunchConfiguration('extra_gazebo_args')

    hrwros_gazebo_share = get_package_share_directory('hrwros_gazebo')

    launch_arguments = {
        'world': os.path.join(hrwros_gazebo_share, 'worlds', 'hrwros.world'),
        'paused': paused,
        'use_sim_time': use_sim_time,
        'gui': gui,
        'headless': headless,
        'extra_gazebo_args': extra_gazebo_args
    }

    return LaunchDescription([
        *declare_launch_arguments(),

        include_launch_description('gazebo_ros', 'gazebo.launch.py', launch_arguments),

        include_launch_description('hrwros_support', 'load_hrwros.launch.py', {}),
        include_launch_description('hrwros_gazebo', 'spawn_static_world_objects.launch.py', {}),
        include_launch_description('hrwros_gazebo', 'spawn_robots.launch.py', {}),

        create_execute_process(),

        *create_nodelet_manager_nodes(hrwros_gazebo_share),

        *create_static_transform_nodes()
    ])

if __name__ == '__main__':
    generate_launch_description()
