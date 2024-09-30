import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def declare_simulation_arguments():
    """Declare common simulation-related launch arguments."""
    return [
        DeclareLaunchArgument('paused', default_value='true', description='Start the Gazebo simulation paused.'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time.'),
        DeclareLaunchArgument('gui', default_value='true', description='Open Gazebo with the GUI.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz.'),
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo headless.'),
        DeclareLaunchArgument('debug', default_value='false', description='Run Gazebo in debug mode.'),
        DeclareLaunchArgument('extra_gazebo_args', default_value='--verbose', description='Extra arguments for Gazebo.'),
        DeclareLaunchArgument('config_topics', default_value=os.path.join(
            get_package_share_directory('hrwros_gazebo'), 'config', 'twist_mux_topics.yaml'),
            description='Default topics config file'),
        DeclareLaunchArgument('cmd_vel_out', default_value='cmd_vel', description='cmd vel output topic')
    ]

def load_gazebo_launch(gzserver_launch_path, gzclient_launch_path, world_path, extra_gazebo_args, paused, debug):
    """Load Gazebo server and client launch files."""
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzserver_launch_path),
        launch_arguments={'world': world_path, 'extra_gazebo_args': extra_gazebo_args, 'paused': paused, 'debug': debug}.items()
    )

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzclient_launch_path),
        launch_arguments={'extra_gazebo_args': extra_gazebo_args}.items()
    )

    return gzserver_launch, gzclient_launch

def create_tf_static_publisher_nodes():
    """Create static TF publisher nodes."""
    odom_world_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_world',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'world']
    )

    map_to_target1_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_target1',
        arguments=['3.75', '2.0', '0', '0', '0', '0', '1', 'map', 'turtlebot_target1']
    )

    map_to_target2_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_target2',
        arguments=['-4.168', '-1.112', '0', '0', '0', '0', '1', 'map', 'turtlebot_target2']
    )

    return [odom_world_transform, map_to_target1_transform, map_to_target2_transform]

def execute_delayed_process(delay, package_name, launch_file):
    """Create an ExecuteProcess action for a delayed launch."""
    return ExecuteProcess(
        cmd=["ros2", "run", "hrwros_gazebo", "timed_ros2_launch.sh", str(delay), package_name, launch_file],
        name=f'delayed launch {delay} sec', 
        output="screen"
    )

def create_twist_mux_node(use_sim_time, config_topics_arg, cmd_vel_out_arg):
    """Create the twist_mux node configuration."""
    return Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings=[('/cmd_vel_out', cmd_vel_out_arg)],
        parameters=[{'use_sim_time': use_sim_time}, config_topics_arg]
    )