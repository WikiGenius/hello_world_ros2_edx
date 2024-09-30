import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from my_python_utils import helper_combination

# Constants for delay
DELAY_SPAWN_TURTLEBOT = 7
DELAY_SPAWN_NAVIGATION = 10


def generate_launch_description():
    # Package directories
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    hrwros_gazebo_share = get_package_share_directory('hrwros_gazebo')

    # Paths to world and launch files
    world_path = os.path.join(hrwros_gazebo_share, 'worlds', 'hrwros.world')
    gzserver_launch_path = os.path.join(
        gazebo_ros_share, 'launch', 'gzserver.launch.py')
    gzclient_launch_path = os.path.join(
        gazebo_ros_share, 'launch', 'gzclient.launch.py')
    spawn_static_objects_launch_path = os.path.join(
        hrwros_gazebo_share, 'launch', 'spawn_static_world_objects.launch.py')
    spawn_robots_launch_path = os.path.join(
        hrwros_gazebo_share, 'launch', 'spawn_robots.launch.py')

    # Declare simulation-related launch arguments
    launch_args = helper_combination.declare_simulation_arguments()

    # Gazebo server and client launch files
    paused = LaunchConfiguration('paused')
    use_sim_time = LaunchConfiguration('use_sim_time')
    extra_gazebo_args = LaunchConfiguration('extra_gazebo_args')
    debug = LaunchConfiguration('debug')
    gzserver_launch, gzclient_launch = helper_combination.load_gazebo_launch(
        gzserver_launch_path, gzclient_launch_path, world_path, extra_gazebo_args, paused, debug)

    # Static object and robot spawn launch files
    spawn_static_objects = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_static_objects_launch_path)
    )
    spawn_robots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_robots_launch_path)
    )

    # Delayed processes for spawning turtlebot and navigation bringup
    spawn_turtlebot = helper_combination.execute_delayed_process(
        DELAY_SPAWN_TURTLEBOT, "hrwros_gazebo", "spawn_turtlebot.launch.py")
    bringup_launch = helper_combination.execute_delayed_process(
        DELAY_SPAWN_NAVIGATION, "hrwros_nav", "bringup_launch.py")

    # Static TF publisher nodes
    tf_static_nodes = helper_combination.create_tf_static_publisher_nodes()

    # Twist Mux Node
    cmd_vel_out_arg = LaunchConfiguration('cmd_vel_out')
    config_topics_arg = LaunchConfiguration('config_topics')
    twist_mux_node = helper_combination.create_twist_mux_node(
        use_sim_time, config_topics_arg, cmd_vel_out_arg)

    # Return LaunchDescription containing all actions
    return LaunchDescription(
        launch_args +
        [
            gzserver_launch,
            gzclient_launch,
            spawn_static_objects,
            spawn_robots,
            spawn_turtlebot,
            twist_mux_node,
            bringup_launch
        ]
        + tf_static_nodes
    )
