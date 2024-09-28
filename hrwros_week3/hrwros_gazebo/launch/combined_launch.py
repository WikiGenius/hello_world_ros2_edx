from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
import os

DELAY_SPAWN_TURTLEBOT = 7


def generate_launch_description():
    # Declare the launch arguments
    declare_paused = DeclareLaunchArgument(
        'paused', default_value='true', description='Start the Gazebo simulation paused.'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time.'
    )
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true', description='Open Gazebo with the GUI.'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true', description='Launch RViz.'
    )
    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false', description='Run Gazebo headless.'
    )
    declare_debug = DeclareLaunchArgument(
        'debug', default_value='false', description='Run Gazebo in debug mode.'
    )
    declare_extra_gazebo_args = DeclareLaunchArgument(
        'extra_gazebo_args', default_value='--verbose', description='Extra arguments for Gazebo.'
    )

    # Retrieve the arguments with LaunchConfiguration
    paused = LaunchConfiguration('paused')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    headless = LaunchConfiguration('headless')
    debug = LaunchConfiguration('debug')
    extra_gazebo_args = LaunchConfiguration('extra_gazebo_args')

    # Find package directories
    gazebo_ros_share = FindPackageShare('gazebo_ros').find('gazebo_ros')
    hrwros_gazebo_share = FindPackageShare(
        'hrwros_gazebo').find('hrwros_gazebo')

    # Paths to Gazebo world and launch files
    world_path = os.path.join(hrwros_gazebo_share, 'worlds', 'hrwros.world')
    gzserver_launch_path = os.path.join(
        gazebo_ros_share, 'launch', 'gzserver.launch.py')
    gzclient_launch_path = os.path.join(
        gazebo_ros_share, 'launch', 'gzclient.launch.py')
    spawn_static_objects_launch_path = os.path.join(
        hrwros_gazebo_share, 'launch', 'spawn_static_world_objects.launch.py')
    spawn_robots_launch_path = os.path.join(
        hrwros_gazebo_share, 'launch', 'spawn_robots.launch.py')
    spawn_turtlebot_launch_path = os.path.join(
        hrwros_gazebo_share, 'launch', 'spawn_turtlebot.launch.py')

    # Include the Gazebo server (gzserver)
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzserver_launch_path),
        launch_arguments={
            'world': world_path,
            'extra_gazebo_args': extra_gazebo_args,
            'paused': paused,
            'debug': debug,
        }.items()
    )

    # Include the Gazebo client (gzclient) with GUI
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzclient_launch_path),
        launch_arguments={
            'extra_gazebo_args': extra_gazebo_args,
        }.items()
    )

    # Include launch files for spawning static world objects, robots, and turtlebot
    spawn_static_objects = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_static_objects_launch_path)
    )

    spawn_robots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_robots_launch_path)
    )

    spawn_turtlebot = ExecuteProcess(
        cmd=["ros2", "run", "hrwros_gazebo", "timed_ros2_launch.sh", str(
            DELAY_SPAWN_TURTLEBOT), "hrwros_gazebo", "spawn_turtlebot.launch.py"],
        output="screen"
    )
    # Define static_transform_publisher nodes for TF2
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
        arguments=['3.75', '2.0', '0', '0', '0',
                   '0', '1', 'map', 'turtlebot_target1']
    )

    map_to_target2_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_target2',
        arguments=['-4.168', '-1.112', '0', '0', '0',
                   '0', '1', 'map', 'turtlebot_target2']
    )

    # Return the LaunchDescription containing all the actions
    return LaunchDescription([
        declare_paused,
        declare_use_sim_time,
        declare_gui,
        declare_rviz,
        declare_headless,
        declare_debug,
        declare_extra_gazebo_args,
        gzserver_launch,
        gzclient_launch,
        spawn_static_objects,
        spawn_robots,
        spawn_turtlebot,
        odom_world_transform,
        map_to_target1_transform,
        map_to_target2_transform
    ])
