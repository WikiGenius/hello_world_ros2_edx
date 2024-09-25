from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    """Launch file to check controllers, hardware, and system status for robot1 and robot2."""

    # Check hardware components for robot1
    list_robot1_hardware = ExecuteProcess(
        cmd=['ros2', 'control', 'list_hardware_components', '-c', '/robot1/controller_manager'],
        output='screen',
        name='list_robot1_hardware'
    )

    # Check hardware components for robot2
    list_robot2_hardware = ExecuteProcess(
        cmd=['ros2', 'control', 'list_hardware_components', '-c', '/robot2/controller_manager'],
        output='screen',
        name='list_robot2_hardware'
    )

    # List controllers for robot1
    list_robot1_controllers = ExecuteProcess(
        cmd=['ros2', 'control', 'list_controllers', '-c', '/robot1/controller_manager'],
        output='screen',
        name='list_robot1_controllers'
    )

    # List controllers for robot2
    list_robot2_controllers = ExecuteProcess(
        cmd=['ros2', 'control', 'list_controllers', '-c', '/robot2/controller_manager'],
        output='screen',
        name='list_robot2_controllers'
    )

    # Check available topics in the system
    list_topics = ExecuteProcess(
        cmd=['ros2', 'topic', 'list'],
        output='screen',
        name='list_topics'
    )

    # Check active ROS 2 nodes
    list_nodes = ExecuteProcess(
        cmd=['ros2', 'node', 'list'],
        output='screen',
        name='list_nodes'
    )

    return LaunchDescription([
        list_robot1_hardware,
        list_robot2_hardware,
        list_robot1_controllers,
        list_robot2_controllers,
        list_topics,
        list_nodes
    ])
