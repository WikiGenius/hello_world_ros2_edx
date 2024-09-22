from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_description_command(package: str, urdf_file: str, *args: str) -> Command:
    """Generate a command to process a URDF file with xacro.

    Args:
        package (str): Name of the package where the URDF is located.
        urdf_file (str): Name of the URDF file.
        *args (str): Additional arguments for xacro.

    Returns:
        Command: The command to execute.
    """
    return Command([
        'xacro ',
        PathJoinSubstitution([FindPackageShare(package), 'urdf', urdf_file]),
        ' ',
        *args
    ])


def generate_robot_state_publisher(name: str, description_command: Command) -> Node:
    """Create a Node that runs the robot_state_publisher.

    Args:
        name (str): Name of the robot.
        description_command (Command): Command to generate the robot description.

    Returns:
        Node: The robot_state_publisher node.
    """
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name=f'{name}_state_publisher',
        parameters=[{'robot_description': description_command}],
        remappings=[('/robot_description', f'/{name}_description')]
    )


def generate_spawner_node(entity_name: str, topic: str, x: float = None, y: float = None) -> Node:
    """Create a Node that spawns an entity in Gazebo.

    Args:
        entity_name (str): Name of the entity to spawn.
        topic (str): Topic name for the robot description.
        x (float, optional): X-coordinate for the entity. Defaults to None.
        y (float, optional): Y-coordinate for the entity. Defaults to None.

    Returns:
        Node: The spawn_entity node.
    """
    arguments = ['-entity', entity_name, '-topic', topic]
    
    # Add position arguments if provided
    if x is not None and y is not None:
        arguments.extend(['-x', str(x), '-y', str(y)])
        
    return Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=f'spawn_{entity_name}',
        output='screen',
        arguments=arguments
    )
