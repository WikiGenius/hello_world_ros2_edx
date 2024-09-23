import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_description_command(package: str, urdf_file: str, *args: str) -> Command:
    """Generate a command to process a URDF file using xacro.

    Args:
        package (str): The package containing the URDF file.
        urdf_file (str): The name of the URDF file.
        *args (str): Additional xacro arguments.

    Returns:
        Command: The command to execute the xacro process.
    """
    urdf_path = PathJoinSubstitution([FindPackageShare(package), 'urdf', urdf_file])
    return Command(['xacro ', urdf_path, ' ', *args])


def generate_robot_state_publisher(name: str, description_command: Command, ns_robot: str = None) -> Node:
    """Create a Node for robot_state_publisher.

    Args:
        name (str): The name of the robot.
        description_command (Command): The command that generates the robot description.
        ns_robot (str, optional): Namespace for the robot.

    Returns:
        Node: The robot_state_publisher node.
    """
    topic = '/robot_description'
    new_topic = topic if ns_robot else f'/{name}_description'

    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'{name}_state_publisher',
        output='screen',
        parameters=[{'robot_description': description_command}],
        remappings=[(topic, new_topic)]
    )


def generate_spawner_node(entity_name: str, topic: str, x: float = None, y: float = None, z: float = None, yaw: float = None) -> Node:
    """Create a Node that spawns an entity in Gazebo.

    Args:
        entity_name (str): The name of the entity to spawn.
        topic (str): The topic name for the robot description.
        x (float, optional): X-coordinate for the entity.
        y (float, optional): Y-coordinate for the entity.
        z (float, optional): Z-coordinate for the entity.
        yaw (float, optional): Yaw angle for the entity in radians.

    Returns:
        Node: The spawn_entity node.
    """
    arguments = ['-entity', entity_name, '-topic', topic, '-unpause']
    if x is not None: arguments += ['-x', str(x)]
    if y is not None: arguments += ['-y', str(y)]
    if z is not None: arguments += ['-z', str(z)]
    if yaw is not None: arguments += ['-Y', str(yaw)]

    return Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=f'spawn_{entity_name}',
        output='screen',
        arguments=arguments
    )


def get_joint_state_publisher_nodes(name: str, package: str, joint_states_yaml: str, gui_config: str) -> list:
    """Create Nodes for joint_state_publisher with and without GUI.

    Args:
        name (str): The name of the robot.
        package (str): The package containing the joint state configuration.
        joint_states_yaml (str): The YAML file for joint states.
        gui_config (str): Condition for whether to use GUI or not.

    Returns:
        list: A list containing the joint_state_publisher nodes (GUI and non-GUI).
    """
    joint_state_publisher_params = PathJoinSubstitution([
        FindPackageShare(package), 'config', joint_states_yaml
    ])
    common_remappings = [('/robot_description', f'/{name}_description')]

    gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        parameters=[joint_state_publisher_params],
        condition=IfCondition(gui_config),
        remappings=common_remappings
    )
    non_gui_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[joint_state_publisher_params],
        condition=UnlessCondition(gui_config),
        remappings=common_remappings
    )
    return [gui_node, non_gui_node]


def get_rviz_node(rviz_config_file: str) -> Node:
    """Create a Node for launching RViz.

    Args:
        rviz_config_file (str): Path to the RViz configuration file.

    Returns:
        Node: The RViz node.
    """
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_file],
        output='screen'
    )


def load_config_file(package: str, config_base_name: str) -> dict:
    """Load a YAML configuration file.

    Args:
        package (str): The package containing the configuration file.
        config_base_name (str): The base name of the configuration file.

    Returns:
        dict: The loaded configuration parameters.
    """
    config_file_path = os.path.join(
        get_package_share_directory(package), 'config', config_base_name
    )
    
    with open(config_file_path, 'r') as config_file:
        config_params = yaml.safe_load(config_file)
    
    return config_params.get('robot_config', {}).get('ros__parameters', {})
