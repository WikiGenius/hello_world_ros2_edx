import yaml
import os
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit


def generate_description_command(package: str, urdf_file: str, *args: str) -> Command:
    """Generate a command to process a URDF file using xacro.

    Args:
        package (str): The package containing the URDF file.
        urdf_file (str): The name of the URDF file.
        *args (str): Additional xacro arguments.

    Returns:
        Command: The command to execute the xacro process.
    """
    urdf_path = PathJoinSubstitution(
        [FindPackageShare(package), 'urdf', urdf_file])
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
    node_name = 'robot_state_publisher' if ns_robot else f'{name}_state_publisher'

    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=node_name,
        output='screen',
        parameters=[{'robot_description': description_command}],
        remappings=[(topic, new_topic)]
    )


def generate_spawner_node(entity_name: str, ns_robot: str = None, x: float = None, y: float = None, z: float = None, yaw: float = None) -> Node:
    """Create a Node that spawns an entity in Gazebo.

    Args:
        entity_name (str): The name of the entity to spawn.
        ns_robot (str): The name space for the robot description.
        x (float, optional): X-coordinate for the entity.
        y (float, optional): Y-coordinate for the entity.
        z (float, optional): Z-coordinate for the entity.
        yaw (float, optional): Yaw angle for the entity in radians.

    Returns:
        Node: The spawn_entity node.
    """
    topic = '/robot_description'
    # new_topic = topic if ns_robot else f'/{entity_name}_description'
    new_topic = f'/{ns_robot}{topic}' if ns_robot else f'/{entity_name}_description'

    node_name = 'spawn_entity' if ns_robot else f'spawn_{entity_name}'

    arguments = ['-entity', entity_name, '-topic', new_topic, '-unpause']
    if x is not None:
        arguments += ['-x', str(x)]
    if y is not None:
        arguments += ['-y', str(y)]
    if z is not None:
        arguments += ['-z', str(z)]
    if yaw is not None:
        arguments += ['-Y', str(yaw)]

    return Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=node_name,
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


def load_controllers(controller_names: list, ns_controller_manager: str = None) -> list:
    """Generate a list of Node actions to load the specified controllers using the controller_manager.

    Args:
        controller_names (list): List of controller names to be loaded.
        ns_controller_manager (str, optional): Namespace for the controller manager. Defaults to None.

    Returns:
        list: List of Node actions for spawning controllers.
    """
    # Define controller manager namespace, handle if it's nested in another namespace
    controller_manager = f'/{ns_controller_manager}/controller_manager' if ns_controller_manager else '/controller_manager'

    # Generate and return a list of Node actions for each controller
    return [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller_name, '-c', controller_manager]
        )
        for controller_name in controller_names
    ]


def generate_robot_group(
    robot_name: str,
    robot_type: str,
    robot_parent: str,
    urdf_file: str,
    vacuum_gripper_prefix: str,
    gripper_plugin_name: str,
    x: float,
    y: float,
    z: float,
    yaw: float = None,
    use_namespace: bool = True,
    sim_gazebo: bool = True,
    simulation_controllers: str = "",
    initial_positions_file: str = "",
    controller_names: list = []
) -> GroupAction:
    """Generate a group of nodes for a single robot.

    Args:
        robot_name (str): Name of the robot.
        robot_type (str): Type of the robot (e.g., 'ur10', 'ur5').
        robot_parent (str): Parent frame for the robot.
        urdf_file (str): Path to the URDF file.
        vacuum_gripper_prefix (str): Prefix for the vacuum gripper.
        gripper_plugin_name (str): Name of the gripper plugin.
        x (float): X-coordinate of the robot.
        y (float): Y-coordinate of the robot.
        z (float): Z-coordinate of the robot.
        yaw (float, optional): Yaw angle of the robot. Defaults to None.
        sim_gazebo (bool, optional): Flag to simulate in Gazebo. Defaults to True.
        simulation_controllers (str, optional): Path to simulation controllers (YAML file). Defaults to "".
        initial_positions_file (str, optional): Path to initial positions file (YAML file). Defaults to UR description file.
        controller_names (list, optional): List of controller names to be loaded. Defaults to [].

    Returns:
        GroupAction: A group of launch actions for the robot.
    """
    # Resolve simulation controllers path if provided
    if simulation_controllers:
        simulation_controllers = os.path.join(
            get_package_share_directory(
                'hrwros_gazebo'), 'config', simulation_controllers
        )
    if initial_positions_file:
        initial_positions_file = os.path.join(
            get_package_share_directory(
                'hrwros_gazebo'), 'config', initial_positions_file
        )
    # Prepare the robot description command
    description_command = generate_description_command(
        'hrwros_support',
        urdf_file,
        f'robot_type:={robot_type} ',
        f'robot_prefix:={robot_name} ',
        f'vacuum_gripper_prefix:={vacuum_gripper_prefix} ',
        f'gripper_plugin_name:={gripper_plugin_name} ',
        f'robot_parent:={robot_parent} ',
        f'sim_gazebo:={sim_gazebo} ',
        f'simulation_controllers:={simulation_controllers} ',
        f'initial_positions_file:={initial_positions_file}',
    )

    # Namespace handling
    namespace = robot_name if use_namespace else None

    # Define actions: robot state publisher, spawner node, and controllers
    actions = [
        PushRosNamespace(namespace) if use_namespace else None,
        generate_robot_state_publisher(
            name=robot_name,
            description_command=description_command,
            ns_robot=namespace
        ),
        generate_spawner_node(
            entity_name=robot_name,
            ns_robot=namespace,
            x=x, y=y, z=z, yaw=yaw
        ),
        *load_controllers(
            controller_names,
            ns_controller_manager=namespace
        )
    ]

    # Filter out any None actions (e.g., if no namespace is used)
    actions = [action for action in actions if action is not None]

    return GroupAction(actions=actions)
