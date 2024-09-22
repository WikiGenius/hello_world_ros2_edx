import os
from launch import LaunchDescription
from my_python_utils import some_utility


def generate_launch_description():
    """
    Generate the launch description for launching the TurtleBot3.
    """
    # Ensure the environment variable is set
    turtlebot_model = os.getenv('TURTLEBOT3_MODEL')
    if not turtlebot_model:
        raise EnvironmentError(
            "TURTLEBOT3_MODEL environment variable is not set. Please export TURTLEBOT3_MODEL in your bashrc.")

    robot_name = 'turtlebot'
    
    x = -4.0
    y = -0.2
    z = 0.0

    turtlebot_description = some_utility.generate_description_command(
        'turtlebot3_description', f'turtlebot3_{turtlebot_model}.urdf')
    turtlebot_state_publisher = some_utility.generate_robot_state_publisher(
        robot_name, turtlebot_description)
    turtlebot_spawner = some_utility.generate_spawner_node(
        robot_name, f'/{robot_name}_description', x, y, z)

    ld = LaunchDescription()
    ld.add_action(turtlebot_state_publisher)
    ld.add_action(turtlebot_spawner)

    return ld
