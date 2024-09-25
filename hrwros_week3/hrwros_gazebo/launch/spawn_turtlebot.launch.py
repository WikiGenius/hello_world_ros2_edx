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

    robot_config = some_utility.load_config_file(
        "hrwros_gazebo", "robot_config.yaml")
    mobile_robot = robot_config['robot_groups']['mobile_robot']

    turtlebot_description = some_utility.generate_description_command(
        'hrwros_support', "mobile_robot/mobile_robot.xacro",
        f"turtlebot_model:={turtlebot_model} ",
        f"robot_parent:={robot_config['robot_parent']}",

    )
    turtlebot_state_publisher = some_utility.generate_robot_state_publisher(
        mobile_robot["robot_name"], turtlebot_description)
    turtlebot_spawner = some_utility.generate_spawner_node(
        mobile_robot["robot_name"], x=mobile_robot["x"], y=mobile_robot["y"], z=mobile_robot["z"])

    ld = LaunchDescription()
    ld.add_action(turtlebot_state_publisher)
    ld.add_action(turtlebot_spawner)

    return ld
