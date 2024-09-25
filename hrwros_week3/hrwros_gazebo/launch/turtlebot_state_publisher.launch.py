import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

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

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true')
    
    robot_config = some_utility.load_config_file(
        "hrwros_gazebo", "robot_config.yaml")

    turtlebot_description = some_utility.generate_description_command(
        'hrwros_support', "mobile_robot/mobile_robot.xacro",
        f"turtlebot_model:={turtlebot_model} ",
        f"robot_parent:={robot_config['robot_parent']}",

    )
    turtlebot_state_publisher = some_utility.generate_robot_state_publisher(
        turtlebot_model, turtlebot_description, use_sim_time=use_sim_time)
    

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(turtlebot_state_publisher)

    return ld
