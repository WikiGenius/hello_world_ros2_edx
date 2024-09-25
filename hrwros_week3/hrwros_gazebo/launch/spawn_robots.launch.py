from launch import LaunchDescription
from my_python_utils import some_utility


def generate_launch_description():
    """Generate the launch description for the robots."""
    robot_config = some_utility.load_config_file(
        "hrwros_gazebo", "robot_config.yaml")
    robot1 = robot_config['robot_groups']['robot1']
    robot2 = robot_config['robot_groups']['robot2']

    # Group for Robot 1
    robot_group1 = some_utility.generate_robot_group(
        robot_name=robot1['robot_name'],
        robot_type=robot1['robot_type'],
        robot_parent=robot_config['robot_parent'],
        urdf_file=robot1['urdf_file'],
        vacuum_gripper_prefix=robot1['vacuum_gripper_prefix'],
        gripper_plugin_name=robot1['gripper_plugin_name'],
        x=robot1['x'], y=robot1['y'], z=robot1['z'],
        simulation_controllers=robot1['simulation_controllers'],
        initial_positions_file=robot1['initial_positions_file'],
        controller_names=robot1['controller_names']
    )

    # Group for Robot 2
    robot_group2 = some_utility.generate_robot_group(
        robot_name=robot2['robot_name'],
        robot_type=robot2['robot_type'],
        robot_parent=robot_config['robot_parent'],
        urdf_file=robot2['urdf_file'],
        vacuum_gripper_prefix=robot2['vacuum_gripper_prefix'],
        gripper_plugin_name=robot2['gripper_plugin_name'],
        x=robot2['x'], y=robot2['y'], z=robot2['z'], yaw=robot2['yaw'],
        simulation_controllers=robot2['simulation_controllers'],
        initial_positions_file=robot2['initial_positions_file'],
        controller_names=robot2['controller_names']
    )

    ld = LaunchDescription()

    ld.add_action(robot_group1)
    ld.add_action(robot_group2)

    return ld
