from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess


def generate_launch_description():
    # Create a launch description object
    ld = LaunchDescription()

    # Define the teleop node for controlling the robot via keyboard
    teleop_node = ExecuteProcess(
        cmd=["ros2", "run", 'teleop_twist_keyboard', "teleop_twist_keyboard",
             "--ros-args", "--remap", "cmd_vel:=cmd_vel_teleop"],
        name='teleop_keyboard_node', 
        output="screen"
    )
    # Add the node to the launch description
    ld.add_action(teleop_node)

    # Return the launch description
    return ld
