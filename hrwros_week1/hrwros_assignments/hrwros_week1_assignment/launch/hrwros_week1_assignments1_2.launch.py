from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the sensor info publisher node.
        Node(
            package='hrwros_week1',
            executable='<add-node-type>',
            name='<add-node-name>',
            output='screen',
        ),

        # Start the BoxHeightInformation publisher node (Assignment 1 Part 3)
        Node(
            package='hrwros_week1_assignment',
            executable='<add-node-type>',
            name='<add-node-name>',
            output='screen',
        ),

        # Start the metres_to_feet service server ROS node.
        Node(
            package='hrwros_week1',
            executable='metres_to_feet_server.py',
            name='metres_to_feet',
            output='screen',
        ),

        # BoxHeightInformation subscriber / convert to Feet node (Assignment 2)
        Node(
            package='hrwros_week1_assignment',
            executable='<add-node-type>',
            name='<add-node-name>',
            output='screen',
        ),
    ])
