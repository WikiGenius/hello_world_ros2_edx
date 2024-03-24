from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Start the sensor info publisher node.
        Node(
            package='hrwros_week1',
            executable='sensor_info_publisher',
            name='sensor_info_publisher',
            output='screen',
        ),

        # Start the BoxHeightInformation publisher node (Assignment 1 Part 3)
        Node(
            package='hrwros_week1_assignment',
            executable='week1_assignment1_part3',
            name='get_height_box_metres',
            output='screen',
        ),

        # Start the metres_to_feet service server ROS node.
        Node(
            package='hrwros_week1',
            executable='metres_to_feet_server',
            name='metres_to_feet',
            output='screen',
        ),

        # BoxHeightInformation subscriber / convert to Feet node (Assignment 2)
        Node(
            package='hrwros_week1_assignment',
            executable='week1_assignment2',
            name='req_conversion_metres_to_feet',
            output='screen',
        ),
    ])
