from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node for the template publisher
    template_publisher_node = Node(
        package='hrwros_week1',
        namespace='hrwros',
        executable='template_publisher_script',
        name='node_1',
        output='screen'
    )

    # Node for the template subscriber
    template_subscriber_node = Node(
        package='hrwros_week1',
        namespace='hrwros',
        executable='template_subscriber_script',
        name='node_2',
        output='screen'
    )

    return LaunchDescription([
        template_publisher_node,
        template_subscriber_node
    ])
