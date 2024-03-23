from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    counter_delay_parameter = DeclareLaunchArgument(
        'counter_delay_parameter',
        default_value='1.0',
        description='Delay parameter for the counter with delay action server.')

    # Node for the metres_to_feet service server
    metres_to_feet_server_node = Node(
        package='hrwros_week1',
        # namespace='hrwros',
        executable='metres_to_feet_server',
        name='metres_to_feet',
        output='screen'
    )

    # Node for the counter with delay action server
    counter_with_delay_as_node = Node(
        package='hrwros_week1',
        namespace='hrwros',
        executable='counter_with_delay_as',
        name='counter_with_delay',
        output='screen',
        parameters=[{'counter_delay': LaunchConfiguration('counter_delay_parameter')}]
    )

    return LaunchDescription([
        counter_delay_parameter,
        metres_to_feet_server_node,
        counter_with_delay_as_node
    ])
