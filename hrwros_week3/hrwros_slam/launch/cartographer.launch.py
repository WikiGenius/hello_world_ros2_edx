import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from my_python_utils import some_utility


def generate_launch_description():

    cartographer_config_dir = os.path.join(
        get_package_share_directory('hrwros_slam'), 'config')

    rviz_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution(
            [get_package_share_directory('hrwros_slam'), 'config', 'hrwros_navigation.rviz']),
        description='Full path to the RViz config file'
    )

    configuration_basename = "cartographer.lua"
    cart_node = Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node', output='screen',
                     parameters=[{'use_sim_time': True}],
                     arguments=['-configuration_directory', cartographer_config_dir,
                                '-configuration_basename', configuration_basename])

    cart_occ_grid_node = Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node', name='cartographer_occupancy_grid_node', output='screen',
                              parameters=[{'use_sim_time': True}],
                              arguments=['-resolution', '0.05',
                                         '-publish_period_sec', '1.0'])
    rviz_node = some_utility.get_rviz_node(
        LaunchConfiguration('rviz_config_file'))
    
    ld = LaunchDescription()
    
    ld.add_action(rviz_arg)
    ld.add_action(cart_node)
    ld.add_action(cart_occ_grid_node)
    ld.add_action(rviz_node)

    return ld
