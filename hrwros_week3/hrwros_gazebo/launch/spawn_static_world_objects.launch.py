import os
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def declare_launch_arguments():
    return [
        # Declare the launch arguments
        DeclareLaunchArgument('workcell', default_value='workcell_'),
        DeclareLaunchArgument('workcell_parent_name',
                              default_value='world_interface'),
        DeclareLaunchArgument('robot1_prefix', default_value='robot1_'),
        DeclareLaunchArgument('robot2_prefix', default_value='robot2_'),
        DeclareLaunchArgument(
            'robot1_pedestal', default_value='robot1_pedestal_'),
        DeclareLaunchArgument(
            'robot2_pedestal', default_value='robot2_pedestal_'),
        DeclareLaunchArgument('vacuum_gripper1_prefix',
                              default_value='vacuum_gripper1_'),
        DeclareLaunchArgument('vacuum_gripper2_prefix',
                              default_value='vacuum_gripper2_'),
        DeclareLaunchArgument('break_beam', default_value='break_beam_'),
        DeclareLaunchArgument('bin_1', default_value='bin_1_'),
    ]


def load_xacro_file(file_path, arguments):
    return ExecuteProcess(
        cmd=[
            'xacro',
            PathJoinSubstitution([
                FindPackageShare('hrwros_support'),
                'urdf', file_path
            ]),
            arguments
        ],
        output='screen',
        shell=True
    ),


def generate_launch_description():
    return LaunchDescription([

        *declare_launch_arguments(),
        # Execute the xacro command
        # Load xacro files
        load_xacro_file(
            'workcell/workcell.xacro',
            ['workcell_parent:=', LaunchConfiguration('workcell_parent_name')],
        ),
        load_xacro_file(
            'break_beam/break_beam.xacro',
            ['prefix:=', LaunchConfiguration('break_beam')],
        ),
        load_xacro_file(
            'bin/bin.xacro',
            ['prefix:=', LaunchConfiguration('bin_1')],
        ),
        # load_xacro_file(
        #     'robot_pedestal/robot1_pedestal.xacro',
        #     [
        #         'pedestal_prefix:=', LaunchConfiguration('robot1_pedestal'),
        #         'pedestal_parent:=', LaunchConfiguration(
        #             'workcell_parent_name'),
        #         'pedestal_height:=0.95'
        #     ],
        # ),
        # load_xacro_file(
        #     'robot_pedestal/robot2_pedestal.xacro',
        #     [
        #         'pedestal_prefix:=', LaunchConfiguration('robot2_pedestal'),
        #         'pedestal_parent:=', LaunchConfiguration(
        #             'workcell_parent_name'),
        #         'pedestal_height:=0.695'
        #     ],
        # ),
    ])





if __name__ == '__main__':
    generate_launch_description()
