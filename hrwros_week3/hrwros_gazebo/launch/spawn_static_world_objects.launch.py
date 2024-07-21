import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def declare_launch_arguments():
    return [
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
        DeclareLaunchArgument('bin_1', default_value='bin_1_')
    ]


def load_xacro_file(file_path, arguments, param_name):
    return Node(
        package='xacro',
        executable='xacro',
        arguments=[PathJoinSubstitution([
            FindPackageShare('hrwros_support'), 'urdf', file_path
        ])] + arguments,

        output='screen',
    )


def spawn_model(name, model, param, x=None, y=None, z=None):
    args = ['-entity', LaunchConfiguration(model), '-topic', [LaunchConfiguration(param), TextSubstitution(text='_description')]]
    if x is not None:
        args.extend(['-x', str(x)])
    if y is not None:
        args.extend(['-y', str(y)])
    if z is not None:
        args.extend(['-z', str(z)])
    return Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=[LaunchConfiguration(name), TextSubstitution(text='_spawner')],
        arguments=args,
        output='screen'
    )

def generate_launch_description():
    return LaunchDescription([
        *declare_launch_arguments(),

        # Load xacro files
        load_xacro_file(
            'workcell/workcell.xacro',
            ['workcell_parent:=', LaunchConfiguration('workcell_parent_name')],
            'workcell'
        ),
        load_xacro_file(
            'break_beam/break_beam.xacro',
            ['prefix:=', LaunchConfiguration('break_beam')],
            'break_beam'
        ),
        load_xacro_file(
            'bin/bin.xacro',
            ['prefix:=', LaunchConfiguration('bin_1')],
            'bin_1'
        ),
        load_xacro_file(
            'robot_pedestal/robot1_pedestal.xacro',
            [
                'pedestal_prefix:=', LaunchConfiguration('robot1_pedestal'),
                'pedestal_parent:=', LaunchConfiguration(
                    'workcell_parent_name'),
                'pedestal_height:=0.95'
            ],
            'robot1_pedestal'
        ),
        load_xacro_file(
            'robot_pedestal/robot2_pedestal.xacro',
            [
                'pedestal_prefix:=', LaunchConfiguration('robot2_pedestal'),
                'pedestal_parent:=', LaunchConfiguration(
                    'workcell_parent_name'),
                'pedestal_height:=0.695'
            ],
            'robot2_pedestal'
        ),

        # Spawn models
        spawn_model('break_beam', 'break_beam', 'break_beam'),
        spawn_model('bin_1', 'bin_1', 'bin_1', x=-8.0, y=-2.2),
        spawn_model('robot1_pedestal', 'robot1_pedestal', 'robot1_pedestal'),
        spawn_model('robot2_pedestal', 'robot2_pedestal', 'robot2_pedestal'),
        spawn_model('workcell', 'workcell', 'workcell')
    ])


if __name__ == '__main__':
    generate_launch_description()
