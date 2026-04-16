from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            description='Absolute path to the generated .world/.sdf file.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Launch Gazebo Harmonic with the GUI client.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py',
                ])
            ),
            launch_arguments={
                'gz_args': ['-r -v4 ', world],
                'gz_version': '8',
                'on_exit_shutdown': 'true',
                'use_sim_time': 'true',
                'gui': gui,
            }.items(),
        ),
    ])
