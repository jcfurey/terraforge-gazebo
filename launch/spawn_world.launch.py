import os

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _resource_path(context: LaunchContext, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    models_dir = os.path.join(os.path.dirname(os.path.abspath(world)), 'models')
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    combined = ':'.join(p for p in (models_dir, existing) if p)
    return [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', combined)]


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
        OpaqueFunction(function=_resource_path),
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
                'on_exit_shutdown': 'true',
                'use_sim_time': 'true',
                'gui': gui,
            }.items(),
        ),
    ])
