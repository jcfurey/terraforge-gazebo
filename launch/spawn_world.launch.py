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
    # Put per-world dirs first so any tree_fuel_* wrappers bundled by
    # `terraforge generate-world` resolve from this world's own
    # models_fuel/ without the user setting GZ_SIM_RESOURCE_PATH. A
    # bringup workspace with richer mesh-backed wrappers can appear
    # earlier by exporting GZ_SIM_RESOURCE_PATH before launch — existing
    # values still take precedence via the prepend order below.
    world = LaunchConfiguration('world').perform(context)
    world_dir = os.path.dirname(os.path.abspath(world))
    models_dir = os.path.join(world_dir, 'models')
    models_fuel_dir = os.path.join(world_dir, 'models_fuel')
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    parts = [p for p in (existing, models_dir, models_fuel_dir) if p]
    combined = ':'.join(parts)
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
