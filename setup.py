# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
from setuptools import find_packages, setup

package_name = 'terraforge_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    # templates/ has no __init__.py, so find_packages() misses it; list it
    # explicitly or setuptools warns it is "importable but not distributed"
    # on every build (and newer setuptools may stop shipping the .j2 files).
    packages=find_packages(
        exclude=['test', 'test.*', 'experimental', 'experimental.*'],
    ) + ['terraforge.data_processing.templates'],
    package_data={
        'terraforge.data_processing.templates': ['*.j2'],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/spawn_world.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'click>=8.0,<9',
        'elevation>=1.1,<2',
        'Jinja2>=3.0,<4',
        'osmnx>=2.0,<3',
        'Pillow>=10.0,<13',
        'pyproj>=3.6,<4',
        'requests>=2.31,<3',
        'shapely>=2.0,<3',
    ],
    extras_require={
        'gui': ['PyQt6>=6.6,<7'],
        # geocoder + PyQt6 are needed only by the experimental interactive
        # map widget (experimental/ui/map/), which is excluded from the
        # installed package (see find_packages above). Kept out of the core
        # deps so headless CLI installs stay lean.
        'experimental': ['geocoder>=1.38,<2', 'PyQt6>=6.6,<7'],
    },
    zip_safe=True,
    maintainer='TerraForge Contributors',
    maintainer_email='dev@example.com',
    description='Generate Gazebo Harmonic simulation worlds from real-world '
                'geospatial data for ROS 2 Jazzy.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'terraforge = terraforge.cli:cli',
            'terraforge-gui = terraforge.ui.main_window:main',
        ],
    },
)
