from setuptools import find_packages, setup

package_name = 'terraforge_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    package_data={
        'terraforge.data_processing': ['templates/*.j2'],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/spawn_world.launch.py']),
    ],
    install_requires=['setuptools'],
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
