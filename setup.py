import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'simulation_ros2_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'set_sim_state = simulation_ros2_utils.set_sim_state:main',
            'spawn_entity = simulation_ros2_utils.spawn_entity:main',
        ],
    },
)
