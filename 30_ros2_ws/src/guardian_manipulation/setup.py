from setuptools import setup
import os
from glob import glob

package_name = 'guardian_arms'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Phillipp Gery',
    maintainer_email='phillipp@example.com',
    description='Dual SO-101 arm management and LeRobot bridge for GUARDIAN',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_manager_node = guardian_arms.arm_manager_node:main',
            'teleop_bridge_node = guardian_arms.teleop_bridge_node:main',
        ],
    },
)
