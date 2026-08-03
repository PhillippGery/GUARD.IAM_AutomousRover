from setuptools import setup
import os
from glob import glob

package_name = 'guardian_navigation'

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
    description='High-level autonomous navigation behaviors for GUARDIAN',
    license='MIT',
    entry_points={
        'console_scripts': [
            'demo_mission_node = guardian_navigation.demo_mission_node:main',
            'localization_bootstrap_node = guardian_navigation.localization_bootstrap_node:main',
            'set_waypoint_node = guardian_navigation.set_waypoint_node:main',
        ],
    },
)
