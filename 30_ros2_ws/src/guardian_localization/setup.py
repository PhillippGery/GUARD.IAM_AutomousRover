from setuptools import setup
import os
from glob import glob

package_name = 'guardian_localization'

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
    description='LIDAR republisher and sensor preprocessing for GUARDIAN Nav2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lidar_republisher_node = guardian_localization.lidar_republisher_node:main',
            'lidar_merger_node = guardian_localization.lidar_merger_node:main',
        ],
    },
)
