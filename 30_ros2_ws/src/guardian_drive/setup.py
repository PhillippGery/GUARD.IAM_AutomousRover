from setuptools import setup
import os
from glob import glob

package_name = 'guardian_drive'

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
    description='Mecanum kinematics and Arduino serial bridge for GUARDIAN',
    license='MIT',
    entry_points={
        'console_scripts': [
            'guardian_drive_node = guardian_drive.serial_bridge_node:main',
            'mecanum_kinematics_node = guardian_drive.mecanum_kinematics_node:main',
            'serial_bridge_node = guardian_drive.serial_bridge_node:main',
            'dummy_odom_node = guardian_drive.dummy_odom_node:main',
            'phidget_bridge_node = guardian_drive.phidget_bridge_node:main',
        ],
    },
)
