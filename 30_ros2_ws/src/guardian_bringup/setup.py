from setuptools import setup
import os
from glob import glob

package_name = 'guardian_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz') + glob('config/*.json') + glob('config/*.xml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Phillipp Gery',
    maintainer_email='phillipp@example.com',
    description='Launch files, EKF config, and Nav2 config for GUARDIAN',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
