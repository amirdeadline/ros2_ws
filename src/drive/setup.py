from setuptools import setup
from setuptools import find_packages
import os
from glob import glob

package_name = 'drive'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Package for controlling the robot motors.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = drive.motor_controller_node:main',
            'drive_control = drive.drive_control_node:main',
            'motor_health_monitor = drive.motor_health_monitor_node:main',
        ],
    },
)
