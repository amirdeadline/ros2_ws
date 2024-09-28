from setuptools import setup
from setuptools import find_packages
import os
from glob import glob

package_name = 'mower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 Mower package for lawn care robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mower_la_control = mower.mower_la_control:main',
            'mower_motor_control = mower.mower_motor_control:main',
            'mower_planner = mower.mower_planner:main',
            'mower_ultra_sensor = mower.mower_ultra_sensor:main',
        ],
    },
)
