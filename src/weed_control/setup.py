from setuptools import setup
from setuptools import find_packages
import os
from glob import glob

package_name = 'weed_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))  # If you add config files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 Weed Control package for lawn care robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'weed_control_la_control = weed_control.weed_control_la_control:main',
            'weed_control_stepper_control = weed_control.weed_control_stepper_control:main',
            'weed_control_planner = weed_control.weed_control_planner:main',
            'weed_control_ultra_sensor = weed_control.weed_control_ultra_sensor:main',
            'weed_control_cam = weed_control.weed_control_cam:main',
        ],
    },
)
