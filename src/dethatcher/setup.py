from setuptools import setup
from setuptools import find_packages
import os
from glob import glob

package_name = 'dethatcher'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 Dethatcher package for lawn care robot',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dethatcher_la_control = dethatcher.dethatcher_la_control:main',
            'dethatcher_motor_control = dethatcher.dethatcher_motor_control:main',
            'dethatcher_ultra_sensor = dethatcher.dethatcher_ultra_sensor:main',
            'dethatcher_planner = dethatcher.dethatcher_planner:main',
        ],
    },
)
