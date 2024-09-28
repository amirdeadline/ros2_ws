from setuptools import setup
from setuptools import find_packages
import os
from glob import glob

package_name = 'aerator'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),  # Exclude the test folder, if present
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),  # Optional, if you have config files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 Aerator package for lawn care robot',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aerator_la_control = aerator.aerator_la_control:main',
            'aerator_ultra_sensor = aerator.aerator_ultra_sensor:main',
            'aerator_planner = aerator.aerator_planner:main',
        ],
    },
)
