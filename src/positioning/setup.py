from setuptools import setup

package_name = 'positioning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 package for robot positioning',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_node = positioning.gps_node:main',
            'imu_node = positioning.imu_node:main',
            'lidar_localization_node = positioning.lidar_localization_node:main',
            'sensor_fusion_node = positioning.sensor_fusion_node:main',
            'nav2_interface_node = positioning.nav2_interface_node:main',
            'position_monitor_node = positioning.position_monitor_node:main',
            'gsm_position_node = positioning.gsm_position_node:main',
        ],
    },
)
