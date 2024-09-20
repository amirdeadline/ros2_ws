from setuptools import setup

package_name = 'positioning'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='delta',
    maintainer_email='delta@example.com',
    description='Positioning package for ROS2 using IMU, GPS, GSM, and 3D depth camera data',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_sensor_node = positioning.imu_sensor_node:main',
            'gps_sensor_node = positioning.gps_sensor_node:main',
            'camera_odometry_node = positioning.camera_odometry_node:main',
            'position_estimator_node = positioning.position_estimator_node:main',
            'sensor_fusion_node = positioning.sensor_fusion_node:main',  # New node for sensor fusion
        ],
    },
)
