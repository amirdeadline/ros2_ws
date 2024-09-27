from setuptools import setup

package_name = 'manual_control'

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
    description='Manual control of the robot using terminal input.',
    license='Your License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_control = manual_control.keyboard_control:main'
        ],
    },
)
