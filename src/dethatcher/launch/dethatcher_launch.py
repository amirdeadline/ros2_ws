from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the YAML config file
    config = os.path.join(
        get_package_share_directory('dethatcher'),  # 'dethatcher' is the package name
        'config',
        'dethatcher_config.yaml'  # This is the config file
    )

    return LaunchDescription([
        Node(
            package='dethatcher',
            executable='dethatcher_la_control',
            output='screen',
            name='dethatcher_la_control',
            parameters=[config]  # Pass the config file to this node
        ),
        Node(
            package='dethatcher',
            executable='dethatcher_motor_control',
            output='screen',
            name='dethatcher_motor_control',
            parameters=[config]  # Pass the config file to this node
        ),
        Node(
            package='dethatcher',
            executable='dethatcher_ultra_sensor',
            output='screen',
            name='dethatcher_ultra_sensor',
            parameters=[config]
        ),
        Node(
            package='dethatcher',
            executable='dethatcher_planner',
            output='screen',
            name='dethatcher_planner',
            parameters=[config]
        )
    ])
