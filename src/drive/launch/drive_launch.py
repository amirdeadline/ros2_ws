from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config = os.path.join(
        os.path.dirname(__file__),
        '..', 'config', 'drive_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='drive',
            executable='motor_controller',
            name='motor_controller',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='drive',
            executable='drive_control',
            name='drive_control',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='drive',
            executable='motor_health_monitor',
            name='motor_health_monitor',
            output='screen',
            parameters=[config]
        )
    ])
