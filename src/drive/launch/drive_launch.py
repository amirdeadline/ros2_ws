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
            executable='motor_controller_node',
            name='motor_controller_node',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='drive',
            executable='drive_control_node',
            name='drive_control_node',
            output='screen',
            parameters=[config]
        )
    ])
