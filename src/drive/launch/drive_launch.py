from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the YAML config file
    config = os.path.join(
        get_package_share_directory('drive'),  # 'drive' is your package name
        'config',
        'drive_config.yaml'
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
