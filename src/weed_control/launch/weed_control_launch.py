from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the YAML config file
    config = os.path.join(
        get_package_share_directory('weed_control'),  # 'weed_control' is the package name
        'config',
        'wc_config.yaml'  # This is the config file
    )

    return LaunchDescription([
        Node(
            package='weed_control',
            executable='weed_control_la_control',
            output='screen',
            name='weed_control_la_control',
            parameters=[config]
        ),
        Node(
            package='weed_control',
            executable='weed_control_stepper_control',
            output='screen',
            name='weed_control_stepper_control',
            parameters=[config]  # Ensure each node is passed the config file if necessary
        ),
        Node(
            package='weed_control',
            executable='weed_control_ultra_sensor',
            output='screen',
            name='weed_control_ultra_sensor',
            parameters=[config]
        ),
        Node(
            package='weed_control',
            executable='weed_control_planner',
            output='screen',
            name='weed_control_planner',
            parameters=[config]
        ),
        Node(
            package='weed_control',
            executable='weed_control_cam',
            output='screen',
            name='weed_control_cam',
            parameters=[config]
        )
    ])
