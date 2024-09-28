from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aerator',
            executable='aerator_la_control',
            output='screen',
            name='aerator_la_control'
        ),
        Node(
            package='aerator',
            executable='aerator_ultra_sensor',
            output='screen',
            name='aerator_ultra_sensor'
        ),
        Node(
            package='aerator',
            executable='aerator_planner',
            output='screen',
            name='aerator_planner'
        )
    ])
