from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mower',
            executable='mower_la_control',
            output='screen',
            name='mower_la_control'
        ),
        Node(
            package='mower',
            executable='mower_motor_control',
            output='screen',
            name='mower_motor_control'
        ),
        Node(
            package='mower',
            executable='mower_ultra_sensor',
            output='screen',
            name='mower_ultra_sensor'
        ),
        Node(
            package='mower',
            executable='mower_planner',
            output='screen',
            name='mower_planner'
        )
    ])
