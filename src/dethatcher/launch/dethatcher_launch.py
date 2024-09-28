from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dethatcher',
            executable='dethatcher_la_control',
            output='screen',
            name='dethatcher_la_control'
        ),
        Node(
            package='dethatcher',
            executable='dethatcher_motor_control',
            output='screen',
            name='dethatcher_motor_control'
        ),
        Node(
            package='dethatcher',
            executable='dethatcher_ultra_sensor',
            output='screen',
            name='dethatcher_ultra_sensor'
        ),
        Node(
            package='dethatcher',
            executable='dethatcher_planner',
            output='screen',
            name='dethatcher_planner'
        )
    ])
