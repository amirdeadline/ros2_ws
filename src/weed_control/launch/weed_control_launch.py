from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='weed_control',
            executable='weed_control_la_control',
            output='screen',
            name='weed_control_la_control'
        ),
        Node(
            package='weed_control',
            executable='weed_control_stepper_control',
            output='screen',
            name='weed_control_stepper_control'
        ),
        Node(
            package='weed_control',
            executable='weed_control_ultra_sensor',
            output='screen',
            name='weed_control_ultra_sensor'
        ),
        Node(
            package='weed_control',
            executable='weed_control_planner',
            output='screen',
            name='weed_control_planner'
        ),
        Node(
            package='weed_control',
            executable='weed_control_cam',
            output='screen',
            name='weed_control_cam'
        )
    ])
