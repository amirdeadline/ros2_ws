#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter


class DriveControlNode(Node):
    def __init__(self):
        super().__init__('drive_control')
        self.publisher = self.create_publisher(Twist, '/motor_commands', 10)

        # Define default parameters
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.0)

        # Publish the initial drive command
        self.publish_drive_command()

    def publish_drive_command(self):
        # Get the parameters
        linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value

        # Create a new Twist message
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed

        # Publish the drive command
        self.publisher.publish(twist)
        self.get_logger().info(f"Sent drive command: linear={linear_speed} angular={angular_speed}")


def main(args=None):
    rclpy.init(args=args)
    drive_control_node = DriveControlNode()
    rclpy.spin(drive_control_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
