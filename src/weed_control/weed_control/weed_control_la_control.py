#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
from std_msgs.msg import String

class WeedControlLAControlNode(Node):
    def __init__(self):
        super().__init__('weed_control_la_control')

        # Declare the 'serial_port' parameter with a default value
        self.declare_parameter('serial_port', '/dev/ttyUSB0')

        # Get the value of the 'serial_port' parameter
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.get_logger().info(f"Serial Port: {self.serial_port}")

        # Subscriber to the weed control linear actuator commands topic
        self.subscription = self.create_subscription(
            String,
            '/weed_control_la_commands',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Weed Control Linear Actuator Control Node has been started.")
    
    def listener_callback(self, msg):
        command = msg.data
        if command == "wc:e":
            self.get_logger().info("Extending weed control linear actuators")
            self.send_command_to_arduino("weed_control:e")
        elif command == "wc:r":
            self.get_logger().info("Retracting weed control linear actuators")
            self.send_command_to_arduino("weed_control:r")
        elif command == "wc:s":
            self.get_logger().info("Stopping weed control linear actuators")
            self.send_command_to_arduino("weed_control:s")
        else:
            self.get_logger().warn(f"Unknown command received: {command}")

    def send_command_to_arduino(self, command):
        try:
            # Use subprocess to send commands to Arduino via the serial port
            result = subprocess.run(['echo', command, '>', self.serial_port], shell=True)
            if result.returncode == 0:
                self.get_logger().info(f"Command sent to Arduino: {command}")
            else:
                self.get_logger().error(f"Failed to send command to Arduino: {command}")
        except Exception as e:
            self.get_logger().error(f"Error sending command to Arduino: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    weed_control_la_control_node = WeedControlLAControlNode()

    rclpy.spin(weed_control_la_control_node)

    weed_control_la_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
