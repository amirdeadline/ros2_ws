#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
from std_msgs.msg import String
import yaml
import os

class MowerLAControlNode(Node):
    def __init__(self):
        super().__init__('mower_la_control')

        # Load Arduino address from config file
        config_path = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'mower_config.yaml')
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        self.arduino_address = config['arduino_address']

        # Subscriber to the mower linear actuator commands topic
        self.subscription = self.create_subscription(
            String,
            '/mower_la_commands',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Mower Linear Actuator Control Node has been started.")
    
    def listener_callback(self, msg):
        command = msg.data
        if command == "mower:e":
            self.get_logger().info("Extending mower linear actuators")
            self.send_command_to_arduino("mower:e")
        elif command == "mower:r":
            self.get_logger().info("Retracting mower linear actuators")
            self.send_command_to_arduino("mower:r")
        elif command == "mower:s":
            self.get_logger().info("Stopping mower linear actuators")
            self.send_command_to_arduino("mower:s")
        else:
            self.get_logger().warn(f"Unknown command received: {command}")

    def send_command_to_arduino(self, command):
        try:
            # Use subprocess to send commands to Arduino via the console
            result = subprocess.run(
                ['echo', command, '>', self.arduino_address], shell=True)
            if result.returncode == 0:
                self.get_logger().info(f"Command sent to Arduino: {command}")
            else:
                self.get_logger().error(f"Failed to send command to Arduino: {command}")
        except Exception as e:
            self.get_logger().error(f"Error sending command to Arduino: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    mower_la_control_node = MowerLAControlNode()

    rclpy.spin(mower_la_control_node)

    mower_la_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
