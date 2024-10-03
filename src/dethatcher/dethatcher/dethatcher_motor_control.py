#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class DethatcherMotorControlNode(Node):
    def __init__(self):
        super().__init__('dethatcher_motor_control')

        # Declare parameters for serial port and baud rate
        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 9600)

        # Get parameter values
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # Log the parameter values
        self.get_logger().info(f"Serial Port: {self.serial_port}")
        self.get_logger().info(f"Baud Rate: {self.baud_rate}")

        # Initialize serial communication with the Arduino
        try:
            self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info("Successfully connected to Arduino via serial port.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            return

        # Subscriber to the dethatcher motor commands topic
        self.subscription = self.create_subscription(
            String,
            '/dethatcher_motor_commands',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.get_logger().info("Dethatcher Motor Control Node has been started.")
    
    def listener_callback(self, msg):
        command = msg.data
        if command == "start":
            self.get_logger().info("Starting dethatcher motor")
            self.send_command_to_arduino("1:950")  # Send start command to Arduino (Motor 1 signal 950)
        elif command == "stop":
            self.get_logger().info("Stopping dethatcher motor")
            self.send_command_to_arduino("1:800")  # Send stop command to Arduino (Motor 1 signal 800)
        else:
            self.get_logger().warn(f"Unknown command received: {command}")

    def send_command_to_arduino(self, command):
        try:
            # Send the command to the Arduino via serial connection
            self.serial_connection.write(f"{command}\n".encode())
            self.get_logger().info(f"Sent command to Arduino: {command}")

            # Optionally read the response from the Arduino
            response = self.serial_connection.readline().decode().strip()
            if response:
                self.get_logger().info(f"Arduino response: {response}")

        except serial.SerialException as e:
            self.get_logger().error(f"Error sending command to Arduino: {str(e)}")

    def destroy(self):
        if self.serial_connection.is_open:
            self.serial_connection.close()
        self.get_logger().info("Serial connection closed.")


def main(args=None):
    rclpy.init(args=args)

    dethatcher_motor_control_node = DethatcherMotorControlNode()

    rclpy.spin(dethatcher_motor_control_node)

    dethatcher_motor_control_node.destroy_node()
    rclpy.shutdown()
    dethatcher_motor_control_node.destroy()


if __name__ == '__main__':
    main()
