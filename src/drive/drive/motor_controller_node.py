#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import serial
import threading


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Declare and get parameters from the namespace 'motor_controller_node'
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('max_speed_percentage', 20)
        self.declare_parameter('wheel_ratio', 1.36)
        self.declare_parameter('motor_ratio', 1.0)

        # Get parameters from the YAML file or launch arguments
        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.max_speed_percentage = self.get_parameter('max_speed_percentage').get_parameter_value().integer_value
        self.wheel_ratio = self.get_parameter('wheel_ratio').get_parameter_value().double_value
        self.motor_ratio = self.get_parameter('motor_ratio').get_parameter_value().double_value
        self.get_logger().info(f"Serial Port: {self.serial_port_name}")
        self.get_logger().info(f"Baud Rate: {self.baud_rate}")
        self.get_logger().info(f"Max Speed Percentage: {self.max_speed_percentage}")
        self.get_logger().info(f"Wheel Ratio: {self.wheel_ratio}")
        self.get_logger().info(f"Motor Ratio: {self.motor_ratio}")

        # Set up the serial connection to the Arduino via UART
        try:
            self.serial_port = serial.Serial(self.serial_port_name, self.baud_rate, timeout=1)
            self.get_logger().info(f"Successfully connected to Arduino via UART at {self.serial_port_name}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.publish_log(f"ERROR: Failed to connect to Arduino - {e}")
            return

        # Publisher for Arduino console output
        self.console_publisher = self.create_publisher(String, '/arduino1_console', 10)

        # Publisher for logs
        self.log_publisher = self.create_publisher(String, '/robot_logs', 10)

        # Subscribers for motor commands and emergency stop
        self.create_subscription(Twist, '/motor_commands', self.motor_command_callback, 10)
        self.create_subscription(Bool, '/emergency_stop', self.emergency_stop_callback, 10)

        self.emergency_stop = False
        self.last_command_time = self.get_clock().now()

        # Initialize last motor command to keep sending the latest command
        self.last_motor_command = None

        # Start a timer to send commands at 1-second intervals
        self.timer = self.create_timer(1.0, self.send_command_periodically)

        # Start a thread to read from the Arduino console
        self.console_thread = threading.Thread(target=self.read_arduino_console)
        self.console_thread.daemon = True
        self.console_thread.start()

    def publish_log(self, message):
        """Helper method to publish logs to the /robot_logs topic."""
        self.log_publisher.publish(String(data=message))

    def motor_command_callback(self, msg):
        # Save the last received motor command
        self.last_motor_command = msg
        self.last_command_time = self.get_clock().now()  # Update last command time
        self.send_motor_command()

    def emergency_stop_callback(self, msg):
        if msg.data:
            self.emergency_stop = True
            self.send_command_to_arduino("STOP\n")
        else:
            self.emergency_stop = False
            self.send_command_to_arduino("RESUME\n")

    def send_motor_command(self):
        """Send the motor command to the Arduino based on the last received message."""
        if not self.emergency_stop and self.last_motor_command:
            msg = self.last_motor_command
            linear_speed = int(msg.linear.x * 100)  # Convert to percentage
            angular_speed = int(msg.angular.z * 100)  # Convert to percentage

            if linear_speed == 0 and angular_speed != 0:
                if angular_speed > 0:
                    # Positive angular.z (turn left)
                    adjusted_front_left_speed = -angular_speed  # Motor 22 backward
                    adjusted_rear_left_speed = -angular_speed   # Motor 12 backward
                    adjusted_front_right_speed = angular_speed  # Motor 21 forward
                    adjusted_rear_right_speed = angular_speed   # Motor 11 forward
                else:
                    # Negative angular.z (turn right)
                    adjusted_front_left_speed = -angular_speed  # Motor 22 forward
                    adjusted_rear_left_speed = -angular_speed   # Motor 12 forward
                    adjusted_front_right_speed = angular_speed  # Motor 21 backward
                    adjusted_rear_right_speed = angular_speed   # Motor 11 backward
            else:
                # Normal linear movement or no rotation
                adjusted_front_left_speed = min(int(linear_speed * self.wheel_ratio), self.max_speed_percentage)  # Motor 22
                adjusted_rear_left_speed = min(int(linear_speed / self.wheel_ratio * self.motor_ratio), self.max_speed_percentage)  # Motor 12
                adjusted_front_right_speed = min(int(linear_speed * self.wheel_ratio), self.max_speed_percentage)  # Motor 21
                adjusted_rear_right_speed = min(int(linear_speed / self.wheel_ratio * self.motor_ratio), self.max_speed_percentage)  # Motor 11

            # Command format: "M11:<speed>,M12:<speed>,M21:<speed>,M22:<speed>"
            command = f"M11:{adjusted_rear_right_speed},M12:{adjusted_rear_left_speed},M21:{adjusted_front_right_speed},M22:{adjusted_front_left_speed}\n"
            self.send_command_to_arduino(command)

    def send_command_to_arduino(self, command):
        try:
            self.serial_port.write(command.encode())
            self.get_logger().info(f"Sent command to Arduino: {command.strip()}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send command to Arduino: {e}")
            self.publish_log(f"ERROR: Failed to send command to Arduino - {e}")

    def send_command_periodically(self):
        """Send a command periodically if no new commands have been received."""
        time_since_last_command = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e6
        if time_since_last_command > 1000:  # 1 second
            self.send_command_to_arduino("M11:0,M12:0,M21:0,M22:0\n")  # Stop motors if no recent command
        else:
            # Resend the last command if it's recent enough
            self.send_motor_command()

    def read_arduino_console(self):
        while rclpy.ok():
            if self.serial_port.in_waiting > 0:
                try:
                    console_output = self.serial_port.readline().decode().strip()
                    self.get_logger().info(f"Arduino Console: {console_output}")
                    self.console_publisher.publish(String(data=console_output))
                except serial.SerialException as e:
                    self.get_logger().error(f"Failed to read from Arduino: {e}")
                    self.publish_log(f"ERROR: Failed to read from Arduino - {e}")

    def destroy_node(self):
        self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    motor_controller_node = MotorControllerNode()

    try:
        rclpy.spin(motor_controller_node)
    except KeyboardInterrupt:
        motor_controller_node.get_logger().info("Shutting down node.")
    finally:
        motor_controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
