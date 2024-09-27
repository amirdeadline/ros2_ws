import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import serial
import threading


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Set up the serial connection to the Arduino via UART
        self.serial_port = serial.Serial('/dev/serial0', 9600, timeout=1)
        self.get_logger().info("Successfully connected to Arduino via UART at /dev/serial0")

        # Declare parameters for speed limits, wheel ratio, and motor ratio
        self.declare_parameter('max_speed_percentage', 20)  # Default max speed to 20% of full capacity
        self.declare_parameter('wheel_ratio', 30/22)  # Default wheel ratio (rear wheels / front wheels)
        self.declare_parameter('motor_ratio', 1.0)  # Default motor ratio (front motors / rear motors)

        # Retrieve parameter values
        self.max_speed_percentage = self.get_parameter('max_speed_percentage').get_parameter_value().integer_value
        self.wheel_ratio = self.get_parameter('wheel_ratio').get_parameter_value().double_value
        self.motor_ratio = self.get_parameter('motor_ratio').get_parameter_value().double_value

        # Publisher for Arduino console output
        self.console_publisher = self.create_publisher(String, '/arduino1_console', 10)

        # Subscribers for motor commands and emergency stop
        self.create_subscription(Twist, '/motor_commands', self.motor_command_callback, 10)
        self.create_subscription(Bool, '/emergency_stop', self.emergency_stop_callback, 10)

        self.emergency_stop = False
        self.last_command_time = self.get_clock().now()

        # Start a timer to send commands at 0.5-second intervals
        self.timer = self.create_timer(0.5, self.send_command_periodically)

        # Start a thread to read from the Arduino console
        self.console_thread = threading.Thread(target=self.read_arduino_console)
        self.console_thread.daemon = True
        self.console_thread.start()

    def motor_command_callback(self, msg):
        self.last_command_time = self.get_clock().now()  # Update last command time
        if not self.emergency_stop:
            # Convert ROS message to Arduino command format
            linear_speed = int(msg.linear.x * 100)  # Convert to percentage
            angular_speed = int(msg.angular.z * 100)  # Convert to percentage

            # Apply wheel ratio and motor ratio adjustments
            adjusted_front_speed = min(int(linear_speed * self.wheel_ratio), self.max_speed_percentage)
            adjusted_rear_speed = min(int(linear_speed / self.wheel_ratio * self.motor_ratio), self.max_speed_percentage)

            # Prepare command to send to Arduino
            command = f"M11:{adjusted_rear_speed},M12:{adjusted_rear_speed},M21:{adjusted_front_speed},M22:{adjusted_front_speed}\n"
            self.send_command_to_arduino(command)

    def emergency_stop_callback(self, msg):
        if msg.data:
            self.emergency_stop = True
            self.send_command_to_arduino("STOP\n")
        else:
            self.emergency_stop = False
            self.send_command_to_arduino("RESUME\n")

    def send_command_to_arduino(self, command):
        self.serial_port.write(command.encode())
        self.get_logger().info(f"Sent command to Arduino: {command.strip()}")

    def send_command_periodically(self):
        # Check if the last command was received within 0.5 seconds
        if (self.get_clock().now() - self.last_command_time).nanoseconds / 1e6 > 500:  # 0.5 seconds
            self.send_command_to_arduino("M11:0,M12:0,M21:0,M22:0\n")  # Stop command if no recent command

    def read_arduino_console(self):
        while rclpy.ok():
            if self.serial_port.in_waiting > 0:
                console_output = self.serial_port.readline().decode().strip()
                self.get_logger().info(f"Arduino Console: {console_output}")
                self.console_publisher.publish(String(data=console_output))

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
