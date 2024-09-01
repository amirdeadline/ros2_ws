import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.parameter import Parameter
import random

class MotorHealthMonitorNode(Node):
    def __init__(self):
        super().__init__('motor_health_monitor')

        # Declare ROS2 parameters
        self.declare_parameter('check_interval', 5)  # Interval for health checks (seconds)
        self.declare_parameter('current_threshold', 10.0)  # Maximum allowable current (Amps)
        self.declare_parameter('temperature_threshold', 80.0)  # Max temperature (Celsius)

        # Initialize parameter values
        self.check_interval = self.get_parameter('check_interval').value
        self.current_threshold = self.get_parameter('current_threshold').value
        self.temperature_threshold = self.get_parameter('temperature_threshold').value

        # Add a parameter callback for dynamic reconfigure
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Create a publisher for health status
        self.health_publisher = self.create_publisher(String, '/motor_health', 10)

        # Create a timer to periodically check motor health
        self.timer = self.create_timer(self.check_interval, self.check_motor_health)

        self.get_logger().info('Motor Health Monitor Node has started.')

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'check_interval':
                self.check_interval = param.value
                self.get_logger().info(f'Check Interval updated to: {self.check_interval}s')
                self.timer.cancel()  # Cancel the current timer
                self.timer = self.create_timer(self.check_interval, self.check_motor_health)

            elif param.name == 'current_threshold':
                self.current_threshold = param.value
                self.get_logger().info(f'Current Threshold updated to: {self.current_threshold}A')

            elif param.name == 'temperature_threshold':
                self.temperature_threshold = param.value
                self.get_logger().info(f'Temperature Threshold updated to: {self.temperature_threshold}°C')

        return rclpy.parameter.ParameterEventCallbackReturnType.SUCCESS

    def check_motor_health(self):
        # Simulate sensor readings
        motor_current = random.uniform(0, 15)
        motor_temperature = random.uniform(20, 100)

        # Check if the readings exceed the thresholds
        if motor_current > self.current_threshold:
            self.get_logger().warn(f'High motor current detected: {motor_current}A')
            self.publish_health_status(f'Warning: High motor current - {motor_current}A')

        if motor_temperature > self.temperature_threshold:
            self.get_logger().warn(f'High motor temperature detected: {motor_temperature}°C')
            self.publish_health_status(f'Warning: High motor temperature - {motor_temperature}°C')

        self.get_logger().info(f'Motor current: {motor_current}A, Temperature: {motor_temperature}°C')

    def publish_health_status(self, message):
        msg = String()
        msg.data = message
        self.health_publisher.publish(msg)
        self.get_logger().info(f'Published motor health status: {message}')
