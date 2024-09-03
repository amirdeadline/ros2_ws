import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
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

        # Subscribe to temperature sensors and current sensors
        self.create_subscription(Float32, '/ts11', self.temperature_callback, 10)
        self.create_subscription(Float32, '/ts12', self.temperature_callback, 10)
        self.create_subscription(Float32, '/ts21', self.temperature_callback, 10)
        self.create_subscription(Float32, '/ts22', self.temperature_callback, 10)
        self.create_subscription(Float32, '/current_m11', self.current_callback, 10)
        self.create_subscription(Float32, '/current_m12', self.current_callback, 10)
        self.create_subscription(Float32, '/current_m21', self.current_callback, 10)
        self.create_subscription(Float32, '/current_m22', self.current_callback, 10)

        # Dictionary to hold sensor readings
        self.temperatures = {'ts11': 0.0, 'ts12': 0.0, 'ts21': 0.0, 'ts22': 0.0}
        self.currents = {'current_m11': 0.0, 'current_m12': 0.0, 'current_m21': 0.0, 'current_m22': 0.0}

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

    def temperature_callback(self, msg, sensor_name):
        # Update the temperature reading for the sensor
        self.temperatures[sensor_name] = msg.data

    def current_callback(self, msg, current_name):
        # Update the current reading for the motor
        self.currents[current_name] = msg.data

    def check_motor_health(self):
        # Check each motor's current against the threshold
        for motor, current in self.currents.items():
            if current > self.current_threshold:
                self.get_logger().warn(f'High current detected on {motor}: {current}A')
                self.publish_health_status(f'Warning: High current on {motor} - {current}A')

        # Check each motor's temperature against the threshold
        for sensor, temperature in self.temperatures.items():
            if temperature > self.temperature_threshold:
                self.get_logger().warn(f'High temperature detected on {sensor}: {temperature}°C')
                self.publish_health_status(f'Warning: High temperature on {sensor} - {temperature}°C')

        # Log current and temperature data for each motor
        self.get_logger().info(f'Motor currents: {self.currents}, Temperatures: {self.temperatures}')

    def publish_health_status(self, message):
        msg = String()
        msg.data = message
        self.health_publisher.publish(msg)
        self.get_logger().info(f'Published motor health status: {message}')


def main(args=None):
    rclpy.init(args=args)
    motor_health_monitor_node = MotorHealthMonitorNode()

    try:
        rclpy.spin(motor_health_monitor_node)
    except KeyboardInterrupt:
        motor_health_monitor_node.get_logger().info("Shutting down node.")
    finally:
        motor_health_monitor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
