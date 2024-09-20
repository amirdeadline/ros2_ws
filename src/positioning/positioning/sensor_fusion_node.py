import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
import serial
import re  # Import regular expressions module


class IMUSensorNode(Node):
    def __init__(self):
        super().__init__('imu_sensor_node')

        # Declare parameters for the serial port configuration
        self.declare_parameter('serial_port', '/dev/ttyACM0')  # Default port
        self.declare_parameter('baud_rate', 115200)  # Default baud rate

        # Get the parameter values
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # Initialize serial connection
        try:
            self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1)
            self.get_logger().info(f"Connected to IMU on {serial_port} at {baud_rate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to IMU: {e}")
            return

        # Publishers for IMU data and heading
        self.imu_data_publisher = self.create_publisher(Imu, 'imu_data', 10)
        self.heading_publisher = self.create_publisher(Float32, 'heading', 10)

        # Start a timer to read from the serial port
        self.timer = self.create_timer(0.1, self.read_serial_data)

    def read_serial_data(self):
        try:
            if self.serial_conn.in_waiting > 0:
                raw_data = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()

                # Parse the data received from the IMU
                self.parse_and_publish_data(raw_data)

        except serial.SerialException as e:
            self.get_logger().error(f"Error reading from serial port: {e}")
        except UnicodeDecodeError:
            self.get_logger().error("Error decoding data from serial port")

    def parse_and_publish_data(self, data):
        try:
            # Use regular expressions to extract only numbers (including decimal points and signs)
            parts = re.findall(r"[-+]?\d*\.\d+|\d+", data)
            if len(parts) < 13:  # Ensure all expected data parts are present
                self.get_logger().warn("Incomplete data received, skipping...")
                return

            # Extract values from data parts
            accel_x = float(parts[2])
            accel_y = float(parts[3])
            accel_z = float(parts[4])
            gyro_x = float(parts[5])
            gyro_y = float(parts[6])
            gyro_z = float(parts[7])
            mag_x = float(parts[8])
            mag_y = float(parts[9])
            mag_z = float(parts[10])
            heading = float(parts[-1])

            # Publish the heading data
            self.publish_heading(heading)

            # Create and publish IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            # Accelerometer data
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z

            # Gyroscope data
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z

            # Publish IMU data
            self.imu_data_publisher.publish(imu_msg)
            self.get_logger().info("Published IMU data.")

        except ValueError:
            self.get_logger().warn(f"Error parsing sensor data: {data}")

    def publish_heading(self, heading):
        heading_msg = Float32()
        heading_msg.data = heading
        self.heading_publisher.publish(heading_msg)
        self.get_logger().info(f"Published heading: {heading}")


def main(args=None):
    rclpy.init(args=args)
    imu_sensor_node = IMUSensorNode()
    rclpy.spin(imu_sensor_node)
    imu_sensor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
