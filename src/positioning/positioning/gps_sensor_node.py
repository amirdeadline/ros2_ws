import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import pynmea2  # NMEA parser library for GPS data


class GpsSensorNode(Node):
    def __init__(self):
        super().__init__('gps_sensor_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')  # Default serial port for GPS
        self.declare_parameter('baud_rate', 9600)  # Common baud rate for GPS modules

        # Get parameter values
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value

        # Set up serial connection
        self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1)
        self.get_logger().info(f"Connected to GPS on {serial_port} at {baud_rate} baud.")

        # Publisher for GPS data
        self.gps_publisher = self.create_publisher(NavSatFix, '/gps/data', 10)

        # Timer to periodically read and parse GPS data
        self.create_timer(1.0, self.read_and_publish_gps_data)  # 1 Hz

    def read_and_publish_gps_data(self):
        try:
            if self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()

                if line.startswith('$GPGGA') or line.startswith('$GPRMC'):  # Process GGA or RMC sentences
                    msg = pynmea2.parse(line)

                    # Create a NavSatFix message
                    gps_msg = NavSatFix()
                    gps_msg.header.frame_id = "gps_frame"
                    gps_msg.header.stamp = self.get_clock().now().to_msg()

                    if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
                        gps_msg.latitude = msg.latitude
                        gps_msg.longitude = msg.longitude
                        gps_msg.altitude = msg.altitude if hasattr(msg, 'altitude') else 0.0

                        # Publish GPS data
                        self.gps_publisher.publish(gps_msg)
                        self.get_logger().info(f"Published GPS data: {gps_msg.latitude}, {gps_msg.longitude}, {gps_msg.altitude}")

        except (serial.SerialException, pynmea2.ParseError, ValueError) as e:
            self.get_logger().error(f"Error reading from serial port: {e}")


def main(args=None):
    rclpy.init(args=args)
    gps_sensor_node = GpsSensorNode()

    try:
        rclpy.spin(gps_sensor_node)
    except KeyboardInterrupt:
        gps_sensor_node.get_logger().info("Shutting down node.")
    finally:
        gps_sensor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
