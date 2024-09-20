import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import serial


class ImuSensorNode(Node):
    def __init__(self):
        super().__init__('imu_sensor_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        # Get parameter values
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value

        # Set up serial connection
        self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1)
        self.get_logger().info(f"Connected to IMU on {serial_port} at {baud_rate} baud.")

        # Publishers
        self.heading_publisher = self.create_publisher(Float32, '/imu/heading', 10)
        self.imu_data_publisher = self.create_publisher(Float32MultiArray, '/imu/data', 10)

        # Timer to periodically read and parse data from serial
        self.create_timer(0.1, self.read_and_publish_data)  # 10 Hz

    def read_and_publish_data(self):
        try:
            if self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode('utf-8').strip()
                if line.endswith('|'):  # Check if the line ends with '|'
                    line = line[:-1]  # Remove the last '|' character
                data_parts = line.split(',')

                if len(data_parts) == 13:  # Ensure all data fields are present
                    new_imu = int(data_parts[0])
                    new_mag = int(data_parts[1])
                    accel_x = float(data_parts[2])
                    accel_y = float(data_parts[3])
                    accel_z = float(data_parts[4])
                    gyro_x = float(data_parts[5])
                    gyro_y = float(data_parts[6])
                    gyro_z = float(data_parts[7])
                    mag_x = float(data_parts[8])
                    mag_y = float(data_parts[9])
                    mag_z = float(data_parts[10])
                    temp_c = float(data_parts[11])
                    heading = float(data_parts[12])

                    # Publish IMU data
                    imu_data_msg = Float32MultiArray()
                    imu_data_msg.data = [
                        accel_x, accel_y, accel_z,
                        gyro_x, gyro_y, gyro_z,
                        mag_x, mag_y, mag_z,
                        temp_c
                    ]
                    self.imu_data_publisher.publish(imu_data_msg)
                    self.get_logger().info(f"Published IMU data: {imu_data_msg.data}")

                    # Publish heading
                    heading_msg = Float32()
                    heading_msg.data = heading
                    self.heading_publisher.publish(heading_msg)
                    self.get_logger().info(f"Published heading: {heading_msg.data}")

        except (serial.SerialException, UnicodeDecodeError, ValueError) as e:
            self.get_logger().error(f"Error reading from serial port: {e}")


def main(args=None):
    rclpy.init(args=args)
    imu_sensor_node = ImuSensorNode()

    try:
        rclpy.spin(imu_sensor_node)
    except KeyboardInterrupt:
        imu_sensor_node.get_logger().info("Shutting down node.")
    finally:
        imu_sensor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
