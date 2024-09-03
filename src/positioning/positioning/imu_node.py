import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import smbus
import math
import time

class IMUNode(Node):

    def __init__(self):
        super().__init__('imu_node')
        
        # Declare parameters for I2C address and sensor offset
        self.declare_parameter('i2c_address', 0x68)  # Default I2C address for MPU-9250
        self.declare_parameter('sensor_offset', -20.0)  # Default sensor offset

        # Retrieve parameter values
        self.i2c_address = self.get_parameter('i2c_address').get_parameter_value().integer_value
        self.sensor_offset = self.get_parameter('sensor_offset').get_parameter_value().double_value

        self.imu_publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.heading_publisher_ = self.create_publisher(Float32, '/imu/heading', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        
        # Initialize I2C bus (assuming I2C bus 1)
        self.bus = smbus.SMBus(1)
        
        # Initialize IMU and magnetometer
        self.init_imu()
        self.init_magnetometer()
        self.get_logger().info(f'IMU Node has been started with I2C address: {hex(self.i2c_address)} and sensor offset: {self.sensor_offset} degrees')

    def init_imu(self):
        self.bus.write_byte_data(self.i2c_address, 0x6B, 0)  # Wake up the MPU-9250
        time.sleep(0.1)

    def init_magnetometer(self):
        # Set magnetometer to power-down mode
        self.bus.write_byte_data(0x0C, 0x0A, 0x00)
        time.sleep(0.01)
        # Set magnetometer to fuse ROM access mode to read adjustment values
        self.bus.write_byte_data(0x0C, 0x0A, 0x0F)
        time.sleep(0.01)
        # Read sensitivity adjustment values
        self.asax = self.bus.read_byte_data(0x0C, 0x10)
        self.asay = self.bus.read_byte_data(0x0C, 0x11)
        self.asaz = self.bus.read_byte_data(0x0C, 0x12)
        # Convert sensitivity adjustment values to scale
        self.asax = (self.asax - 128) * 0.5 / 128 + 1
        self.asay = (self.asay - 128) * 0.5 / 128 + 1
        self.asaz = (self.asaz - 128) * 0.5 / 128 + 1
        # Set magnetometer to continuous measurement mode (Mode 2)
        self.bus.write_byte_data(0x0C, 0x0A, 0x16)
        time.sleep(0.01)
        self.get_logger().info('Magnetometer initialized in continuous measurement mode.')

    def read_imu_data(self):
        accel_xout = self.read_word_2c(0x3B)
        accel_yout = self.read_word_2c(0x3D)
        accel_zout = self.read_word_2c(0x3F)

        gyro_xout = self.read_word_2c(0x43)
        gyro_yout = self.read_word_2c(0x45)
        gyro_zout = self.read_word_2c(0x47)

        # Convert raw values to physical units
        accel_x = accel_xout / 16384.0
        accel_y = accel_yout / 16384.0
        accel_z = accel_zout / 16384.0
        gyro_x = gyro_xout / 131.0
        gyro_y = gyro_yout / 131.0
        gyro_z = gyro_zout / 131.0
        
        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

    def read_magnetometer_data(self):
        try:
            # Read magnetometer status register to check if data is ready
            status = self.bus.read_byte_data(0x0C, 0x02)
            if status & 0x01 == 0:
                self.get_logger().warn('!!!Magnetometer data not ready')
                return None, None, None

            # Read magnetometer data registers
            mag_xout = self.read_word_2c_magnetometer(0x03)
            mag_yout = self.read_word_2c_magnetometer(0x05)
            mag_zout = self.read_word_2c_magnetometer(0x07)

            # Apply sensitivity adjustment
            mag_xout = mag_xout * self.asax * 0.6  # Scale by sensitivity factor (0.6 µT/LSB)
            mag_yout = mag_yout * self.asay * 0.6
            mag_zout = mag_zout * self.asaz * 0.6

            return mag_xout, mag_yout, mag_zout
        except Exception as e:
            self.get_logger().error(f'Failed to read magnetometer data: {e}')
            return None, None, None

    def calculate_heading(self, mag_x, mag_y):
        if mag_x is None or mag_y is None:
            self.get_logger().warn('Magnetometer data is not ready, skipping heading calculation.')
            return None

        heading_radians = math.atan2(mag_y, mag_x)
        heading_degrees = heading_radians * (180 / math.pi)
        corrected_heading = heading_degrees + self.sensor_offset

        if corrected_heading < 0:
            corrected_heading += 360
        elif corrected_heading >= 360:
            corrected_heading -= 360

        rounded_heading = float(round(corrected_heading))
        return rounded_heading

    def read_word_2c(self, addr):
        try:
            high = self.bus.read_byte_data(self.i2c_address, addr)
            low = self.bus.read_byte_data(self.i2c_address, addr + 1)
            val = (high << 8) + low
            if val >= 0x8000:
                return -((65535 - val) + 1)
            else:
                return val
        except Exception as e:
            self.get_logger().error(f'Failed to read from I2C: {e}')
            return None

    def read_word_2c_magnetometer(self, addr):
        try:
            high = self.bus.read_byte_data(0x0C, addr)
            low = self.bus.read_byte_data(0x0C, addr + 1)
            val = (high << 8) + low
            if val >= 0x8000:
                return -((65535 - val) + 1)
            else:
                return val
        except Exception as e:
            self.get_logger().error(f'Failed to read from magnetometer I2C: {e}')
            return None

    def timer_callback(self):
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.read_imu_data()

        imu_data = Imu()
        imu_data.linear_acceleration.x = accel_x
        imu_data.linear_acceleration.y = accel_y
        imu_data.linear_acceleration.z = accel_z
        imu_data.angular_velocity.x = gyro_x
        imu_data.angular_velocity.y = gyro_y
        imu_data.angular_velocity.z = gyro_z

        self.imu_publisher_.publish(imu_data)
        self.get_logger().info(f'Published IMU data: Accel({accel_x}, {accel_y}, {accel_z}), Gyro({gyro_x}, {gyro_y}, {gyro_z})')

        mag_x, mag_y, mag_z = self.read_magnetometer_data()

        heading = self.calculate_heading(mag_x, mag_y)
        if heading is not None:
            self.heading_publisher_.publish(Float32(data=heading))
            self.get_logger().info(f'Published Heading: {heading} degrees')

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()

    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass

    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
