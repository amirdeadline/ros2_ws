import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import smbus
import math

class IMUNode(Node):

    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('i2c_address', 0x68)  # Declare the I2C address parameter with default value 0x68
        self.declare_parameter('sensor_offset', -20.0)
        self.i2c_address = self.get_parameter('i2c_address').get_parameter_value().integer_value
        self.sensor_offset = self.get_parameter('sensor_offset').get_parameter_value().integer_value

        self.imu_publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.heading_publisher_ = self.create_publisher(Float32, '/imu/heading', 10)  # New heading publisher
        self.timer_ = self.create_timer(0.1, self.timer_callback)

        # self.sensor_offset = -20.0  # Sensor offset correction in degrees
        
        # Initialize I2C bus (replace with actual bus number)
        self.bus = smbus.SMBus(1)  # Assuming I2C bus 1
        
        # Initialize IMU with parameterized I2C address
        self.init_imu()
        self.get_logger().info(f'IMU Node has been started with I2C address: {hex(self.i2c_address)}')

    def init_imu(self):
        # Example IMU initialization logic using parameterized I2C address
        self.bus.write_byte_data(self.i2c_address, 0x6B, 0)  # Wake up the MPU-9250
        # Add other necessary initialization commands

    def read_imu_data(self):
        # Example IMU reading logic using parameterized I2C address
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
        # Example magnetometer reading logic using parameterized I2C address
        mag_xout = self.read_word_2c(0x3B)  # Replace with actual magnetometer register addresses
        mag_yout = self.read_word_2c(0x3D)
        mag_zout = self.read_word_2c(0x3F)
        return mag_xout, mag_yout, mag_zout

    def calculate_heading(self, mag_x, mag_y):
        # Calculate heading in radians
        heading_radians = math.atan2(mag_y, mag_x)

        # Convert heading to degrees
        heading_degrees = heading_radians * (180 / math.pi)

        # Apply sensor offset correction
        corrected_heading = heading_degrees + self.sensor_offset

        # Normalize the heading to 0-360 degrees
        if corrected_heading < 0:
            corrected_heading += 360
        elif corrected_heading >= 360:
            corrected_heading -= 360

        return corrected_heading

    def read_word_2c(self, addr):
        high = self.bus.read_byte_data(self.i2c_address, addr)
        low = self.bus.read_byte_data(self.i2c_address, addr + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def timer_callback(self):
        # Read IMU data
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.read_imu_data()

        imu_data = Imu()
        imu_data.linear_acceleration.x = accel_x
        imu_data.linear_acceleration.y = accel_y
        imu_data.linear_acceleration.z = accel_z
        imu_data.angular_velocity.x = gyro_x
        imu_data.angular_velocity.y = gyro_y
        imu_data.angular_velocity.z = gyro_z

        # Publish IMU data
        self.imu_publisher_.publish(imu_data)
        self.get_logger().info(f'Published IMU data: Accel({accel_x}, {accel_y}, {accel_z}), Gyro({gyro_x}, {gyro_y}, {gyro_z})')

        # Read Magnetometer data
        mag_x, mag_y, mag_z = self.read_magnetometer_data()

        # Calculate and publish heading
        heading = self.calculate_heading(mag_x, mag_y)
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
