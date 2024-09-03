import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32
import numpy as np

class SensorFusionNode(Node):

    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Declare subscribers for all relevant sensor data
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(Float32, '/imu/heading', self.heading_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/lidar/localization', self.lidar_callback, 10)

        # Publisher for fused position data
        self.fusion_publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/robot/position', 10)

        # Initialize data storage
        self.gps_data = None
        self.imu_data = None
        self.heading_data = None
        self.lidar_data = None

        self.get_logger().info('Sensor Fusion Node has been started.')

    def gps_callback(self, msg):
        # Store GPS data
        self.gps_data = msg
        self.get_logger().info(f'GPS data received: {msg.latitude}, {msg.longitude}, {msg.altitude}')
        self.fuse_data()

    def imu_callback(self, msg):
        # Store IMU data
        self.imu_data = msg
        self.get_logger().info(f'IMU data received: Accel({msg.linear_acceleration.x}, {msg.linear_acceleration.y}, {msg.linear_acceleration.z})')
        self.fuse_data()

    def heading_callback(self, msg):
        # Store heading data
        self.heading_data = msg.data
        self.get_logger().info(f'Heading data received: {msg.data} degrees')
        self.fuse_data()

    def lidar_callback(self, msg):
        # Store LIDAR localization data
        self.lidar_data = msg
        self.get_logger().info(f'LIDAR localization data received: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}')
        self.fuse_data()

    def fuse_data(self):
        # Ensure all sensor data is available
        if not (self.gps_data and self.imu_data and self.heading_data and self.lidar_data):
            return

        # Example fusion logic - Replace with real sensor fusion algorithm (e.g., EKF)
        fused_pose = PoseWithCovarianceStamped()
        fused_pose.header.stamp = self.get_clock().now().to_msg()
        fused_pose.header.frame_id = 'map'

        # Use LIDAR position as the base
        fused_pose.pose.pose.position.x = self.lidar_data.pose.pose.position.x
        fused_pose.pose.pose.position.y = self.lidar_data.pose.pose.position.y
        fused_pose.pose.pose.position.z = self.gps_data.altitude

        # Example: Combine IMU and LIDAR data for orientation
        # Note: This is a simple placeholder; a real application would use complex math
        fused_pose.pose.pose.orientation = self.imu_data.orientation

        # Example covariance - Replace with computed covariance matrix
        fused_pose.pose.covariance = [
            0.2, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.2, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.2, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        # Publish fused position
        self.fusion_publisher_.publish(fused_pose)
        self.get_logger().info(f'Published fused position: {fused_pose.pose.pose.position.x}, {fused_pose.pose.pose.position.y}, {fused_pose.pose.pose.position.z}')

def main(args=None):
    rclpy.init(args=args)
    sensor_fusion_node = SensorFusionNode()

    try:
        rclpy.spin(sensor_fusion_node)
    except KeyboardInterrupt:
        pass

    sensor_fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
