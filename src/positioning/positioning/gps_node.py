import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

class GPSNode(Node):

    def __init__(self):
        super().__init__('gps_node')
        self.gps_publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.quality_publisher_ = self.create_publisher(String, '/gps/quality', 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        
        # Initialize any other required settings or configurations for GPS module
        self.get_logger().info('GPS Node has been started.')

    def timer_callback(self):
        # Simulate reading GPS data (replace this with actual GPS module reading logic)
        gps_data = NavSatFix()
        gps_data.latitude = 37.7749    # Example data: replace with actual GPS readings
        gps_data.longitude = -122.4194
        gps_data.altitude = 30.0

        signal_quality = "Good"  # Example signal quality: replace with actual signal status

        # Publish GPS data
        self.gps_publisher_.publish(gps_data)
        self.get_logger().info(f'Published GPS data: {gps_data.latitude}, {gps_data.longitude}, {gps_data.altitude}')

        # Publish GPS signal quality
        self.quality_publisher_.publish(String(data=signal_quality))
        self.get_logger().info(f'Published GPS quality: {signal_quality}')

def main(args=None):
    rclpy.init(args=args)
    gps_node = GPSNode()

    try:
        rclpy.spin(gps_node)
    except KeyboardInterrupt:
        pass

    gps_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
