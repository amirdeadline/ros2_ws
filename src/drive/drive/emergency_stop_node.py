import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')

        # Publisher for emergency stop command
        self.emergency_stop_publisher = self.create_publisher(Bool, '/emergency_stop', 10)

        # Subscribers to various event topics (replace with your specific topics)
        self.create_subscription(String, '/obstacle_detected', self.obstacle_detected_callback, 10)

        self.get_logger().info('Emergency Stop Node has started.')

    def obstacle_detected_callback(self, msg):
        # If an obstacle is detected, publish an emergency stop
        if msg.data == 'obstacle_close':
            self.get_logger().warn('Obstacle detected! Sending emergency stop.')
            self.publish_emergency_stop(True)

    def publish_emergency_stop(self, stop):
        msg = Bool()
        msg.data = stop
        self.emergency_stop_publisher.publish(msg)
        self.get_logger().info(f'Published emergency stop: {stop}')


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Emergency Stop Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
