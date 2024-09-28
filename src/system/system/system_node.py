import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import logging

class SystemNode(Node):
    def __init__(self):
        super().__init__('system_node')
        self.logger = self.get_logger()
        
        # Publisher for system logs
        self.log_publisher = self.create_publisher(String, '/logs_system', 10)

        # Start publishing logs
        self.timer = self.create_timer(5, self.publish_logs)  # Every 5 seconds

    def publish_logs(self):
        # Simulating logs, you can replace this with actual logs or system events
        log_msg = String()
        log_msg.data = "System running smoothly at " + self.get_clock().now().to_msg().sec.__str__()
        self.log_publisher.publish(log_msg)
        self.logger.info(f"Log published: {log_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    system_node = SystemNode()

    try:
        rclpy.spin(system_node)
    except KeyboardInterrupt:
        system_node.get_logger().info("Shutting down system_node...")
    finally:
        system_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
