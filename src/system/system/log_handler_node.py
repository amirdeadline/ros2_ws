#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import time
from datetime import datetime, timedelta

class LogHandlerNode(Node):
    def __init__(self):
        super().__init__('log_handler')
        
        # Subscribe to the /robot_logs topic
        self.subscription = self.create_subscription(
            String, 
            '/robot_logs', 
            self.log_callback, 
            10
        )
        
        # Initialize log file details
        self.log_directory = '/home/delta/lawnbot/logs'
        os.makedirs(self.log_directory, exist_ok=True)
        self.log_file = self.create_new_log_file()

        # Create a timer to rotate the logs every 24 hours
        self.rotate_interval = timedelta(hours=24)
        self.last_rotation = datetime.now()
        self.create_timer(60, self.rotate_log_file)  # Check every 60 seconds

    def create_new_log_file(self):
        """Create a new log file with timestamp."""
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        file_path = os.path.join(self.log_directory, f'robot_log_{timestamp}.log')
        return open(file_path, 'a')

    def log_callback(self, msg):
        """Callback to write log messages to file."""
        log_entry = f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} - {msg.data}\n"
        self.log_file.write(log_entry)
        self.log_file.flush()  # Ensure it's written to disk immediately

    def rotate_log_file(self):
        """Rotate the log file every 24 hours."""
        if datetime.now() - self.last_rotation >= self.rotate_interval:
            self.log_file.close()  # Close the old log file
            self.log_file = self.create_new_log_file()  # Create a new log file
            self.last_rotation = datetime.now()
            self.get_logger().info("Log file rotated.")

    def destroy_node(self):
        """Ensure that log files are properly closed."""
        self.log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    log_handler = LogHandlerNode()

    try:
        rclpy.spin(log_handler)
    except KeyboardInterrupt:
        log_handler.get_logger().info("Shutting down log handler node.")
    finally:
        log_handler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
