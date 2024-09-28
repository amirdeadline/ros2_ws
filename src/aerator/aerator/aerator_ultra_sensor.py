#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class AeratorUltraSensorNode(Node):
    def __init__(self):
        super().__init__('aerator_ultra_sensor')
        self.get_logger().info("Aerator Ultra Sensor Node will be implemented here.")

def main(args=None):
    rclpy.init(args=args)

    aerator_ultra_sensor_node = AeratorUltraSensorNode()

    rclpy.spin(aerator_ultra_sensor_node)

    aerator_ultra_sensor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
