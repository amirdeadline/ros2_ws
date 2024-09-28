#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MowerUltraSensorNode(Node):
    def __init__(self):
        super().__init__('mower_ultra_sensor')
        self.get_logger().info("Mower Ultrasonic Sensor Node will be implemented here.")

def main(args=None):
    rclpy.init(args=args)

    mower_ultra_sensor_node = MowerUltraSensorNode()

    rclpy.spin(mower_ultra_sensor_node)

    mower_ultra_sensor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
