#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class DethatcherUltraSensorNode(Node):
    def __init__(self):
        super().__init__('dethatcher_ultra_sensor')
        self.get_logger().info("Dethatcher Ultrasonic Sensor Node will be implemented here.")

def main(args=None):
    rclpy.init(args=args)

    dethatcher_ultra_sensor_node = DethatcherUltraSensorNode()

    rclpy.spin(dethatcher_ultra_sensor_node)

    dethatcher_ultra_sensor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
