#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class WeedControlUltraSensorNode(Node):
    def __init__(self):
        super().__init__('weed_control_ultra_sensor')
        self.get_logger().info("Weed Control Ultrasonic Sensor Node will be implemented here.")

def main(args=None):
    rclpy.init(args=args)

    weed_control_ultra_sensor_node = WeedControlUltraSensorNode()

    rclpy.spin(weed_control_ultra_sensor_node)

    weed_control_ultra_sensor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
