#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class WeedControlPlannerNode(Node):
    def __init__(self):
        super().__init__('weed_control_planner')
        self.get_logger().info("Weed Control Planner Node will be implemented here.")

def main(args=None):
    rclpy.init(args=args)

    weed_control_planner_node = WeedControlPlannerNode()

    rclpy.spin(weed_control_planner_node)

    weed_control_planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
