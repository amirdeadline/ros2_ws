#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class AeratorPlannerNode(Node):
    def __init__(self):
        super().__init__('aerator_planner')
        self.get_logger().info("Aerator Planner Node will be implemented here.")

def main(args=None):
    rclpy.init(args=args)

    aerator_planner_node = AeratorPlannerNode()

    rclpy.spin(aerator_planner_node)

    aerator_planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
