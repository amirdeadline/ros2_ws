#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MowerPlannerNode(Node):
    def __init__(self):
        super().__init__('mower_planner')
        self.get_logger().info("Mower Planner Node will be implemented here.")

def main(args=None):
    rclpy.init(args=args)

    mower_planner_node = MowerPlannerNode()

    rclpy.spin(mower_planner_node)

    mower_planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
