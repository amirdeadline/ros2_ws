#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class DethatcherPlannerNode(Node):
    def __init__(self):
        super().__init__('dethatcher_planner')
        self.get_logger().info("Dethatcher Planner Node will be implemented here.")

def main(args=None):
    rclpy.init(args=args)

    dethatcher_planner_node = DethatcherPlannerNode()

    rclpy.spin(dethatcher_planner_node)

    dethatcher_planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
