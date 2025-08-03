#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class AroManager(Node):
    def __init__(self):
        super().__init__('aro_manager')
        self.get_logger().info('aro_manager is now running')

def main(args=None):
    rclpy.init(args=args)
    node = AroManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
