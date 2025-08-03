#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class AroManager(Node):
    def __init__(self):
        super().__init__('aro_manager')
        self.get_logger().info('aro_manager is now running')

        #Subscriber to crash topic
        self.crash_sub = self.create_subscription(Empty, '/aro_manager/crash', self.crash_callback, 10)

    
    def crash_callback(self, msg):
        self.get_logger().warn('Crash command received. Crashing aro_manager now...')
        raise RuntimeError("Internal crash triggered via /aro_manager/crash")

def main(args=None):
    rclpy.init(args=args)
    node = AroManager()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Node crashed: {e}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
