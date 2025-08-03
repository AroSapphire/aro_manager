#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import logging
import os
from datetime import datetime

class AroManager(Node):
    def __init__(self):
        super().__init__('aro_manager')
        self.logger = self._setup_file_logger()

        # File log
        self.logger.info('aro_manager is now running')
        # Console log
        self.get_logger().info('aro_manager node is now running')


        #Subscriber to crash topic
        self.crash_sub = self.create_subscription(Empty, '/aro_manager/crash', self.crash_callback, 10)

    
    def crash_callback(self, msg):
        self.get_logger().warn('Crash command received. Crashing aro_manager now...')
        raise RuntimeError("Internal crash triggered via /aro_manager/crash")
    
    def _setup_file_logger(self):
        log_dir = os.path.expanduser('~/aro_logs')
        os.makedirs(log_dir, exist_ok=True)

        # Set log filename e,g aro_manager_20250803_193500.log
        log_filename = os.path.join(
            log_dir,
            f"aro_manager_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        )

        logger = logging.getLogger('aro_manager_logger')
        logger.setLevel(logging.DEBUG)

        file_handler = logging.FileHandler(log_filename)
        file_handler.setLevel(logging.DEBUG)

        formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s')
        file_handler.setFormatter(formatter)

        logger.addHandler(file_handler)
        logger.propagate = False  # Prevent logs from showing up twice

        return logger

def main(args=None):
    rclpy.init(args=args)
    node = AroManager()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.logger.error(f"Node crashed: {e}")
        node.get_logger().error(f"Node crashed: {e}")
        raise
    finally:
        node.logger.info("aro_manager is shutting down.")
        node.get_logger().info("aro_manager is shutting down.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
