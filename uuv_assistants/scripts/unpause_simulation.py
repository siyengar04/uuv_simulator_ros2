#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import time
import sys

class UnpauseSimulation(Node):
    def __init__(self):
        super().__init__('unpause_simulation')
        self.declare_parameter('timeout', 0.0)
        self.timeout = self.get_parameter('timeout').value
        if self.timeout <= 0:
            self.get_logger().fatal('Unpause time must be a positive floating point value')
            rclpy.shutdown()
        self.get_logger().info(f'Unpause simulation - Time = {self.timeout} s')

    def unpause(self):
        start_time = time.time()
        while time.time() - start_time < self.timeout:
            time.sleep(0.1)
        client = self.create_client(Empty, '/unpause_physics')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/unpause_physics service is unavailable')
            return
        req = Empty.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Simulation unpaused...')

def main(args=None):
    rclpy.init(args=args)
    node = UnpauseSimulation()
    node.unpause()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
