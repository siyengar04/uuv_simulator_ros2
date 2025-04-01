#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class SetSimulationTimer(Node):
    def __init__(self):
        super().__init__('set_simulation_timer')
        self.declare_parameter('timeout', 0.0)
        self.timeout = self.get_parameter('timeout').value
        if self.timeout <= 0:
            self.get_logger().fatal('Termination time must be a positive floating point value')
            rclpy.shutdown()
        self.get_logger().info(f'Starting simulation timer - Timeout = {self.timeout} s')
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        if self.get_clock().now().nanoseconds / 1e9 >= self.timeout:
            self.get_logger().info('Simulation timeout - Killing simulation...')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SetSimulationTimer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
