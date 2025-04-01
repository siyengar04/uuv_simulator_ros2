#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from copy import deepcopy
import numpy as np
from tf_quaternion.transformations import euler_from_quaternion

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point32
from visualization_msgs.msg import Marker

class VehicleFootprint(Node):
    MARKER = np.array([[0, 0.75], [-0.5, -0.25], [0.5, -0.25]])

    def __init__(self):
        super().__init__('generate_vehicle_footprint')
        self.namespace = self.get_namespace().strip('/')
        self.declare_parameter('scale_footprint', 10)
        self.declare_parameter('scale_label', 10)
        self.declare_parameter('label_x_offset', 60)
        self.scale_footprint = self.get_parameter('scale_footprint').value
        self.scale_label = self.get_parameter('scale_label').value
        self.label_x_offset = self.get_parameter('label_x_offset').value

        self.get_logger().info(f'Footprint marker scale factor = {self.scale_footprint}')
        self.get_logger().info(f'Label marker scale factor = {self.scale_label}')

        self.label_marker = Marker()
        self.label_marker.header.frame_id = 'world'
        self.label_marker.header.stamp = self.get_clock().now().to_msg()
        self.label_marker.ns = self.namespace
        self.label_marker.type = Marker.TEXT_VIEW_FACING
        self.label_marker.text = self.namespace
        self.label_marker.action = Marker.ADD
        self.label_marker.pose.orientation.w = 1.0
        self.label_marker.scale.z = self.scale_label
        self.label_marker.color.a = 1.0
        self.label_marker.color.g = 1.0

        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.footprint_pub = self.create_publisher(PolygonStamped, 'footprint', 10)
        self.label_pub = self.create_publisher(Marker, 'label', 10)

    def rot(self, alpha):
        return np.array([[np.cos(alpha), -np.sin(alpha)],
                         [np.sin(alpha),  np.cos(alpha)]])

    def odometry_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = [msg.pose.pose.orientation.x,
                       msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z,
                       msg.pose.pose.orientation.w]
        yaw = euler_from_quaternion(orientation)[2]
        new_marker = self.scale_footprint * deepcopy(self.MARKER)
        points = []
        for i in range(new_marker.shape[0]):
            new_marker[i, :] = np.dot(self.rot(yaw - np.pi/2), new_marker[i, :])
            new_marker[i, 0] += x
            new_marker[i, 1] += y
            p = Point32()
            p.x = new_marker[i, 0]
            p.y = new_marker[i, 1]
            points.append(p)
        new_poly = PolygonStamped()
        new_poly.header.stamp = self.get_clock().now().to_msg()
        new_poly.header.frame_id = 'world'
        new_poly.polygon.points = points
        self.footprint_pub.publish(new_poly)

        self.label_marker.pose.position.x = x + self.label_x_offset
        self.label_marker.pose.position.y = y
        self.label_marker.pose.position.z = msg.pose.pose.position.z
        self.label_pub.publish(self.label_marker)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleFootprint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
