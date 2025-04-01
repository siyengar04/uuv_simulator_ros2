#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from copy import deepcopy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point32

from tf_quaternion.transformations import euler_from_quaternion

class PublishFootprints(Node):
    def __init__(self):
        super().__init__('publish_footprints')
        self.vehicle_pub = dict()   # Dictionary for publishers, keyed by topic name.
        self.odom_sub = dict()      # Dictionary for subscriptions.
        self.marker = np.array([[0, 0.75], [-0.5, -0.25], [0.5, -0.25]])
        # Call update_vehicle_list every 10 seconds to discover new odometry topics.
        self.create_timer(10.0, self.update_vehicle_list)

    def rot(self, alpha):
        return np.array([[np.cos(alpha), -np.sin(alpha)],
                         [np.sin(alpha),  np.cos(alpha)]])

    def odometry_callback(self, msg, topic):
        # Use the topic name (or a derived robot name) as a key.
        if topic not in self.vehicle_pub:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        yaw = euler_from_quaternion(orientation)[2]
        new_marker = deepcopy(self.marker)
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
        # Publish on the publisher associated with this topic
        self.vehicle_pub[topic].publish(new_poly)

    def update_vehicle_list(self):
        # Retrieve all active topics and their types
        topics_and_types = self.get_topic_names_and_types()
        for topic, type_list in topics_and_types.items():
            # Skip system topics
            if topic.startswith('/rosout'):
                continue
            # Check if the topic is of type Odometry
            if 'nav_msgs/msg/Odometry' in type_list:
                if topic not in self.odom_sub:
                    self.get_logger().info(f"Discovered new Odometry topic: {topic}")
                    # Create a subscription to the new odometry topic
                    sub = self.create_subscription(
                        Odometry,
                        topic,
                        lambda msg, t=topic: self.odometry_callback(msg, t),
                        10)
                    self.odom_sub[topic] = sub
                    # Create a corresponding publisher for the vehicle footprint
                    pub_topic = topic + '/footprint'
                    pub = self.create_publisher(PolygonStamped, pub_topic, 10)
                    self.vehicle_pub[topic] = pub

def main(args=None):
    rclpy.init(args=args)
    node = PublishFootprints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
