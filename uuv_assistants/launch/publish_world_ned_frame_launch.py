#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_ned_frame_publisher',
        output='screen',
        arguments=[
            '0', '0', '0', '1.5707963267948966', '0', '3.141592653589793',
            'world', 'world_ned'
        ]
    )
    return LaunchDescription([node])
