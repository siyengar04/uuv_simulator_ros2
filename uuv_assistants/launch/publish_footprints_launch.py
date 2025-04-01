#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package='uuv_assistants',
        executable='publish_footprints.py',
        name='publish_footprints',
        output='screen'
    )
    return LaunchDescription([node])
