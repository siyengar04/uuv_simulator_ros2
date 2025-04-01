#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    uuv_name_arg = DeclareLaunchArgument('uuv_name')
    
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sname_frame_publisher',
        output='screen',
        arguments=[
            '0', '0', '0', '0', '0', '3.141592653589793',
            ["/", LaunchConfiguration('uuv_name'), "/base_link"],
            ["/", LaunchConfiguration('uuv_name'), "/base_link_ned"]
        ]
    )
    
    group = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('uuv_name')),
            static_tf_node
        ]
    )
    
    return LaunchDescription([
        uuv_name_arg,
        group
    ])
