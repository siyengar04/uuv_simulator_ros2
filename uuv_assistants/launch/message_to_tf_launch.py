#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument('namespace', default_value='rexrov')
    world_frame_arg = DeclareLaunchArgument('world_frame', default_value='world')
    child_frame_id_arg = DeclareLaunchArgument('child_frame_id', default_value='/rexrov/base_link')
    odometry_topic_arg = DeclareLaunchArgument('odometry_topic', default_value='/rexrov/pose_gt')
    
    # Define the node that publishes TF from messages
    message_to_tf_node = Node(
        package='uuv_assistants',
        executable='uuv_message_to_tf',
        name=[LaunchConfiguration('namespace'), '_ground_truth_to_tf'],
        output='screen',
        parameters=[{
            'odometry_topic': LaunchConfiguration('odometry_topic'),
            'frame_id': LaunchConfiguration('world_frame'),
            'stabilized_frame_id': ["/", LaunchConfiguration('namespace'), "/base_stabilized"],
            'footprint_frame_id': ["/", LaunchConfiguration('namespace'), "/base_footprint"],
            'child_frame_id': LaunchConfiguration('child_frame_id')
        }]
    )
    
    # Group the node under the given namespace
    group = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('namespace')),
            message_to_tf_node
        ]
    )
    
    return LaunchDescription([
        namespace_arg,
        world_frame_arg,
        child_frame_id_arg,
        odometry_topic_arg,
        group
    ])
