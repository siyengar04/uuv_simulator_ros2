from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="rexrov", description="Namespace"
    )
    world_frame_arg = DeclareLaunchArgument(
        "world_frame", default_value="world", description="World frame"
    )
    child_frame_id_arg = DeclareLaunchArgument(
        "child_frame_id",
        default_value=[LaunchConfiguration("namespace"), "/base_link"],
        description="Child frame ID",
    )
    odometry_topic_arg = DeclareLaunchArgument(
        "odometry_topic",
        default_value=[LaunchConfiguration("namespace"), "/pose_gt"],
        description="Odometry topic",
    )

    # Node to convert ground truth messages to TF
    ground_truth_to_tf = Node(
        package="uuv_assistants",
        executable="message_to_tf.cc",
        name="ground_truth_to_tf",  # Static node name
        namespace=LaunchConfiguration("namespace"),  # Dynamic namespace
        output="screen",
        parameters=[
            {"odometry_topic": LaunchConfiguration("odometry_topic")},
            {"frame_id": LaunchConfiguration("world_frame")},
            {
                "stabilized_frame_id": [
                    LaunchConfiguration("namespace"),
                    "/base_stabilized",
                ]
            },
            {
                "footprint_frame_id": [
                    LaunchConfiguration("namespace"),
                    "/base_footprint",
                ]
            },
            {"child_frame_id": LaunchConfiguration("child_frame_id")},
        ],
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(namespace_arg)
    ld.add_action(world_frame_arg)
    ld.add_action(child_frame_id_arg)
    ld.add_action(odometry_topic_arg)

    # Add nodes
    ld.add_action(ground_truth_to_tf)

    return ld
