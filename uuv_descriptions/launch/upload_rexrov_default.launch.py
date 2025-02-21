from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Declare arguments
    debug_arg = DeclareLaunchArgument(
        "debug", default_value="0", description="Debug flag"
    )

    x_arg = DeclareLaunchArgument("x", default_value="0", description="X position")

    y_arg = DeclareLaunchArgument("y", default_value="0", description="Y position")

    z_arg = DeclareLaunchArgument("z", default_value="-20", description="Z position")

    roll_arg = DeclareLaunchArgument(
        "roll", default_value="0.0", description="Roll orientation"
    )

    pitch_arg = DeclareLaunchArgument(
        "pitch", default_value="0.0", description="Pitch orientation"
    )

    yaw_arg = DeclareLaunchArgument(
        "yaw", default_value="0.0", description="Yaw orientation"
    )

    mode_arg = DeclareLaunchArgument(
        "mode", default_value="default", description="Mode"
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="rexrov", description="Namespace"
    )

    use_ned_frame_arg = DeclareLaunchArgument(
        "use_ned_frame", default_value="false", description="Use NED frame"
    )

    # Robot description based on use_ned_frame
    robot_description_ned = Node(
        package="uuv_descriptions",
        executable="spawn_model",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {
                "robot_description": PythonExpression(
                    [
                        "'$(find uuv_descriptions)/robots/rexrov_' + '",
                        LaunchConfiguration("mode"),
                        "'.xacro'",
                        " --inorder debug:=",
                        LaunchConfiguration("debug"),
                        " namespace:=",
                        LaunchConfiguration("namespace"),
                        " inertial_reference_frame:=world_ned",
                    ]
                )
            }
        ],
        condition=IfCondition(LaunchConfiguration("use_ned_frame")),
    )

    robot_description_world = Node(
        package="uuv_descriptions",
        executable="spawn_model",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {
                "robot_description": PythonExpression(
                    [
                        "'$(find uuv_descriptions)/robots/rexrov_' + '",
                        LaunchConfiguration("mode"),
                        "'.xacro'",
                        " --inorder debug:=",
                        LaunchConfiguration("debug"),
                        " namespace:=",
                        LaunchConfiguration("namespace"),
                        " inertial_reference_frame:=world",
                    ]
                )
            }
        ],
        condition=UnlessCondition(LaunchConfiguration("use_ned_frame")),
    )

    # Spawn URDF robot
    urdf_spawner = Node(
        package="uuv_descriptions",
        executable="spawn_model",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        arguments=[
            "-urdf",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-R",
            LaunchConfiguration("roll"),
            "-P",
            LaunchConfiguration("pitch"),
            "-Y",
            LaunchConfiguration("yaw"),
            "-model",
            LaunchConfiguration("namespace"),
            "-param",
            "/$(arg namespace)/robot_description",
        ],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {
                "robot_description": "/$(arg namespace)/robot_description",
                "publish_frequency": 5.0,
            }
        ],
    )

    # Include message_to_tf launch file
    message_to_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            ["$(find uuv_assistants)/launch/message_to_tf.launch.py"]
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "world_frame": "world",
            "child_frame_id": PythonExpression(
                ['"/', LaunchConfiguration("namespace"), '/base_link"']
            ),
        }.items(),
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(debug_arg)
    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)
    ld.add_action(roll_arg)
    ld.add_action(pitch_arg)
    ld.add_action(yaw_arg)
    ld.add_action(mode_arg)
    ld.add_action(namespace_arg)
    ld.add_action(use_ned_frame_arg)

    # Add nodes
    ld.add_action(robot_description_ned)
    ld.add_action(robot_description_world)
    ld.add_action(urdf_spawner)
    ld.add_action(robot_state_publisher)

    # Include other launch files
    ld.add_action(message_to_tf_launch)

    return ld
