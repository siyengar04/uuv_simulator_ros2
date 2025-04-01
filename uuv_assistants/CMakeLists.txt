cmake_minimum_required(VERSION 3.5)
project(uuv_assistants)

# Find ament_cmake and other dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories if needed (for example, if you add headers in an include/ folder)
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# Build the C++ executable
add_executable(uuv_message_to_tf
  src/message_to_tf.cpp
)

ament_target_dependencies(uuv_message_to_tf
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Install the C++ executable
install(TARGETS uuv_message_to_tf
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/publish_footprints.py
  scripts/set_simulation_timer.py
  scripts/publish_world_models.py
  scripts/publish_vehicle_footprint.py
  scripts/unpause_simulation.py
  scripts/create_new_robot_model
  scripts/create_thruster_manager_configuration
  DESTINATION lib/${PROJECT_NAME}
)

# Install the local Python module (tf_quaternion) so that it can be imported via:
# "from tf_quaternion.transformations import euler_from_quaternion"
install(DIRECTORY src/tf_quaternion
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install launch files (assuming they have been converted to ROS2 Python launch files)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

ament_package()
