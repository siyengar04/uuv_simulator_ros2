cmake_minimum_required(VERSION 3.5)
project(uuv_assistants)

# Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(ament_lint_auto REQUIRED)  # Now called before ament_package()
find_package(rclcpp REQUIRED)

# Optional: if BUILD_TESTING, find additional dependencies
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
#  find_package(ros_testing REQUIRED)
  find_package(xacro REQUIRED)
endif()
add_executable(message_to_tf src/message_to_tf.cc)

install(TARGETS
        message_to_tf
        DESTINATION lib/${PROJECT_NAME})
# Install python scripts
install(PROGRAMS scripts/set_simulation_timer.py
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch src
        DESTINATION share/${PROJECT_NAME}
        PATTERN "*~" EXCLUDE)


# Call ament_package() only once at the end
ament_package()
