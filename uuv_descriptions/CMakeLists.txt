cmake_minimum_required(VERSION 3.5)
project(uuv_descriptions)

# Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(ament_lint_auto REQUIRED)  # Now called before ament_package()

# Optional: if BUILD_TESTING, find additional dependencies
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
#  find_package(ros_testing REQUIRED)
  find_package(xacro REQUIRED)
endif()

# Install python scripts
install(PROGRAMS scripts/spawn_model
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch meshes robots urdf
        DESTINATION share/${PROJECT_NAME}
        PATTERN "*~" EXCLUDE)

if(BUILD_TESTING)
  ament_add_gtest(test_uuv_descriptions test/test_urdf_files.py)
endif()

# Call ament_package() only once at the end
ament_package()
