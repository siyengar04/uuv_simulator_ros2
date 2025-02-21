cmake_minimum_required(VERSION 3.5)
project(uuv_descriptions)

## Find ament macros and libraries
find_package(ament_cmake REQUIRED)

ament_package()


# Install python scripts
install(PROGRAMS scripts/spawn_model.py
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch meshes robots urdf
        DESTINATION share/${PROJECT_NAME}
        PATTERN "*~" EXCLUDE)


