cmake_minimum_required(VERSION 3.0.2)
project(dynablox_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()
add_definitions(-std=c++17 -Wall -Wextra)

cs_add_library(${PROJECT_NAME}
        src/visualization/motion_visualizer.cpp
        src/visualization/cloud_visualizer.cpp
        src/motion_detector.cpp
        )

cs_add_executable(motion_detector
        src/motion_detector_node.cpp
        )
target_link_libraries(motion_detector ${PROJECT_NAME})

cs_add_executable(cloud_visualizer
        src/cloud_visualizer_node.cpp
        )
target_link_libraries(cloud_visualizer ${PROJECT_NAME})

cs_install()
cs_export()
