cmake_minimum_required(VERSION 3.0.2)
project(lidar_motion_detection)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()
add_definitions(-std=c++17 -Wall -Wextra)

cs_add_library(${PROJECT_NAME}
        src/preprocessing.cpp
        src/motion_detector.cpp
        src/ground_truth_handler.cpp
        src/visualization_utils.cpp
        src/motion_visualizer.cpp
        src/ever_free_integrator.cpp
        src/evaluator.cpp
        src/clustering.cpp
        )

message(STATUS ${catkin_LIBRARIES})

cs_add_executable(motion_detector
        src/motion_detector_node.cpp
        )
target_link_libraries(motion_detector ${PROJECT_NAME})

cs_install()
cs_export()
