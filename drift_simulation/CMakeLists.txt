cmake_minimum_required(VERSION 3.0.2)
project(drift_simulation)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()
add_definitions(-std=c++14 -Wall -Wextra)

cs_add_library(${PROJECT_NAME}
        src/normal_distribution.cpp
        src/odometry_drift_simulator.cpp
        src/read_simulated_drift.cpp
        )

cs_add_executable(odometry_drift_simulator
        src/odometry_drift_simulator_node.cpp
        )
target_link_libraries(odometry_drift_simulator ${PROJECT_NAME})

cs_add_executable(drift_reader
        src/read_simulated_drift_node.cpp
        )
target_link_libraries(drift_reader ${PROJECT_NAME})

cs_install()
cs_export()
