cmake_minimum_required(VERSION 3.0.2)
project(dynablox)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()
add_definitions(-std=c++17 -Wall -Wextra)

cs_add_library(${PROJECT_NAME}
        src/processing/preprocessing.cpp
        src/processing/clustering.cpp
        src/processing/tracking.cpp
        src/processing/ever_free_integrator.cpp
        src/evaluation/evaluator.cpp
        src/evaluation/ground_truth_handler.cpp
        src/evaluation/io_tools.cpp
        )

cs_install()
cs_export()
