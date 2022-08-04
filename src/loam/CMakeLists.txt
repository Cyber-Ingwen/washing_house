cmake_minimum_required(VERSION 3.5)
project(loam)

SET(CMAKE_BUILD_TYPE "Debug")

find_package(ament_cmake_auto REQUIRED)

include_directories(include ${rclcpp_INCLUDE_DIRS})

ament_auto_find_build_dependencies ()

ament_auto_add_executable (point_cloud_node
    src/point_cloud_node.cpp)

target_link_libraries(point_cloud_node ${PCL_LIBRARIES})

ament_auto_package ()