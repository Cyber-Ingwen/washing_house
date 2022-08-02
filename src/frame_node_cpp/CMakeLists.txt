cmake_minimum_required(VERSION 3.5)
project(frame_node_cpp)

find_package(ament_cmake_auto REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(Eigen3 )
# find_package(PCL REQUIRED COMPONENTS common)

include_directories(include ${rclcpp_INCLUDE_DIRS})

ament_auto_find_build_dependencies ()

ament_auto_add_executable (frame_node
    src/frame_node.cpp 
    src/subscriber/cloud_subscriber.cpp
    src/apps/pcl_viz.cpp)

target_link_libraries(frame_node ${PCL_LIBRARIES})

ament_auto_package ()