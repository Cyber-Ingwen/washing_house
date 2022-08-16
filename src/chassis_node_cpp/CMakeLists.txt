cmake_minimum_required(VERSION 3.5)
project(chassis_node_cpp)

add_definitions(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++14")

set(CMAKE_BUILD_TYPE "RELEASE")

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(ament_cmake_auto REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(Eigen3 )
find_package(PCL REQUIRED COMPONENTS common)

###########
## Build ##
###########

include_directories(include ${rclcpp_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

ament_auto_find_build_dependencies ()

ament_auto_add_library(chassis_lib SHARED
    src/subscriber/cloud_subscriber.cpp 
    src/apps/pcl_viz.cpp
)

target_link_libraries(chassis_lib ${PCL_LIBRARIES})

if(OpenMP_CXX_FOUND)
  target_link_libraries(chassis_lib OpenMP::OpenMP_CXX)
else()
  message(WARNING "OpenMP not found")
endif()

ament_auto_add_executable (chassis_node
    src/chassis_node.cpp 
)


add_dependencies(chassis_node 
    chassis_lib
)

target_link_libraries(chassis_node ${PCL_LIBRARIES})

ament_auto_package ()