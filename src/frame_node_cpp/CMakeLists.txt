cmake_minimum_required(VERSION 3.5)
project(frame_node_cpp)

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

ament_auto_add_library(ndt_omp SHARED
  src/pclomp/voxel_grid_covariance_omp.cpp
  src/pclomp/ndt_omp.cpp
  src/pclomp/gicp_omp.cpp
)

target_link_libraries(ndt_omp ${PCL_LIBRARIES})

if(OpenMP_CXX_FOUND)
  target_link_libraries(ndt_omp OpenMP::OpenMP_CXX)
else()
  message(WARNING "OpenMP not found")
endif()


ament_auto_add_executable (frame_node
    src/frame_node.cpp 
    src/subscriber/cloud_subscriber.cpp 
    src/apps/pcl_viz.cpp 
    src/apps/align.cpp)


add_dependencies(frame_node
    ndt_omp
  )

target_link_libraries(frame_node ${PCL_LIBRARIES})

ament_auto_package ()