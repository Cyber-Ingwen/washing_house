cmake_minimum_required(VERSION 3.5)
project(loam)

# SET(CMAKE_BUILD_TYPE "Debug")
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(ament_cmake_auto REQUIRED)
# find_package(Ceres REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(include ${rclcpp_INCLUDE_DIRS} 
	${CERES_INCLUDE_DIRS} 
	${PCL_LIBRARIES})

ament_auto_find_build_dependencies ()

ament_auto_add_executable (point_cloud_node
    src/point_cloud_node.cpp
    src/LidarOdometry.cpp)

target_link_libraries(point_cloud_node 
	${PCL_LIBRARIES} 
	${CERES_LIBRARIES})

if(OpenMP_CXX_FOUND)
	target_link_libraries(point_cloud_node OpenMP::OpenMP_CXX)
else()
	message(WARNING "OpenMP not found")
endif()
  

ament_auto_package ()

# install(TARGETS
# point_cloud_node
#   DESTINATION lib/${PROJECT_NAME}
# )
