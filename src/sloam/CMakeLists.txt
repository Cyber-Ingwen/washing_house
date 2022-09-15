cmake_minimum_required(VERSION 3.5)
project(sloam)

add_definitions(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "RELEASE") 

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include ${rclcpp_INCLUDE_DIRS} 
                            ${CERES_INCLUDE_DIRS} 
                            ${PCL_LIBRARIES})

ament_auto_find_build_dependencies()

ament_auto_add_executable (frameFeature src/frameFeature.cpp)
ament_auto_add_executable (lidarOdometry src/lidarOdometry.cpp)
ament_auto_add_executable (FeatureExtraction src/FeatureExtraction.cpp)
ament_auto_add_executable (MapOptmization src/MapOptmization.cpp)
ament_auto_add_executable (cloud_map src/cloud_map.cpp)

target_link_libraries(FeatureExtraction ${PCL_LIBRARIES})
target_link_libraries(frameFeature ${PCL_LIBRARIES})
target_link_libraries(lidarOdometry ${PCL_LIBRARIES} ${CERES_LIBRARIES})
target_link_libraries(MapOptmization ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES})
target_link_libraries(cloud_map ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES})

ament_auto_package ()

install(TARGETS
        FeatureExtraction
        frameFeature
        lidarOdometry
        MapOptmization
        cloud_map
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})