cmake_minimum_required(VERSION 3.5)
project(hunter_base)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(hunter_msgs REQUIRED)
find_package(ugv_sdk REQUIRED)
add_subdirectory(ascent)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include
    ${ugv_sdk_INCLUDE_DIRS}
)

ament_auto_find_build_dependencies ()

set(DEPENDENCIES
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "std_msgs"
  "tf2"
  "tf2_ros"
  "hunter_msgs"
  "sensor_msgs"
)

ament_auto_add_executable(hunter_base_node 
    src/hunter_base_ros.cpp
    src/hunter_base_node.cpp
    src/bicycle_model.cpp)

target_link_libraries(hunter_base_node 
    ascent 
    ${dependencies})

# ament_target_dependencies(hunter_base_node rclcpp tf2 tf2_ros std_msgs nav_msgs sensor_msgs hunter_msgs ${dependencies})

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
  )

ament_auto_package ()
