cmake_minimum_required(VERSION 3.5.1)
project(ugv_sdk VERSION 0.2.0)

# ## Additional cmake module path
# set(USER_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
# list(APPEND CMAKE_MODULE_PATH "${USER_CMAKE_PATH}/modules")
# list(APPEND CMAKE_PREFIX_PATH "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/cmake")

## Set compiler to use c++ 14 features
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Chosse build type
set(default_build_type "Release")

## Build libraries
find_package(ament_cmake_auto REQUIRED)
find_package(Threads REQUIRED)

ament_auto_find_build_dependencies ()

# add sdk main library
ament_auto_add_library(${PROJECT_NAME}
    ########################
    ## async_port from weston robot
    src/async_port/async_serial.cpp
    src/async_port/async_can.cpp
    ########################
    ## utilities
    src/utilities/protocol_detector.cpp
    ########################
    ## public interface to access robot
    src/mobile_robot/scout_robot.cpp
    src/mobile_robot/hunter_robot.cpp
    src/mobile_robot/bunker_robot.cpp
    ########################
    ## protocol v2 support
    src/protocol_v2/agilex_msg_parser_v2.c
    src/protocol_v2/protocol_v2_parser.cpp
    ########################
    ## legacy protocol v1 support (transparent to user)
    src/protocol_v1/agilex_msg_parser_v1.c
)

target_link_libraries(${PROJECT_NAME}  ${Threads_LIBRARIES})
target_compile_definitions(${PROJECT_NAME} PUBLIC ASIO_ENABLE_OLD_SERVICES)
target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/ugv_sdk/details/asio/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_compile_options(ugv_sdk PRIVATE -fpie)

ament_auto_package ()

