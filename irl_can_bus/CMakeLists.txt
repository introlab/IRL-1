# ROS-independent lib for CAN bus management at IntRoLab.
# F. Ferland, 2014/07.

cmake_minimum_required(VERSION 2.8)
project(irl_can_bus)

#find_package(Boost REQUIRED COMPONENTS thread system)

# NOTE: These flags assume G++-4.8 is available.
set(CMAKE_CXX_FLAGS "--std=c++11 -pthread ${CMAKE_CXX_FLAGS}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ./bin)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "-DDEBUG ${CMAKE_CXX_FLAGS}")
endif(CMAKE_BUILD_TYPE STREQUAL "Debug")

include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(irl_can_bus
    src/can_manager.cpp
    src/can_robot.cpp
    src/can_tools.cpp
)

# Self-contained socket polling prototype.
add_executable(bus_proto src/bus_proto.cpp)
target_link_libraries(bus_proto irl_can_bus)

# Self-contained prototype for UniDriveV2 state..
add_executable(uni_drive_proto src/uni_drive_proto.cpp)
target_link_libraries(uni_drive_proto irl_can_bus)
