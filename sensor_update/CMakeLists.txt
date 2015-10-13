cmake_minimum_required(VERSION 2.8.3)
project(sensor_update)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS generic_transform)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES sensor_update)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(sensor_update src/sensor_update.cpp)
target_link_libraries(sensor_update ${catkin_LIBRARIES})
