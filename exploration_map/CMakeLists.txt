cmake_minimum_required(VERSION 2.8.3)
project(exploration_map)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
    COMPONENTS
        generic_transform 
        pcl_conversions 
        sensor_update)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES exploration_map
    CATKIN_DEPENDS generic_transform sensor_update)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(exploration_map src/exploration_map.cpp src/exploration_map_types.cpp)
target_link_libraries(exploration_map ${catkin_LIBRARIES})
