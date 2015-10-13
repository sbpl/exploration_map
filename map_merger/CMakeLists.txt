cmake_minimum_required(VERSION 2.8.3)
project(map_merger)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
    COMPONENTS
        exploration_map
        generic_transform
        geometry_msgs
        pcl_ros
        roscpp
        sensor_msgs
        std_msgs
        tf)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES map_merger
    CATKIN_DEPENDS exploration_map generic_transform)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(map_merger src/map_merger.cpp)
target_link_libraries(map_merger ${catkin_LIBRARIES})

