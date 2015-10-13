cmake_minimum_required(VERSION 2.8.3)
project(map_merger_node)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
    COMPONENTS
        exploration_map
        geometry_msgs
        map_merger
        pcl_conversions
        pcl_ros
        roscpp
        sensor_msgs
        std_msgs
        tf)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_executable(map_merger_node src/map_merger_node.cpp)
target_link_libraries(map_merger_node ${catkin_LIBRARIES})
