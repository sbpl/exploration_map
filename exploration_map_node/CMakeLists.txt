cmake_minimum_required(VERSION 2.8.3)
project(exploration_map_node)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
    COMPONENTS
        camera_node
        exploration_map
        geometry_msgs
        pcl_ros
        roscpp
        rospy
        sensor_msgs
        sensor_update
        std_msgs
        tf)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_executable(exploration_map_node src/exploration_map_node.cpp)
target_link_libraries(exploration_map_node ${catkin_LIBRARIES})
