cmake_minimum_required(VERSION 2.8.3)
project(camera_node)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
    COMPONENTS
        geometry_msgs
        message_generation
        pcl_ros
        roscpp
        rospy
        sensor_msgs
        sensor_update
        std_msgs
        tf)

add_message_files(DIRECTORY msg FILES camera_scan.msg)

generate_messages(DEPENDENCIES sensor_msgs std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime sensor_msgs std_msgs)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera_node ${catkin_LIBRARIES})
add_dependencies(camera_node ${camera_node_EXPORTED_TARGETS})
