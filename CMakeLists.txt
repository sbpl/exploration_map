cmake_minimum_required(VERSION 2.8.3)
project(exploration_map)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
    COMPONENTS
        geometry_msgs
        pcl_conversions
        pcl_ros
        roscpp
        sensor_msgs
        std_msgs
        tf)

add_message_files(DIRECTORY msg FILES camera_scan.msg)

generate_messages(DEPENDENCIES sensor_msgs std_msgs)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        sensor_update
        exploration_map
    CATKIN_DEPENDS
        message_runtime
        sensor_msgs
        std_msgs
    DEPENDS)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_subdirectory(src)

install(
    DIRECTORY include/exploration_map/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
