cmake_minimum_required(VERSION 2.8.3)
project(generic_transform)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED)

catkin_package(INCLUDE_DIRS include)
