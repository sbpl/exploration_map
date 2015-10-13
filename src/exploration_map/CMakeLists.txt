add_library(exploration_map exploration_map.cpp exploration_map_types.cpp)
target_link_libraries(exploration_map ${catkin_LIBRARIES} sensor_update)
