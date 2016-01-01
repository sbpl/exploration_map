add_executable(exploration_map_node exploration_map_node.cpp)
target_link_libraries(exploration_map_node ${catkin_LIBRARIES} sensor_update exploration_map)
add_dependencies(exploration_map_node ${exploration_map_EXPORTED_TARGETS})
install(
    TARGETS exploration_map_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
