add_executable(map_merger_node map_merger_node.cpp)
target_link_libraries(map_merger_node ${catkin_LIBRARIES} map_merger exploration_map)
install(
    TARGETS map_merger_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
