add_executable(camera_node camera_node.cpp)
target_link_libraries(camera_node ${catkin_LIBRARIES})
add_dependencies(camera_node ${exploration_map_EXPORTED_TARGETS})
install(
    TARGETS camera_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
