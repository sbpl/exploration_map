add_library(exploration_map exploration_map.cpp exploration_map_types.cpp)
target_link_libraries(exploration_map ${catkin_LIBRARIES} sensor_update)
install(
    TARGETS exploration_map
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
