add_library(map_merger map_merger.cpp)
target_link_libraries(map_merger ${catkin_LIBRARIES} exploration_map)
install(
    TARGETS map_merger
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
