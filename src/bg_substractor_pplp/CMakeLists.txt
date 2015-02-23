add_executable(bg_substractor_pplp.exe bg_substractor_pplp.cpp)
target_link_libraries(bg_substractor_pplp.exe disjoint_sets2 ${catkin_LIBRARIES})

add_executable(rlpd_grabber.exe rlpd_grabber.cpp)
target_link_libraries(rlpd_grabber.exe disjoint_sets2 ${catkin_LIBRARIES})
