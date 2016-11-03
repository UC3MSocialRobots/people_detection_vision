add_executable(bg_substractor_pplp.exe bg_substractor_pplp.cpp)
target_link_libraries(bg_substractor_pplp.exe ${catkin_LIBRARIES})
add_dependencies(bg_substractor_pplp.exe ${catkin_EXPORTED_TARGETS})

add_executable(rlpd_grabber.exe rlpd_grabber.cpp)
target_link_libraries(rlpd_grabber.exe ${catkin_LIBRARIES})
add_dependencies(rlpd_grabber.exe ${catkin_EXPORTED_TARGETS})
