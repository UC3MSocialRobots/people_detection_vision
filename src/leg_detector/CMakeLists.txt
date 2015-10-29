add_executable(leg_pplp.exe leg_pplp.cpp)
target_link_libraries(leg_pplp.exe ${catkin_LIBRARIES})
add_dependencies(leg_pplp.exe ${catkin_EXPORTED_TARGETS})

add_executable(scan_and_leg_publisher.exe scan_and_leg_publisher.cpp)
target_link_libraries(scan_and_leg_publisher.exe ${catkin_LIBRARIES})
add_dependencies(scan_and_leg_publisher.exe ${catkin_EXPORTED_TARGETS})
