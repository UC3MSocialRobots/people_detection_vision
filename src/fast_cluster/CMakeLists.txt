add_executable(fast_cluster_detector.exe fast_cluster_detector.cpp )
target_link_libraries(fast_cluster_detector.exe ${catkin_LIBRARIES})
add_dependencies(fast_cluster_detector.exe ${catkin_EXPORTED_TARGETS})

add_executable(fast_cluster_selector.exe fast_cluster_selector.cpp )
target_link_libraries(fast_cluster_selector.exe ${catkin_LIBRARIES})
add_dependencies(fast_cluster_selector.exe ${catkin_EXPORTED_TARGETS})

add_executable(fast_cluster_joy_control.exe fast_cluster_joy_control.cpp )
target_link_libraries(fast_cluster_joy_control.exe ${catkin_LIBRARIES})
add_dependencies(fast_cluster_joy_control.exe ${catkin_EXPORTED_TARGETS})

add_executable(fast_dialog.exe fast_dialog.cpp)
target_link_libraries(fast_dialog.exe ${catkin_LIBRARIES})
add_dependencies(fast_dialog.exe ${catkin_EXPORTED_TARGETS})

################################################################################

add_executable(test_connected_components_matcher.exe  test_connected_components_matcher.cpp )
target_link_libraries(test_connected_components_matcher.exe ${catkin_LIBRARIES})
add_dependencies(test_connected_components_matcher.exe ${catkin_EXPORTED_TARGETS})
