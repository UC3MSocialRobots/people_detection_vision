add_executable(hog_pplp.exe hog_pplp.cpp)
target_link_libraries(hog_pplp.exe disjoint_sets2 ${catkin_LIBRARIES})
add_dependencies(hog_pplp.exe ${catkin_EXPORTED_TARGETS})
