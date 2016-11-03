add_executable       (hog_pplp.exe hog_pplp.cpp)
target_link_libraries(hog_pplp.exe ${catkin_LIBRARIES})
add_dependencies     (hog_pplp.exe ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
