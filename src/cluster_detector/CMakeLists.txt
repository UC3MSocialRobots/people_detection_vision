add_library(multi_modal cloud_filter.cpp)
target_link_libraries(multi_modal OpenNI pcl_kdtree pcl_search ${OpenCV_LIBS} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(floor_removal.exe floor_removal.cpp)
target_link_libraries(floor_removal.exe ${catkin_LIBRARIES})

add_executable(cluster_detector.exe cluster_detector.cpp)
target_link_libraries(cluster_detector.exe pcl_kdtree pcl_search ${catkin_LIBRARIES})

install(TARGETS multi_modal
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
