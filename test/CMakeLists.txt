catkin_add_gtest(gtest_ppm gtest_ppm.cpp)
target_link_libraries(gtest_ppm  ${catkin_LIBRARIES})

catkin_add_gtest(gtest_face_detector_pplp gtest_face_detector_pplp.cpp)
target_link_libraries(gtest_face_detector_pplp  ${catkin_LIBRARIES})

catkin_add_gtest(gtest_artoolkit_utils gtest_artoolkit_utils.cpp)
target_link_libraries(gtest_artoolkit_utils  ${catkin_LIBRARIES})
add_dependencies(gtest_artoolkit_utils
  ${ar_pose_EXPORTED_TARGETS}
  ${people_msgs_EXPORTED_TARGETS})

catkin_add_gtest(gtest_hog_pplp gtest_hog_pplp.cpp)
target_link_libraries(gtest_hog_pplp  ${catkin_LIBRARIES})

catkin_add_gtest(gtest_nite_user_mask_pplp gtest_nite_user_mask_pplp.cpp)
target_link_libraries(gtest_nite_user_mask_pplp  ${catkin_LIBRARIES})

catkin_add_gtest(gtest_tabletop_pplp  gtest_tabletop_pplp.cpp)
target_link_libraries(gtest_tabletop_pplp  ${catkin_LIBRARIES})
