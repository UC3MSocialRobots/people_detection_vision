rosbuild_add_gtest(gtest_ppm
  gtest_ppm.cpp
  ../src/depth_people_detector/ppm_pplp.h)
target_link_libraries(gtest_ppm   string_utils disjoint_sets2)

rosbuild_add_gtest(gtest_face_detector_pplp
  gtest_face_detector_pplp.cpp
  ../src/face_detector/face_detector_pplp.h)
target_link_libraries(gtest_face_detector_pplp string_utils disjoint_sets2)

rosbuild_add_gtest(gtest_artoolkit_utils
  gtest_artoolkit_utils.cpp
  ../src/artoolkit/artoolkit_utils.h)
target_link_libraries(gtest_artoolkit_utils string_utils)

rosbuild_add_gtest(gtest_hog_pplp
  gtest_hog_pplp.cpp
  ../src/hog_pplp/hog_pplp.h)

rosbuild_add_gtest(gtest_nite_user_mask_pplp
  gtest_nite_user_mask_pplp.cpp
  ../src/depth_people_detector/nite_user_mask_pplp.h)

rosbuild_add_gtest(gtest_tabletop_pplp
  gtest_tabletop_pplp.cpp
  ../src/depth_people_detector/tabletop_pplp.h)
target_link_libraries(gtest_tabletop_pplp disjoint_sets2)
