add_executable(face_detector_pplp.exe face_detector_pplp.cpp)
target_link_libraries(face_detector_pplp.exe disjoint_sets2)

add_executable(face_detector2d_pplp.exe face_detector2d_pplp.cpp)
target_link_libraries(face_detector2d_pplp.exe ${catkin_LIBRARIES})

add_executable(fake_face_detector_pplp.exe fake_face_detector_pplp.cpp)
target_link_libraries(fake_face_detector_pplp.exe string_utils ${catkin_LIBRARIES})
