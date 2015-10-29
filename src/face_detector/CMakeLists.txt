add_executable(face_detector_pplp.exe face_detector_pplp.cpp)
target_link_libraries(face_detector_pplp.exe disjoint_sets2)
add_dependencies(face_detector_pplp.exe ${catkin_EXPORTED_TARGETS})

add_executable(face_detector2d_pplp.exe face_detector2d_pplp.cpp)
target_link_libraries(face_detector2d_pplp.exe ${catkin_LIBRARIES})
add_dependencies(face_detector2d_pplp.exe ${catkin_EXPORTED_TARGETS})

add_executable(fake_face_detector_pplp.exe fake_face_detector_pplp.cpp)
target_link_libraries(fake_face_detector_pplp.exe ${catkin_LIBRARIES})
add_dependencies(fake_face_detector_pplp.exe ${catkin_EXPORTED_TARGETS})
