add_executable(face_detector_pplp.exe face_detector_pplp.cpp)
target_link_libraries(face_detector_pplp.exe disjoint_sets2)

add_executable(face_detector2d_pplp.exe face_detector2d_pplp.cpp)
target_link_libraries(face_detector2d_pplp.exe ${catkin_LIBRARIES})

add_executable(fake_face_detector_pplp.exe fake_face_detector_pplp.cpp)
target_link_libraries(fake_face_detector_pplp.exe string_utils ${catkin_LIBRARIES})

add_executable(color_face_detector.exe color_face_detector.cpp 
                                   color_face_detector.h)
target_link_libraries(color_face_detector.exe ${catkin_LIBRARIES})

add_executable(texture_sensor.exe texture_sensor.cpp texture_sensor.h)
target_link_libraries(texture_sensor.exe ${catkin_LIBRARIES})

add_executable(faces_benchmark.exe faces_benchmark.cpp)
target_link_libraries(faces_benchmark.exe xml_images_interface 
					  xml_images_reader xml_document string_utils
                                          libdai ${catkin_LIBRARIES})

add_executable(face_eyes_detector.exe face_eyes_detector.cpp)
target_link_libraries(face_eyes_detector.exe ${catkin_LIBRARIES})
