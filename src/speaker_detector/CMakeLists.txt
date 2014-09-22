add_executable(motion_based_speaker_detector.exe motion_based_speaker_detector.cpp)
target_link_libraries(motion_based_speaker_detector.exe lap ${catkin_LIBRARIES})

add_executable(test_motion_based_speaker_detector.exe 
			   test_motion_based_speaker_detector.cpp
			   motion_based_speaker_detector)
target_link_libraries(test_motion_based_speaker_detector.exe lap ${catkin_LIBRARIES})

add_executable(face_warper_speaker_detector.exe face_warper_speaker_detector.cpp)
target_link_libraries(face_warper_speaker_detector.exe lap ${catkin_LIBRARIES})
