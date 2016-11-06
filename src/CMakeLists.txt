INCLUDE_DIRECTORIES("third_parties/libdai/include")

########################################################################
### then add our folders
########################################################################

add_executable(twist_stamper.exe twist_stamper.cpp)
target_link_libraries(twist_stamper.exe ${catkin_LIBRARIES})

add_executable(robot_info_grabber.exe robot_info_grabber.cpp)
target_link_libraries(robot_info_grabber.exe ${catkin_LIBRARIES})

################################################################################

ADD_SUBDIRECTORY(third_parties)

ADD_SUBDIRECTORY(artoolkit)
ADD_SUBDIRECTORY(bg_substractor_pplp)
ADD_SUBDIRECTORY(depth_people_detector)
ADD_SUBDIRECTORY(face_detector)
#ADD_SUBDIRECTORY(fast_cluster)
ADD_SUBDIRECTORY(hog_pplp)
ADD_SUBDIRECTORY(leg_detector)

