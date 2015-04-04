########################################################################
### options
########################################################################
#OPTION(FOO_OPTION           "a string explaining it"     OFF)

########################################################################
### find deps
########################################################################
#AD_CHECK_PROGRAM_INSTALLED(mplayer)
INCLUDE_DIRECTORIES("third_parties/libdai/include")

########################################################################
### then add our folders
########################################################################

add_executable(twist_stamper.exe twist_stamper.cpp)
target_link_libraries(twist_stamper.exe ${catkin_LIBRARIES})

add_executable(robot_wanderer.exe robot_wanderer.cpp)
target_link_libraries(robot_wanderer.exe ${catkin_LIBRARIES})

add_executable(robot_info_grabber.exe robot_info_grabber.cpp)
target_link_libraries(robot_info_grabber.exe ${catkin_LIBRARIES})

add_executable(robot_wanderer_with_moving_goal.exe
         robot_wanderer_with_moving_goal.cpp)
target_link_libraries(robot_wanderer_with_moving_goal.exe ${catkin_LIBRARIES})

################################################################################

ADD_SUBDIRECTORY(third_parties)

ADD_SUBDIRECTORY(artoolkit) # how to find rosmake-based "ar_pose" project?
ADD_SUBDIRECTORY(cluster_detector) ## error: ‘transformPointCloud’ no es un miembro de ‘pcl’
ADD_SUBDIRECTORY(bg_substractor_pplp)
ADD_SUBDIRECTORY(depth_people_detector)
ADD_SUBDIRECTORY(face_detector)
ADD_SUBDIRECTORY(fast_cluster)
ADD_SUBDIRECTORY(hog_pplp) # troubles with PCL: /usr/include/pcl-1.7/pcl/kdtree/kdtree_flann.h:115:9: error: ‘param_k_’ was not declared in this scope
ADD_SUBDIRECTORY(leg_detector)
ADD_SUBDIRECTORY(ppl_benchmarking)
ADD_SUBDIRECTORY(speaker_detector)
ADD_SUBDIRECTORY(stereo)
