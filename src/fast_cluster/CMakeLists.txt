
#~ add_maggie_test(kinect_template.exe
    #~ SOURCES     kinect_template.cpp
#~ )


################################################################################

add_executable(fast_cluster_detector.exe fast_cluster_detector.cpp
                                         )
target_link_libraries(fast_cluster_detector.exe disjoint_sets2 ${catkin_LIBRARIES})

add_executable(fast_cluster_selector.exe fast_cluster_selector.cpp
                                         )
target_link_libraries(fast_cluster_selector.exe ${catkin_LIBRARIES})

add_executable(fast_cluster_joy_control.exe fast_cluster_joy_control.cpp
                                            )
target_link_libraries(fast_cluster_joy_control.exe ${catkin_LIBRARIES})

add_executable(fast_dialog.exe fast_dialog.cpp)
target_link_libraries(fast_dialog.exe ${catkin_LIBRARIES})

################################################################################

add_executable(test_connected_components_matcher.exe  
			   test_connected_components_matcher.cpp
                           )
target_link_libraries(test_connected_components_matcher.exe disjoint_sets2 ${catkin_LIBRARIES})

### rules for PedestrianDetectionClustering
#add_maggie_lib( pedestrian_detection_clustering
#    SOURCES     pedestrian_detection_clustering.cpp
#)

#add_maggie_test(test_pedestrian_detection_clustering.exe
#    SOURCES     test_pedestrian_detection_clustering.cpp
#    LINK        pedestrian_detection_clustering
#)

