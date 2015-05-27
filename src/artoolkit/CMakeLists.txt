### copy ARToolKit patterns from long_term_memory to ar_pose

add_custom_target(copy_artoolit_patterns ALL
    COMMAND       bash `rospack find people_detection_vision`/data/artoolkit/copy_patterns_to_ar_pose.bash
    DEPENDS
)

add_executable(artoolkit_pplp.exe
			   artoolkit_pplp.cpp artoolkit_utils.h)
target_link_libraries(artoolkit_pplp.exe
					  ${catkin_LIBRARIES})
add_dependencies(artoolkit_pplp.exe
                 ${people_msgs_EXPORTED_TARGETS})

add_executable(artoolkit_img2patt.exe artoolkit_img2patt.cpp)
target_link_libraries(artoolkit_img2patt.exe string_utils ${catkin_LIBRARIES})

add_executable(artoolkit_patt2img.exe artoolkit_patt2img.cpp)
target_link_libraries(artoolkit_patt2img.exe string_utils ${catkin_LIBRARIES})
