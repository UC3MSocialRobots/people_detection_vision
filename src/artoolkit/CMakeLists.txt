### copy ARToolKit patterns from long_term_memory to ar_pose
add_custom_target(copy_artoolkit_patterns ALL
    COMMAND       bash `rospack find people_detection_vision`/data/artoolkit/copy_patterns_to_ar_pose.bash
    DEPENDS
)

add_executable(artoolkit_pplp.exe
         artoolkit_pplp.cpp )
target_link_libraries(artoolkit_pplp.exe ${catkin_LIBRARIES})
add_dependencies(artoolkit_pplp.exe  ar_pose_generate_messages_cpp)
