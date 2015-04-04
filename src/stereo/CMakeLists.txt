ADD_SUBDIRECTORY(cluster_detector)

INCLUDE_DIRECTORIES(bp-vision)

add_executable(stereo.exe stereo.cpp)
target_link_libraries(stereo.exe string_utils ${PCL_LIBRARIES} ${catkin_LIBRARIES})
