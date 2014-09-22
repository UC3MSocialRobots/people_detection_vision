#ADD_SUBDIRECTORY(cluster_detector)

INCLUDE_DIRECTORIES(bp-vision)

add_maggie_test(stereo.exe
    SOURCES     stereo.cpp
    LINK        string_utils
)
target_link_libraries(stereo.exe string_utils ${PCL_LIBRARIES} )
