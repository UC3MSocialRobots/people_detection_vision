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

add_maggie_test(twist_stamper.exe
    SOURCES     twist_stamper.cpp
)

add_maggie_test(robot_wanderer.exe
    SOURCES     robot_wanderer.cpp
)

add_maggie_test(robot_info_grabber.exe
    SOURCES     robot_info_grabber.cpp
)


add_maggie_test(robot_wanderer_with_moving_goal.exe
    SOURCES     robot_wanderer_with_moving_goal.cpp
    LINK
)

################################################################################

ADD_SUBDIRECTORY(third_parties)

ADD_SUBDIRECTORY(artoolkit)
ADD_SUBDIRECTORY(cluster_detector)
ADD_SUBDIRECTORY(depth_people_detector)
ADD_SUBDIRECTORY(face_detector)
ADD_SUBDIRECTORY(fast_cluster)
ADD_SUBDIRECTORY(hog_pplp)
ADD_SUBDIRECTORY(leg_detector)
ADD_SUBDIRECTORY(ppl_benchmarking)
ADD_SUBDIRECTORY(speaker_detector)
ADD_SUBDIRECTORY(stereo)
