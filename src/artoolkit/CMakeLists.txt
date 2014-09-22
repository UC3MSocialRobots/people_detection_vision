### copy ARToolKit patterns from long_term_memory to ar_pose

add_custom_target(copy_artoolit_patterns ALL
    COMMAND       bash `rospack find people_detection_vision`/data/artoolkit/copy_patterns_to_ar_pose.bash
    DEPENDS
)

add_maggie_test(artoolkit_pplp.exe
    SOURCES     artoolkit_pplp.cpp artoolkit_utils.h
    LINK
)

add_maggie_test(artoolkit_img2patt.exe
    SOURCES     artoolkit_img2patt.cpp
    LINK        string_utils
)

add_maggie_test(artoolkit_patt2img.exe
    SOURCES     artoolkit_patt2img.cpp
    LINK        string_utils
)
