add_maggie_test(face_detector_pplp.exe
    SOURCES     face_detector_pplp.cpp
    LINK        disjoint_sets2
)

add_maggie_test(face_detector2d_pplp.exe
    SOURCES     face_detector2d_pplp.cpp
)

add_maggie_test(fake_face_detector_pplp.exe
    SOURCES     fake_face_detector_pplp.cpp
    LINK        string_utils
)

add_maggie_test(color_face_detector.exe
    SOURCES     color_face_detector.cpp color_face_detector.h
    LINK
)

add_maggie_test(texture_sensor.exe
    SOURCES     texture_sensor.cpp texture_sensor.h
    LINK
)

add_maggie_test(faces_benchmark.exe
    SOURCES     faces_benchmark.cpp
    LINK        xml_images_interface xml_images_reader xml_document string_utils
                libdai
)

add_maggie_test(face_eyes_detector.exe
    SOURCES     face_eyes_detector.cpp
)
