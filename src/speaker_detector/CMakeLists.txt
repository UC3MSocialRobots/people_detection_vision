add_maggie_test(motion_based_speaker_detector.exe
    SOURCES     motion_based_speaker_detector.cpp
    LINK        lap
)

add_maggie_test(test_motion_based_speaker_detector.exe
    SOURCES     test_motion_based_speaker_detector.cpp
                motion_based_speaker_detector.h
    LINK        lap
)

add_maggie_test(face_warper_speaker_detector.exe
    SOURCES     face_warper_speaker_detector.cpp
    LINK        lap
)
