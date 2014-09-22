#add_maggie_test(cloud_filter.exe
#    SOURCES     cloud_filter.cpp
#)
rosbuild_add_library(multi_modal  cloud_filter.cpp)

add_maggie_test(floor_removal.exe
    SOURCES     floor_removal.cpp
)

# does not compile with Fuerte
add_maggie_test(cluster_detector.exe
    SOURCES     cluster_detector.cpp
)
