add_maggie_test(ppm_pplp.exe
    SOURCES     ppm_pplp.cpp   ppm_pplp.h
    LINK        disjoint_sets2
)

add_maggie_test(nite_user_mask_pplp.exe
    SOURCES     nite_user_mask_pplp.cpp
    LINK
)

add_maggie_test(tabletop_pplp.exe
    SOURCES     tabletop_pplp.cpp
    LINK        disjoint_sets2
)


