add_executable(ppm_pplp.exe ppm_pplp.cpp ppm_pplp.h)
target_link_libraries(ppm_pplp.exe disjoint_sets2)

add_executable(nite_user_mask_pplp.exe nite_user_mask_pplp.cpp)
target_link_libraries(nite_user_mask_pplp.exe ${catkin_LIBRARIES})

add_executable(tabletop_pplp.exe tabletop_pplp.cpp)
target_link_libraries(tabletop_pplp.exe disjoint_sets2)


