add_executable(ppm_pplp.exe ppm_pplp.cpp )
target_link_libraries(ppm_pplp.exe)
add_dependencies(ppm_pplp.exe ${catkin_EXPORTED_TARGETS})

add_executable(nite_user_mask_pplp.exe nite_user_mask_pplp.cpp)
target_link_libraries(nite_user_mask_pplp.exe ${catkin_LIBRARIES})
add_dependencies(nite_user_mask_pplp.exe ${catkin_EXPORTED_TARGETS})

add_executable(tabletop_pplp.exe tabletop_pplp.cpp)
target_link_libraries(tabletop_pplp.exe)
add_dependencies(tabletop_pplp.exe ${catkin_EXPORTED_TARGETS})
