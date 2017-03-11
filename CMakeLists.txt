### 1 - Required CMake Version
cmake_minimum_required(VERSION 2.8.3)
### 2- Package Name
project(people_detection_vision)
### 3 - Find other CMake/Catkin packages needed for build
SET(ROSDEPS  cv_bridge  dynamic_reconfigure  image_geometry image_transport
             people_msgs  pcl_ros  rosbag  roscpp  roslib  tf  vision_utils)
find_package(catkin REQUIRED COMPONENTS ${ROSDEPS})
find_package(PCL REQUIRED common visualization)

### 4 - Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())
generate_dynamic_reconfigure_options(
  cfg/HOG.cfg
  cfg/Viola.cfg
)
### 5 - Invoke message/service/action generation (generate_messages())
# Nothing in this package
### 6 - Specify package build info export (catkin_package())
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${ROSDEPS}
  DEPENDS
)
### 7 - Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
ADD_SUBDIRECTORY(src)
### 8 - Tests to build (catkin_add_gtest())
ADD_SUBDIRECTORY(test)
### 9 - Install rules (install())
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
