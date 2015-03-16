### 1 - Required CMake Version
cmake_minimum_required(VERSION 2.8.3)
### 2- Package Name
project(people_detection_vision)
### 3 - Find other CMake/Catkin packages needed for build
find_package(catkin REQUIRED COMPONENTS
  roscpp cv_bridge # ar_pose
  # AD
  people_msgs people_utils utils vision_utils followme_laser)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
### 4 - Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())
# Nothing in this package
### 5 - Invoke message/service/action generation (generate_messages())
# Nothing in this package
### 6 - Specify package build info export (catkin_package())
catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp cv_bridge
  DEPENDS people_msgs people_utils followme_laser
          utils vision_utils
)
### 7 - Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
include_directories(
  include src
  ${catkin_INCLUDE_DIRS}
  /usr/include/opencv2
  ${PCL_INCLUDE_DIRS}
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
