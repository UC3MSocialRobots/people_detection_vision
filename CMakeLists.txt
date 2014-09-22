cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET(AD_USE_ROS 1) # needed for CMakeMacros to use ROS building routines

### include the macros
set( CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../cmake" )
include(CMakeMacros)

### build the custom messages and services
#rosbuild_genmsg()
#INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/msg_gen/cpp/include)
#rosbuild_gensrv()
#INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/srv_gen/cpp/include)

#### dependencies
find_package(PCL 1.2 REQUIRED)

### compile !
SET(CMAKE_VERBOSE_MAKEFILE OFF)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(test)
