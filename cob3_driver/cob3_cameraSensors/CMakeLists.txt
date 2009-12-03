cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/lib)

#uncomment if you have defined messages
#genmsg()
#uncomment if you have defined services
#gensrv()

#common commands for building c++ executables and libraries
#rospack_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)
#rospack_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# add project executable
rosbuild_add_executable(${PROJECT_NAME}_swissRanger ros/src/${PROJECT_NAME}_swissRanger.cpp)
rosbuild_add_executable(${PROJECT_NAME}_swissRanger_test ros/src/${PROJECT_NAME}_swissRanger_test.cpp)
rosbuild_add_executable(${PROJECT_NAME}_colorCamera ros/src/${PROJECT_NAME}_colorCamera.cpp)
rosbuild_add_executable(${PROJECT_NAME}_colorCamera_test ros/src/${PROJECT_NAME}_colorCamera_test.cpp)

# add include search paths
#INCLUDE_DIRECTORIES(/usr/local/include/opencv)
#INCLUDE_DIRECTORIES(/usr/include/Wm4)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common)

# add compile flag
rosbuild_add_compile_flags(${PROJECT_NAME}_swissRanger -D__LINUX__)
rosbuild_add_compile_flags(${PROJECT_NAME}_swissRanger_test -D__LINUX__)
rosbuild_add_compile_flags(${PROJECT_NAME}_colorCamera -D__LINUX__)
rosbuild_add_compile_flags(${PROJECT_NAME}_colorCamera_test -D__LINUX__)

# link libraries
target_link_libraries(${PROJECT_NAME}_swissRanger dc1394 cv cxcore highgui mesasr Wm4Foundation)
target_link_libraries(${PROJECT_NAME}_swissRanger ${PROJECT_SOURCE_DIR}/common/lib/libtinyxml.a)
target_link_libraries(${PROJECT_NAME}_swissRanger ${PROJECT_SOURCE_DIR}/common/lib/libCameraSensors.so)
target_link_libraries(${PROJECT_NAME}_swissRanger_test cv cxcore highgui mesasr Wm4Foundation)
target_link_libraries(${PROJECT_NAME}_swissRanger_test ${PROJECT_SOURCE_DIR}/common/lib/libtinyxml.a)
target_link_libraries(${PROJECT_NAME}_swissRanger_test ${PROJECT_SOURCE_DIR}/common/lib/libCameraSensors.so)
target_link_libraries(${PROJECT_NAME}_colorCamera dc1394 cv cxcore highgui mesasr Wm4Foundation)
target_link_libraries(${PROJECT_NAME}_colorCamera ${PROJECT_SOURCE_DIR}/common/lib/libtinyxml.a)
target_link_libraries(${PROJECT_NAME}_colorCamera ${PROJECT_SOURCE_DIR}/common/lib/libCameraSensors.so)
target_link_libraries(${PROJECT_NAME}_colorCamera_test cv cxcore highgui mesasr Wm4Foundation)
target_link_libraries(${PROJECT_NAME}_colorCamera_test ${PROJECT_SOURCE_DIR}/common/lib/libtinyxml.a)
target_link_libraries(${PROJECT_NAME}_colorCamera_test ${PROJECT_SOURCE_DIR}/common/lib/libCameraSensors.so)
