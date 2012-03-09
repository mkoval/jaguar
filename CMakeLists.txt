cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_add_boost_directories()

set(ROS_BUILD_TYPE Debug)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(assign_id src/assign_id.cc src/jaguar.cc src/jaguar_bridge.cc)
rosbuild_link_boost(assign_id system)
rosbuild_add_link_flags(assign_id -L/usr/local/esd/lib64 -lntcan)
