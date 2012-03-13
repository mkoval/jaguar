cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_add_boost_directories()

set(ROS_BUILD_TYPE Debug)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(jaguar
	src/jaguar.cc
	src/jaguar_helper.cc
	src/jaguar_bridge.cc
	src/jaguar_broadcaster.cc
)
rosbuild_link_boost(jaguar signals system thread)

rosbuild_add_executable(assign_id src/assign_id.cc)
rosbuild_add_gtest(jaguar_bridge_test test/jaguar_bridge_test.cc)
rosbuild_add_gtest(jaguar_helper_test test/jaguar_helper_test.cc)
target_link_libraries(assign_id jaguar)
target_link_libraries(jaguar_bridge_test jaguar gtest_main gmock)
target_link_libraries(jaguar_helper_test jaguar gtest_main gmock)
