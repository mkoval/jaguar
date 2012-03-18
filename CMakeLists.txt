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

rosbuild_add_executable(assign_id
    src/assign_id.cc
)

rosbuild_add_executable(diff_drive
    src/diff_drive.cc
    src/diff_drive_node.cc
)

rosbuild_add_gtest(utests
    test/jaguar_test.cc
    test/jaguar_bridge_test.cc
    test/jaguar_helper_test.cc
)

rosbuild_link_boost(jaguar signals system thread)
target_link_libraries(assign_id jaguar)
target_link_libraries(diff_drive jaguar)
target_link_libraries(utests jaguar gtest_main gmock)