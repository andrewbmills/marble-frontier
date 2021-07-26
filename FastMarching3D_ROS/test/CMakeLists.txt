cmake_minimum_required(VERSION 2.8.3)
project(msfm3d_test)
set(CMAKE_CXX_STANDARD 11)

include_directories(
    "${PROJECT_SOURCE_DIR}/src/"
    "~/catkin_ws/src/marble-frontier/FastMarching3D_ROS/include/"
)

message('${CMAKE_MODULE_PATH}')
find_package(Catch2 REQUIRED)

add_executable(mapGrid3D_test mapGrid3D_test.cpp)
target_link_libraries(mapGrid3D_test PRIVATE Catch2::Catch2)

include(CTest)
include(Catch2)
catch_discover_tests(catch_test)