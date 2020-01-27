cmake_minimum_required(VERSION 2.8.3)
project(traversability_mapping)
set(CMAKE_CXX_STANDARD 11)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules/")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    pcl_ros
    pcl_conversions
    sensor_msgs
    cmake_modules
    octomap_ros
    octomap_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS std_msgs
)

include_directories(
    "${PROJECT_SOURCE_DIR}/include/"
    "${PROJECT_SOURCE_DIR}/src/"
    ${catkin_INCLUDE_DIRS}
)

add_executable(normal_filter src/PCL_normal_filter.cpp)
target_link_libraries(normal_filter
  ${catkin_LIBRARIES}
)