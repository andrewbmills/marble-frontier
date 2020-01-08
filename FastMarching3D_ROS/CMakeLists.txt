cmake_minimum_required(VERSION 2.8.3)
project(msfm3d)
set(CMAKE_CXX_STANDARD 11)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules/")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    pcl_ros
    pcl_conversions
    sensor_msgs
    octomap_ros
    octomap_msgs
    cmake_modules
    message_generation
    message_runtime
)

add_definitions(${EIGEN_DEFINITIONS})

add_message_files(
  FILES
  PathArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS std_msgs
  )

include_directories(
    "${PROJECT_SOURCE_DIR}/include/"
    "${PROJECT_SOURCE_DIR}/src/"
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    )


add_executable(msfm3d msfm3d_node.cpp)
target_link_libraries(msfm3d
  ${catkin_LIBRARIES}
)