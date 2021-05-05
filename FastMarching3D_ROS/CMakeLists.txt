cmake_minimum_required(VERSION 2.8.3)
project(msfm3d)
set(CMAKE_CXX_STANDARD 11)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules/")

find_package(Eigen3 REQUIRED)
find_package(OpenMP)
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    cv_bridge
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
  Goal.msg
  GoalArray.msg
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
  "${PROJECT_SOURCE_DIR}/test/src/"
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(msfm3d msfm3d_node.cpp)
target_link_libraries(msfm3d
  ${catkin_LIBRARIES}
)

add_executable(costmap_generator src/costmap_generator.cpp)
target_link_libraries(costmap_generator
  ${catkin_LIBRARIES}
)

add_executable(VoxelHashingTest src/VoxelHashingTest.cpp)
target_link_libraries(VoxelHashingTest
  ${catkin_LIBRARIES}
)

add_executable(gain_test_node src/gain_test.cpp)
target_link_libraries(gain_test_node
  ${catkin_LIBRARIES}
)

add_executable(depthImageToPointCloud2 src/depthImg2PCL2.cpp)
target_link_libraries(depthImageToPointCloud2
  ${catkin_LIBRARIES}
)

add_executable(frontier src/frontier_node.cpp)
target_link_libraries(frontier
  ${catkin_LIBRARIES}
)
if(OpenMP_CXX_FOUND)
    target_link_libraries(frontier PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(frontier_planner src/frontier_planner_node.cpp)
target_link_libraries(frontier_planner
  ${catkin_LIBRARIES}
)

add_executable(goal_pose_sampler src/goal_pose_sampler_node.cpp)
target_link_libraries(goal_pose_sampler
  ${catkin_LIBRARIES}
)

add_executable(goal_pose_planner src/goal_pose_planner_node.cpp)
target_link_libraries(goal_pose_planner
  ${catkin_LIBRARIES}
)

add_executable(debug_paths_from_bag src/debug_paths_from_bag.cpp)
target_link_libraries(debug_paths_from_bag
  ${catkin_LIBRARIES}
)

add_executable(hashGrid3D_test test/src/hashGrid3D_test.cpp)
target_link_libraries(hashGrid3D_test
  ${catkin_LIBRARIES}
)

add_executable(hashGrid3D_conversions_test test/src/hashGrid3D_conversions_test.cpp)
target_link_libraries(hashGrid3D_conversions_test
  ${catkin_LIBRARIES}
)

add_executable(frontier_test test/src/frontier_test.cpp)
target_link_libraries(frontier_test
  ${catkin_LIBRARIES}
)