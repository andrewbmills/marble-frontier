#include <algorithm>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <msfm3d/Goal.h>
#include <msfm3d/GoalArray.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// pcl ROS
#include <pcl_conversions/pcl_conversions.h>
// local packages
#include <reach.h>

// Cost map global
MapGrid3D<double> speedMap;
float voxelSize = 0.2;
bool mapUpdated = false;
bool firstMapReceived = false;
double speedMax = 10.0;
double speedSafe = 0.0;
// Frontier global
pcl::PointCloud<pcl::PointXYZ>::Ptr frontierCloud(new pcl::PointCloud<pcl::PointXYZ>);
MapGrid3D<bool> frontier;
bool frontierUpdated = false;
bool firstFrontierReceived = false;
// Robot pose
geometry_msgs::Pose robot;
bool poseUpdated = false;
std_msgs::Header pathMsgHeader;
bool firstPoseReceived = false;

void GetPointCloudBounds(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float min[3], float max[3])
{
  Point boundsMin{0.0, 0.0, 0.0};
  Point boundsMax{0.0, 0.0, 0.0};

  // Get the xyz extents of the PCL by running one loop through the data
  for (int i=0; i<cloud->points.size(); i++) {
    pcl::PointXYZI query = cloud->points[i];
    if (i == 0) {
      boundsMin.x = (double)query.x; boundsMin.y = (double)query.y; boundsMin.z = (double)query.z;
      boundsMax.x = (double)query.x; boundsMax.y = (double)query.y; boundsMax.z = (double)query.z;
      continue;
    }
    if (query.x < boundsMin.x) boundsMin.x = (double)query.x;
    if (query.y < boundsMin.y) boundsMin.y = (double)query.y;
    if (query.z < boundsMin.z) boundsMin.z = (double)query.z;
    if (query.x > boundsMax.x) boundsMax.x = (double)query.x;
    if (query.y > boundsMax.y) boundsMax.y = (double)query.y;
    if (query.z > boundsMax.z) boundsMax.z = (double)query.z;
  }

  min[0] = boundsMin.x; min[1] = boundsMin.y; min[2] = boundsMin.z;
  max[0] = boundsMax.x; max[1] = boundsMax.y; max[2] = boundsMax.z;
  return;
}

void GetPointCloudBounds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float min[3], float max[3])
{
  Point boundsMin{0.0, 0.0, 0.0};
  Point boundsMax{0.0, 0.0, 0.0};

  // Get the xyz extents of the PCL by running one loop through the data
  for (int i=0; i<cloud->points.size(); i++) {
    pcl::PointXYZ query = cloud->points[i];
    if (i == 0) {
      boundsMin.x = (double)query.x; boundsMin.y = (double)query.y; boundsMin.z = (double)query.z;
      boundsMax.x = (double)query.x; boundsMax.y = (double)query.y; boundsMax.z = (double)query.z;
      continue;
    }
    if (query.x < boundsMin.x) boundsMin.x = (double)query.x;
    if (query.y < boundsMin.y) boundsMin.y = (double)query.y;
    if (query.z < boundsMin.z) boundsMin.z = (double)query.z;
    if (query.x > boundsMax.x) boundsMax.x = (double)query.x;
    if (query.y > boundsMax.y) boundsMax.y = (double)query.y;
    if (query.z > boundsMax.z) boundsMax.z = (double)query.z;
  }

  min[0] = boundsMin.x; min[1] = boundsMin.y; min[2] = boundsMin.z;
  max[0] = boundsMax.x; max[1] = boundsMax.y; max[2] = boundsMax.z;
  return;
}

void CallbackSpeedMap(const sensor_msgs::PointCloud2::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  speedMap.voxels.clear();

  pcl::PointCloud<pcl::PointXYZI>::Ptr speedMapCloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *speedMapCloud);
  // Get bounds
  float boundsMin[3], boundsMax[3];
  GetPointCloudBounds(speedMapCloud, boundsMin, boundsMax);

  // Copy cloud to speedMap data structure
  int size[3];
  for (int i=0; i<3; i++) {
    boundsMin[i] = boundsMin[i] - 2.0*voxelSize;
    boundsMax[i] = boundsMax[i] + 2.0*voxelSize;
    size[i] = std::roundf((boundsMax[i] - boundsMin[i])/voxelSize) + 1;
  }

  MapGrid3D<double> speedMapNew(voxelSize, size, boundsMin);
  speedMap = speedMapNew;
  speedMap.SetAll(0.0);

  for (int i=0; i<speedMapCloud->points.size(); i++) {
    pcl::PointXYZI query = speedMapCloud->points[i];
    // Consider the hyperbolic tan function from the btraj paper
    // esdf = v_max*(tanh(d - e) + 1)/2
    if (query.intensity >= speedSafe) speedMap.SetVoxel(query.x, query.y, query.z, min((double)query.intensity, speedMax));
  }
  
  mapUpdated = true;
  pathMsgHeader = msg->header;
  firstMapReceived = true;
}

Point ProjectPointToSpeedMapZ(Point p, MapGrid3D<double> map)
{
  Point pProjected = p;
  bool positive = true;
  int dVoxel = 1;
  int itt = 0;
  while ((map.Query(pProjected.x, pProjected.y, pProjected.z) < map.voxelSize) && (itt < 50)) {
    if (positive) {
      pProjected.z = min(p.z + dVoxel*map.voxelSize, map.maxBounds.z);
      positive = false;
    }
    else {
      pProjected.z = max(p.z - dVoxel*map.voxelSize, map.minBounds.z);
      positive = true;
      dVoxel++;
    }
    itt++;
  }
  return pProjected;
}

void CallbackFrontier(const sensor_msgs::PointCloud2::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  pcl::fromROSMsg(*msg, *frontierCloud);
  frontierUpdated = true;
  firstFrontierReceived = true;
  return;
}

MapGrid3D<bool> ConvertFrontierCloudToMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  MapGrid3D<bool> frontierNew(voxelSize, speedMap.size, speedMap.minBounds);
  frontierNew.SetAll(false);

  for (int i=0; i<cloud->points.size(); i++) {
    Point query = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
    if (speedMap._CheckVoxelPositionInBounds(query)) {
      if (speedMap.Query(query.x, query.y, query.z) >= voxelSize) {
        frontierNew.SetVoxel(query.x, query.y, query.z, true);
      }
    }
  }
  return frontierNew;
}

void CallbackPose(const geometry_msgs::PoseStamped msg)
{
  robot.position = msg.pose.position;
  robot.orientation = msg.pose.orientation;
  poseUpdated = true;
  firstPoseReceived = true;
  return;
}

void CallbackOdometry(const nav_msgs::Odometry msg)
{
  robot.position = msg.pose.pose.position;
  robot.orientation = msg.pose.pose.orientation;
  poseUpdated = true;
  firstPoseReceived = true;
  return;
}

void CallbackGoalsMarkers(const visualization_msgs::Marker::ConstPtr msg)
{
  if (msg->points.size() == 0) return;
  frontierCloud->points.clear();
  for (int i=0; i<msg->points.size(); i++) {
    pcl::PointXYZ p = {msg->points[i].x, msg->points[i].y, msg->points[i].z};
    frontierCloud->push_back(p);
  }
  frontierUpdated = true;
  firstFrontierReceived = true;
  return;
}

struct Quaternion {
  float w, x, y, z;
};

Quaternion euler2Quaternion(float yaw, float pitch, float roll) // yaw (Z), pitch (Y), roll (X)
{
  // From Wikipedia for 3-2-1 euler angles
  // Abbreviations for the various angular functions
  float cy = std::cos(yaw * 0.5);
  float sy = std::sin(yaw * 0.5);
  float cp = std::cos(pitch * 0.5);
  float sp = std::sin(pitch * 0.5);
  float cr = std::cos(roll * 0.5);
  float sr = std::sin(roll * 0.5);

  Quaternion q;
  q.w = cy * cp * cr + sy * sp * sr;
  q.x = cy * cp * sr - sy * sp * cr;
  q.y = sy * cp * sr + cy * sp * cr;
  q.z = sy * cp * cr - cy * sp * sr;
  return q;
}

std::pair<nav_msgs::Path, geometry_msgs::PoseStamped> ConvertPointVectorToPathMsg(std::vector<Point> points)
{
  ROS_INFO("Converting path vector of length %d to path message", points.size());
  nav_msgs::Path path;
  for (int i=0; i<points.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = points[i].x; pose.pose.position.y = points[i].y; pose.pose.position.z = points[i].z;
    path.poses.push_back(pose);
  }

  geometry_msgs::PoseStamped goalPose;
  assert(points.size()>1);
  Point goalVec = {points[points.size()-1].x - points[points.size()-2].x, 
                   points[points.size()-1].y - points[points.size()-2].y,
                   points[points.size()-1].z - points[points.size()-2].z};
  float yaw = atan2(goalVec.y, goalVec.x);
  Point zeroVec = {0.0, 0.0, 0.0};
  float pitch = asin(goalVec.z/dist3(goalVec, zeroVec));
  Quaternion q = euler2Quaternion(yaw, pitch, 0.0);
  goalPose.pose.position.x = points[points.size()-1].x;
  goalPose.pose.position.y = points[points.size()-1].y;
  goalPose.pose.position.z = points[points.size()-1].z;
  goalPose.pose.orientation.w = q.w;
  goalPose.pose.orientation.x = q.x;
  goalPose.pose.orientation.y = q.y;
  goalPose.pose.orientation.z = q.z;

  return std::make_pair(path, goalPose);
}

sensor_msgs::PointCloud2 ConvertReachMapToPointCloud2(MapGrid3D<double>* map)
{
  sensor_msgs::PointCloud2 msg;
  pcl::PointCloud<pcl::PointXYZI>::Ptr reachCloud(new pcl::PointCloud<pcl::PointXYZI>);
  for (int i=0; i<map->voxels.size(); i++) {
    if (map->voxels[i] > 0.0) {
      Point query = map->_ConvertIndexToPosition(i);
      pcl::PointXYZI p;
      p.x = query.x; p.y = query.y; p.z = query.z;
      p.intensity = map->voxels[i];
      reachCloud->points.push_back(p);
    }
  }
  pcl::toROSMsg(*reachCloud, msg);
  msg.header = pathMsgHeader;
  return msg;
}

// Could put this function inside of mapGrid3D header file
int main(int argc, char **argv)
{
  ros::init(argc, argv, "frontier_planner");
  ros::NodeHandle n;

  ros::Subscriber subspeedMap = n.subscribe("speed_map", 1, &CallbackSpeedMap);
  ros::Subscriber subGoals = n.subscribe("frontier", 1, &CallbackFrontier);
  ros::Subscriber subPose = n.subscribe("pose", 1, &CallbackPose);
  ros::Subscriber subOdometry = n.subscribe("odometry", 1, &CallbackOdometry);
  ros::Subscriber subGoalsMarkers = n.subscribe("goal_points", 1, &CallbackGoalsMarkers);
  ros::Publisher pubPath = n.advertise<nav_msgs::Path>("path", 5);
  ros::Publisher pubGoalArray = n.advertise<msfm3d::GoalArray>("goalArray", 5);
  ros::Publisher pubReach = n.advertise<sensor_msgs::PointCloud2>("reach_grid", 5);
  ros::Publisher pubGoalPose = n.advertise<geometry_msgs::PoseStamped>("goal_pose", 5);

  // Params
  int numGoals;
  n.param("frontier_planner/num_agents", numGoals, (int)1);
  float goalSeparationDistance;
  n.param("frontier_planner/goal_separation_distance", goalSeparationDistance, (float)5.0);
  n.param("frontier_planner/voxel_size", voxelSize, (float)0.2);
  n.param("frontier_planner/speed_max", speedMax, (double)10.0);
  n.param("frontier_planner/speed_safe", speedSafe, (double)0.4);
  double marchingTimeOut;
  n.param("frontier_planner/marching_timeout", marchingTimeOut, (double)0.5);
  ROS_INFO("Voxel size set to %0.2f", voxelSize);
  float update_rate;
  n.param("frontier_planner/update_rate", update_rate, (float)1.0);
  std::string pathMode;
  n.param<std::string>("frontier_planner/path_mode", pathMode, "Astar");
  
  ros::Rate r(update_rate); // 5 Hz
  ROS_INFO("Finished reading params.");
  // Main Loop
  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
    if ((frontierUpdated) && (firstMapReceived*firstPoseReceived*firstFrontierReceived)) {
      clock_t tStart = clock();
      ROS_INFO("Planning to best frontier.");
      mapUpdated = false; poseUpdated = false; frontierUpdated = false;

      // Project Robot position to speedMap
      ROS_INFO("Projecting robot position onto map...");
      Point robotPosition = {robot.position.x, robot.position.y, robot.position.z};
      robotPosition = ProjectPointToSpeedMapZ(robotPosition, speedMap);
      robot.position.x = robotPosition.x; robot.position.y = robotPosition.y; robot.position.z = robotPosition.z;
      int robotPositionIds[3] = {std::roundf((robot.position.x - speedMap.minBounds.x)/speedMap.voxelSize),
          std::roundf((robot.position.y - speedMap.minBounds.y)/speedMap.voxelSize),
          std::roundf((robot.position.z - speedMap.minBounds.z)/speedMap.voxelSize)};

      // Fast Marching
      frontier = ConvertFrontierCloudToMap(frontierCloud);
      MapGrid3D<double> reachMap(voxelSize, speedMap.size, speedMap.minBounds);
      reachMap.SetAll(0.0);
      ROS_INFO("Calculating cost to frontiers...");
      std::vector<pcl::PointXYZI> goalsRanked = reach(robotPositionIds, &speedMap, frontier.voxels, &reachMap, true, true, min(frontierCloud->size(), numGoals), marchingTimeOut, goalSeparationDistance);
      pubReach.publish(ConvertReachMapToPointCloud2(&reachMap));

      // Get paths using A* and publish them
      ROS_INFO("Tracing paths from goals back to robot...");
      msfm3d::GoalArray goalArrayMsg;
      for (int i=0; i<goalsRanked.size(); i++) {
        Point goal = {goalsRanked[i].x, goalsRanked[i].y, goalsRanked[i].z};
        std::vector<Point> path;
        if (pathMode == "gradient") path = followGradientPath(robotPosition, goal, &reachMap, &speedMap);
        else if (pathMode == "gradient2D") path = followGradientPathManifold2D(robotPosition, goal, &reachMap);
        else path = AStar(robotPosition, goal, &reachMap, &speedMap);
        msfm3d::Goal goalMsg;
        std::pair<nav_msgs::Path, geometry_msgs::PoseStamped> pathPoseMsg = ConvertPointVectorToPathMsg(path);
        pathPoseMsg.second.header = pathMsgHeader;
        goalMsg.pose = pathPoseMsg.second;
        pathPoseMsg.first.header = pathMsgHeader;
        goalMsg.path = pathPoseMsg.first;
        goalMsg.cost.data = goalsRanked[i].intensity;
        goalArrayMsg.goals.push_back(goalMsg);
        goalArrayMsg.costHome.data = -1.0;

        if (i==0) {
          pubPath.publish(pathPoseMsg.first);
          pubGoalPose.publish(pathPoseMsg.second);
          ROS_INFO("Path published!");
        }
      }
      pubGoalArray.publish(goalArrayMsg);
      ROS_INFO("Plan generated in: %.3fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    }
  }

  return 0;
}