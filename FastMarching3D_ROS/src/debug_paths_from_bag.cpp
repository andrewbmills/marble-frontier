#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
// pcl ROS
#include <pcl_conversions/pcl_conversions.h>
// local
#include <reach.h>
#include <gain.h>

// Cost map global
MapGrid3D<double> speedMap;
pcl::PointCloud<pcl::PointXYZI>::Ptr speedMapCloud(new pcl::PointCloud<pcl::PointXYZI>);
float voxelSize = 0.2;
bool mapUpdated = false;
bool firstMapReceived = false;
double speedMax = 10.0;
double speedSafe = 0.4;
std_msgs::Header pathMsgHeader;
// Goals global
pcl::PointCloud<pcl::PointXYZINormal>::Ptr goalsCloud (new pcl::PointCloud<pcl::PointXYZINormal>);
bool firstGoalsReceived = false;
bool goalsUpdated = false;
sensor_msgs::PointCloud2 goalsMsg;
// Robot pose
geometry_msgs::Pose robot;
bool poseUpdated = false;
bool firstPoseReceived = false;
nav_msgs::Odometry odomMsg;
// Frontier global
pcl::PointCloud<pcl::PointXYZ>::Ptr frontierCloud(new pcl::PointCloud<pcl::PointXYZ>);
bool firstFrontierReceived = false;
bool frontierUpdated = false;
sensor_msgs::PointCloud2 frontierMsg;
// Last Goal poses message
geometry_msgs::PoseArray goalPosesMsg;

Point ProjectPointToSpeedCloudMap(Point p, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float intensityMin=0.0)
{
  // Finds the closest voxel coordinates in the map to the point p.

  // Filter out all points below intensityMin in cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI> passThroughFilter;
  passThroughFilter.setInputCloud(cloud);
  passThroughFilter.setFilterFieldName("intensity");
  passThroughFilter.setFilterLimits(intensityMin, 100.0);
  passThroughFilter.filter(*cloudFiltered);

  // Use KdTree PCL to do binary search and find the closest voxel to p
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(cloudFiltered);
  pcl::PointXYZI searchPoint;
  searchPoint.x = p.x; searchPoint.y = p.y; searchPoint.z = p.z; searchPoint.intensity = 0.0;
  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);
  Point closestPoint;

  if (kdtree.nearestKSearch (searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
    closestPoint.x = cloudFiltered->points[pointIdxNKNSearch[0]].x;
    closestPoint.y = cloudFiltered->points[pointIdxNKNSearch[0]].y;
    closestPoint.z = cloudFiltered->points[pointIdxNKNSearch[0]].z;
  } else {
    // If no point is found, return the original
    closestPoint = p;
  }

  return closestPoint;
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

nav_msgs::Path ConvertPointVectorToPathMsg(std::vector<Point> points)
{
  // ROS_INFO("Converting path vector of length %d to path message", points.size());
  nav_msgs::Path path;
  for (int i=0; i<points.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = points[i].x; pose.pose.position.y = points[i].y; pose.pose.position.z = points[i].z;
    path.poses.push_back(pose);
  }

  return path;
}

void GetPointCloudBounds(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float min[3], float max[3])
{
  for (int i=0; i<3; i++) {
    min[i] = 0.0; max[i] = 0.0;
  }

  // Get the xyz extents of the PCL by running one loop through the data
  for (int i=0; i<cloud->points.size(); i++) {
    pcl::PointXYZI query = cloud->points[i];
    if (i == 0) {
      min[0] = (double)query.x; min[1] = (double)query.y; min[2] = (double)query.z;
      max[0] = (double)query.x; max[1] = (double)query.y; max[2] = (double)query.z;
      continue;
    }
    if (query.x < min[0]) min[0] = (double)query.x;
    if (query.y < min[1]) min[1] = (double)query.y;
    if (query.z < min[2]) min[2] = (double)query.z;
    if (query.x > max[0]) max[0] = (double)query.x;
    if (query.y > max[1]) max[1] = (double)query.y;
    if (query.z > max[2]) max[2] = (double)query.z;
  }

  return;
}

void CallbackSpeedMap(const sensor_msgs::PointCloud2::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  speedMap.voxels.clear();

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
    if (query.intensity >= speedSafe) speedMap.SetVoxel(query.x, query.y, query.z, std::min((double)query.intensity, speedMax));
  }
  
  mapUpdated = true;
  pathMsgHeader = msg->header;
  firstMapReceived = true;
}

void CallbackOdometry(const nav_msgs::Odometry msg)
{
  if ((!mapUpdated) || (!firstPoseReceived)) {
    robot.position = msg.pose.pose.position;
    robot.orientation = msg.pose.pose.orientation;
    poseUpdated = true;
    firstPoseReceived = true;
    odomMsg = msg;
  }
  return;
}

void CallbackGoals(const sensor_msgs::PointCloud2::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  if ((!mapUpdated) || (!firstGoalsReceived)) {
    pcl::fromROSMsg(*msg, *goalsCloud);
    firstGoalsReceived = true;
    goalsUpdated = true;
    goalsMsg = *msg;
  }
  return;
}

void CallbackFrontier(const sensor_msgs::PointCloud2::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  if (!(mapUpdated) || (!firstFrontierReceived)) {
    pcl::fromROSMsg(*msg, *frontierCloud);
    firstFrontierReceived = true;
    frontierUpdated = true;
    frontierMsg = *msg;
  }
  return;
}

void CallbackPoses(const geometry_msgs::PoseArray::ConstPtr msg)
{
  if (msg->poses.size() == 0) return;
  if ((!mapUpdated)) {
    goalPosesMsg = *msg;
  }
  return;
}

int main(int argc, char **argv)
{
  // Initialize ROS node with name and object instantiation
  ros::init(argc, argv, "debug_paths_from_bag");
  ros::NodeHandle n;

  ros::Subscriber sub1 = n.subscribe("speed_map", 1, CallbackSpeedMap);
  ros::Subscriber sub2 = n.subscribe("odometry", 1, CallbackOdometry);
  ros::Subscriber sub3 = n.subscribe("goals", 1, CallbackGoals);
  ros::Subscriber sub4 = n.subscribe("frontier", 1, CallbackFrontier);
  ros::Subscriber sub5 = n.subscribe("goal_poses", 1, CallbackPoses);

  ros::Publisher pubPaths = n.advertise<nav_msgs::Path>("paths", 5);
  nav_msgs::Path pathsMsg;
  ros::Publisher pubStats = n.advertise<std_msgs::Float32MultiArray>("paths_stats", 5);
  ros::Publisher pubReach = n.advertise<sensor_msgs::PointCloud2>("reach", 5);
  ros::Publisher pubGoals = n.advertise<sensor_msgs::PointCloud2>("goals_debug", 5);
  ros::Publisher pubGoalPoses = n.advertise<geometry_msgs::PoseArray>("goal_poses_debug", 5);
  ros::Publisher pubOdom = n.advertise<nav_msgs::Odometry>("odometry_debug", 5);
  ros::Publisher pubFrontier = n.advertise<sensor_msgs::PointCloud2>("frontier_debug", 5);

  float goalSeparationDistance, turnRate;
  n.param("goal_pose_planner/goal_separation_distance", goalSeparationDistance, (float)5.0); // meters
  n.param("goal_pose_planner/turn_rate", turnRate, (float)90.0); // deg/s
  n.param("goal_pose_planner/voxel_size", voxelSize, (float)0.2); // meters
  n.param("goal_pose_planner/speed_max", speedMax, (double)10.0); // m/s
  n.param("goal_pose_planner/speed_safe", speedSafe, (double)0.4); // m/s
  double marchingTimeOut;
  n.param("goal_pose_planner/marching_timeout", marchingTimeOut, (double)5.0); // seconds
  // Declare and read in the node update rate from the launch file parameters
  double rate;
  n.param("debug_paths_from_bag/update_rate", rate, (double)0.2); // Every 5 seconds
  ros::Rate r(rate);

  // Run the node until ROS quits
  while (ros::ok())
  {
    ROS_INFO("Calling callbacks...");
    r.sleep(); // Node sleeps to update at a rate as close as possible to the updateRate parameter
    ros::spinOnce(); // All subscriber callbacks are called here.

    // Run reachRaw() on the updated ESDF map
    if ((mapUpdated) && (firstMapReceived*firstPoseReceived*firstGoalsReceived)) {
      clock_t tStart = clock();
      ROS_INFO("Planning to best goal pose.");
      mapUpdated = false; poseUpdated = false; goalsUpdated = false;

      // Project Robot position to speedMap
      ROS_INFO("Projecting robot position onto map...");
      Point robotPosition = {robot.position.x, robot.position.y, robot.position.z};
      // robotPosition = ProjectPointToSpeedMapZ(robotPosition, speedMap);
      robotPosition = ProjectPointToSpeedCloudMap(robotPosition, speedMapCloud, speedSafe + speedMap.voxelSize);
      robot.position.x = robotPosition.x; robot.position.y = robotPosition.y; robot.position.z = robotPosition.z;
      int robotPositionIds[3] = {std::roundf((robot.position.x - speedMap.minBounds.x)/speedMap.voxelSize),
          std::roundf((robot.position.y - speedMap.minBounds.y)/speedMap.voxelSize),
          std::roundf((robot.position.z - speedMap.minBounds.z)/speedMap.voxelSize)};

      // Fast Marching
      ROS_INFO("Calculating cost to goals...");
      MapGrid3D<double> reachMap(voxelSize, speedMap.size, speedMap.minBounds);
      reachMap.SetAll(0.0);
      reachRaw(robotPositionIds, &speedMap, &reachMap, true, true, marchingTimeOut);
      pubReach.publish(ConvertReachMapToPointCloud2(&reachMap));

      // Get paths using gradient following and publish them as one message
      pathsMsg.header = pathMsgHeader;
      pathsMsg.poses.clear();
      for (int i=0; i<goalsCloud->points.size(); i++) {
        Point goal;
        goal.x = goalsCloud->points[i].x; goal.y = goalsCloud->points[i].y; goal.z = goalsCloud->points[i].z;
        std::vector<Point> path = followGradientPath(robotPosition, goal, &reachMap);
        if (path.size() > 0) {
          nav_msgs::Path newPathSegment = ConvertPointVectorToPathMsg(path);
          for (int j=0; j<newPathSegment.poses.size(); j++) {
            pathsMsg.poses.push_back(newPathSegment.poses[j]);
          }
          path = flipPath(path);
          newPathSegment = ConvertPointVectorToPathMsg(path);
          for (int j=0; j<newPathSegment.poses.size(); j++) {
            pathsMsg.poses.push_back(newPathSegment.poses[j]);
          }
        }
      }
      pubPaths.publish(pathsMsg);
      pubGoals.publish(goalsMsg);
      pubGoalPoses.publish(goalPosesMsg);
      pubOdom.publish(odomMsg);
      pubFrontier.publish(frontierMsg);
    }

  }

  return 0;
}