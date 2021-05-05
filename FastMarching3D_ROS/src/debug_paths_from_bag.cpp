#include <iostream>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseArray.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
// pcl ROS
#include <pcl_conversions/pcl_conversions.h>
// local
#include <reach.h>
#include <gain.h>
#include <kinematics.h>

// Cost map global
MapGrid3D<double> speedMap;
pcl::PointCloud<pcl::PointXYZI>::Ptr speedMapCloud(new pcl::PointCloud<pcl::PointXYZI>);
double voxelSize = 0.2;
bool mapUpdated = false;
bool firstMapReceived = false;
double speedMax = 10.0;
double speedSafe = 0.4;
std_msgs::Header pathMsgHeader;
// Goals global
pcl::PointCloud<pcl::PointXYZINormal>::Ptr goalsCloud (new pcl::PointCloud<pcl::PointXYZINormal>);
bool firstGoalsReceived = false;
bool goalsUpdated = false;
bool goalPosesUpdated = false;
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
// Octomap global
octomap::OcTree* mapTree;
bool firstOctomapReceived = false;
bool octomapUpdated = false;
octomap_msgs::Octomap octomapMsg;

// Last Goal poses message
geometry_msgs::PoseArray goalPosesMsg;

struct PathStats
{
  double costReach;
  double costLength;
  double costLengthTurn;
  double costKinematic;
  double gainPath;
  double utility;
  double gainPose;
  double gainPoseCorrect;
};

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

std::vector<Point> FollowPathWithKinematics(std::vector<Point> path, geometry_msgs::Pose x0, double &tf)
{
  // Prep inputs and outputs
  path_integrate_kinematics.clear();
  for (int i=0; i<path.size(); i++) {
    std::vector<double> p{path[i].x, path[i].y, path[i].z};
    path_integrate_kinematics.push_back(p);
  }
  Quaternion q;
  q.x = x0.orientation.x; q.y = x0.orientation.y; q.z = x0.orientation.z; q.w = x0.orientation.w;
  Eigen::Matrix3f R = Quaternion2RotationMatrix(q);
  double yaw = std::atan2(R(1,0), R(0,0));
  state_type_unicycle3D x = {x0.position.x, x0.position.y, x0.position.z, yaw};
  // ROS_INFO("Initial state for kinematics is (%0.2f m, %0.2f m, %0.2f deg)", x[0], x[1], x[2]*(180.0/M_PI));

  // Run and store the kinematics ODE solver result
  states_integrate_kinematics.clear();
  integrate(unicycle3D, x, 0.0, 60.0, 0.01, unicycleObserver3D);

  // Copy result back to path_kinematics
  std::vector<Point> pathKinematic;
  for (int i=0; i<states_integrate_kinematics.size(); i++) {
    Point p;
    p.x = states_integrate_kinematics[i][1];
    p.y = states_integrate_kinematics[i][2];
    p.z = states_integrate_kinematics[i][3];
    pathKinematic.push_back(p);
  }
  tf = states_integrate_kinematics[states_integrate_kinematics.size()-1][0];
  return pathKinematic;
}

void CallbackSpeedMap(const sensor_msgs::PointCloud2::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  speedMap.voxels.clear();

  pcl::fromROSMsg(*msg, *speedMapCloud);
  // Get bounds
  double boundsMin[3], boundsMax[3];
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
  if ((!mapUpdated) || (!firstPoseReceived) || (!poseUpdated)) {
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
  if ((!mapUpdated) || (!firstGoalsReceived) || (!goalsUpdated)) {
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
  if (!(mapUpdated) || (!firstFrontierReceived) || (!frontierUpdated)) {
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
  if ((!mapUpdated) || (!goalPosesUpdated)) {
    goalPosesMsg = *msg;
    goalPosesUpdated = true;
  }
  return;
}

void CallbackOctomap(const octomap_msgs::Octomap::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  if ((!mapUpdated) || (!firstOctomapReceived) || (!octomapUpdated)) {
    delete mapTree;
    mapTree = (octomap::OcTree*)octomap_msgs::fullMsgToMap(*msg);
    firstOctomapReceived = true;
    octomapUpdated = true;
    octomapMsg = *msg;
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
  ros::Subscriber sub6 = n.subscribe("octomap_full", 1, CallbackOctomap);

  ros::Publisher pubPaths = n.advertise<nav_msgs::Path>("paths", 5);
  ros::Publisher pubPathsKinematic = n.advertise<nav_msgs::Path>("paths_kinematic", 5);
  nav_msgs::Path pathsMsg;
  nav_msgs::Path pathsMsgKinematic;
  ros::Publisher pubStats = n.advertise<std_msgs::Float32MultiArray>("paths_stats", 5);
  ros::Publisher pubReach = n.advertise<sensor_msgs::PointCloud2>("reach", 5);
  ros::Publisher pubGoals = n.advertise<sensor_msgs::PointCloud2>("goals_debug", 5);
  ros::Publisher pubGoalPoses = n.advertise<geometry_msgs::PoseArray>("goal_poses_debug", 5);
  ros::Publisher pubOdom = n.advertise<nav_msgs::Odometry>("odometry_debug", 5);
  ros::Publisher pubFrontier = n.advertise<sensor_msgs::PointCloud2>("frontier_debug", 5);
  ros::Publisher pubOctomap = n.advertise<octomap_msgs::Octomap>("octomap_debug", 5);
  ros::Publisher pubPathCloud = n.advertise<sensor_msgs::PointCloud2>("path_seen_cloud_debug", 5);
  ros::Publisher pubPoseCloud = n.advertise<sensor_msgs::PointCloud2>("pose_seen_cloud_debug", 5);
  ros::Publisher pubPathCurrent = n.advertise<nav_msgs::Path>("path_current_debug", 5);
  ros::Publisher pubPathCurrentKinematic = n.advertise<nav_msgs::Path>("path_current_kinematic_debug", 5);

  float goalSeparationDistance, turnRate;
  n.param("debug_paths_from_bag/goal_separation_distance", goalSeparationDistance, (float)5.0); // meters
  n.param("debug_paths_from_bag/turn_rate", turnRate, (float)90.0); // deg/s
  n.param("debug_paths_from_bag/voxel_size", voxelSize, (double)0.2); // meters
  n.param("debug_paths_from_bag/speed_max", speedMax, (double)10.0); // m/s
  n.param("debug_paths_from_bag/speed_safe", speedSafe, (double)0.4); // m/s
  double marchingTimeOut;
  n.param("debug_paths_from_bag/marching_timeout", marchingTimeOut, (double)5.0); // seconds
  // Declare and read in the node update rate from the launch file parameters
  double rate;
  SensorFoV sensorParams;
  n.param("debug_paths_from_bag/update_rate", rate, (double)0.2); // Every 5 seconds
  n.param("debug_paths_from_bag/sensor_hFoV", sensorParams.horizontalFoV, (float)60.0);
  n.param("debug_paths_from_bag/sensor_vFoV", sensorParams.verticalFoV, (float)40.0);
  n.param("debug_paths_from_bag/sensor_rMax", sensorParams.rMax, (float)60.0);
  n.param("debug_paths_from_bag/sensor_rMin", sensorParams.rMin, (float)60.0);
  n.param<std::string>("debug_paths_from_bag/sensor_type", sensorParams.type, "camera");
  std::string gainType;
  std::string debugMode;
  n.param<std::string>("debug_paths_from_bag/gain_type", gainType, "frontier");
  n.param<std::string>("debug_paths_from_bag/debug_mode", debugMode, "normal");
  std::string filename; std::string filenameCost; std::string filenameGainPath; std::string filenameGainPose; std::string filenameUtility;
  n.param<std::string>("debug_paths_from_bag/filename", filename, "/home/andrew/tests/data/debug_paths.csv");
  n.param<std::string>("debug_paths_from_bag/filename_costs", filenameCost, "/home/andrew/tests/data/debug_paths_costs.csv");
  n.param<std::string>("debug_paths_from_bag/filename_gains_paths", filenameGainPath, "/home/andrew/tests/data/debug_paths_gains_path.csv");
  n.param<std::string>("debug_paths_from_bag/filename_gains_poses", filenameGainPose, "/home/andrew/tests/data/debug_paths_gains_pose.csv");
  n.param<std::string>("debug_paths_from_bag/filename_utilities", filenameUtility, "/home/andrew/tests/data/debug_paths_utilities.csv");
  n.param("debug_paths_from_bag/kinematics_L1", L1params.L1, (double)1.0);
  n.param("debug_paths_from_bag/kinematics_speed", L1params.speed, (double)1.0);
  n.param("debug_paths_from_bag/kinematics_speed_max", L1params.speed_max, (double)1.0);
  n.param("debug_paths_from_bag/kinematics_yaw_rate_max", L1params.yaw_rate_max, (double)0.4);
  n.param("debug_paths_from_bag/kinematics_z_gain", L1params.z_gain, (double)0.5);
  ros::Rate r(rate);

  // Run the node until ROS quits
  while (ros::ok())
  {
    ROS_INFO("Calling callbacks...");
    r.sleep(); // Node sleeps to update at a rate as close as possible to the updateRate parameter
    ros::spinOnce(); // All subscriber callbacks are called here.

    // Run reachRaw() on the updated ESDF map
    if ((mapUpdated) && (firstMapReceived*firstPoseReceived*firstGoalsReceived*firstOctomapReceived)) {
      clock_t tStart = clock();
      ROS_INFO("Planning to best goal pose.");
      mapUpdated = false; poseUpdated = false; goalsUpdated = false; octomapUpdated = false; goalPosesUpdated = false; frontierUpdated = false;

      // Project Robot position to speedMap
      ROS_INFO("Projecting robot position onto map...");
      Point robotPosition = {(float)robot.position.x, (float)robot.position.y, (float)robot.position.z};
      // robotPosition = ProjectPointToSpeedMapZ(robotPosition, speedMap);
      robotPosition = ProjectPointToSpeedCloudMap(robotPosition, speedMapCloud, speedSafe + speedMap.voxelSize);
      robot.position.x = robotPosition.x; robot.position.y = robotPosition.y; robot.position.z = robotPosition.z;
      int robotPositionIds[3] = {(int)std::roundf(((float)robot.position.x - speedMap.minBounds.x)/speedMap.voxelSize),
          (int)std::roundf(((float)robot.position.y - speedMap.minBounds.y)/speedMap.voxelSize),
          (int)std::roundf(((float)robot.position.z - speedMap.minBounds.z)/speedMap.voxelSize)};

      // Fast Marching
      ROS_INFO("Calculating cost to goals...");
      MapGrid3D<double> reachMap(voxelSize, speedMap.size, speedMap.minBounds);
      reachMap.SetAll(0.0);
      reachRaw(robotPositionIds, &speedMap, &reachMap, true, true, marchingTimeOut);
      pubReach.publish(ConvertReachMapToPointCloud2(&reachMap));

      // Get paths using gradient following and publish them as one message
      ROS_INFO("Following gradients to paths...");
      pathsMsg.header = pathMsgHeader;
      pathsMsg.poses.clear();
      pathsMsgKinematic.header = pathMsgHeader;
      pathsMsgKinematic.poses.clear();
      std::vector<std::vector<Point>> pathList;
      std::vector<std::vector<Point>> pathListKinematic;
      std::vector<double> costListKinematic;
      for (int i=0; i<goalsCloud->points.size(); i++) {
        Point goal;
        goal.x = goalsCloud->points[i].x; goal.y = goalsCloud->points[i].y; goal.z = goalsCloud->points[i].z;
        std::vector<Point> path = followGradientPath(robotPosition, goal, &reachMap);
        pathList.push_back(path);
        // ROS_INFO("Integrating kinematic model of quad following gradient path %d...", i);
        std::vector<Point> pathKinematic;
        double costKinematic = 0.0;
        if (path.size() > 0) pathKinematic = FollowPathWithKinematics(path, robot, costKinematic);
        else pathKinematic = path;
        pathListKinematic.push_back(pathKinematic);
        costListKinematic.push_back(costKinematic);
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
        if (pathKinematic.size() > 0) {
          // ROS_INFO("Adding kinematic path to ROS msg...");
          nav_msgs::Path newPathSegment = ConvertPointVectorToPathMsg(pathKinematic);
          for (int j=0; j<newPathSegment.poses.size(); j++) {
            pathsMsgKinematic.poses.push_back(newPathSegment.poses[j]);
          }
          pathKinematic = flipPath(pathKinematic);
          newPathSegment = ConvertPointVectorToPathMsg(pathKinematic);
          for (int j=0; j<newPathSegment.poses.size(); j++) {
            pathsMsgKinematic.poses.push_back(newPathSegment.poses[j]);
          }
        }

      }
      ROS_INFO("Paths calculated in %0.3f seconds.", (double)(clock() - tStart)/CLOCKS_PER_SEC);

      pubPaths.publish(pathsMsg);
      pubPathsKinematic.publish(pathsMsgKinematic);
      pubGoals.publish(goalsMsg);
      pubGoalPoses.publish(goalPosesMsg);
      pubOdom.publish(odomMsg);
      pubFrontier.publish(frontierMsg);
      pubOctomap.publish(octomapMsg);

      // Calculate the gain, cost, and utility of each path
      ROS_INFO("Calculating true path stats...");
      tStart = clock();
      std::vector<PathStats> stats;
      for (int i=0; i<pathList.size(); i++) {
        std::vector<Point> currentPath = pathList[i];
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPath (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPose (new pcl::PointCloud<pcl::PointXYZI>);
        PathStats stat;
        stat.costLength  = CalculatePathCost(currentPath);
        stat.costLengthTurn  = CalculatePathCost(currentPath) + CostToTurnPath(robot, currentPath, L1params.yaw_rate_max*(180.0/M_PI));
        stat.costKinematic = costListKinematic[i];
        stat.costReach = reachMap.Query(goalsCloud->points[i].x, goalsCloud->points[i].y, goalsCloud->points[i].z);
        // ROS_INFO("Path %d has cost %0.1f", i, stat.cost);
        clock_t tStartGain = clock();
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPaddedMap (new pcl::PointCloud<pcl::PointXYZI>);
        ConvertCloudForGainCalculation(speedMapCloud, cloudPaddedMap, sensorParams, voxelSize);
        stat.gainPath = GainPath(currentPath, sensorParams, cloudPaddedMap, cloudPath, pubPathCloud, pathMsgHeader, mapTree, voxelSize, "unseen", "normal");
        // ROS_INFO("Gain calculated in %0.3f seconds.", i, (double)(clock() - tStartGain)/CLOCKS_PER_SEC);
        // ROS_INFO("Path %d sees %d points and has gain %0.1f", i, cloudPath->points.size(), stat.gainPath);
        stat.utility = Utility(stat.gainPath, stat.costLength, 0.0, "efficiency");
        // ROS_INFO("Path %d has utility %0.1f", i, stat.utility);
        stat.gainPose = goalsCloud->points[i].intensity;
        geometry_msgs::Pose p = goalPosesMsg.poses[i];
        stat.gainPoseCorrect = GainDebug(p, sensorParams, cloudPaddedMap, cloudPose, pubPathCloud, pathMsgHeader, mapTree, voxelSize, "unseen", "normal");
        // ROS_INFO("Pose %d has gain %0.1f", i, stat.gainPose);
        stats.push_back(stat);
        if (debugMode == "debug") {
          sensor_msgs::PointCloud2 seenCloudMsg;
          pcl::toROSMsg(*cloudPath, seenCloudMsg);
          seenCloudMsg.header = pathMsgHeader;
          pubPathCloud.publish(seenCloudMsg);
          pcl::toROSMsg(*cloudPose, seenCloudMsg);
          seenCloudMsg.header = pathMsgHeader;
          pubPoseCloud.publish(seenCloudMsg);
          nav_msgs::Path newPathMsg;
          newPathMsg = ConvertPointVectorToPathMsg(currentPath);
          newPathMsg.header = pathMsgHeader;
          pubPathCurrent.publish(newPathMsg);
          clock_t tStartKinematic = clock();
          ROS_INFO("Integrating kinematic unicycle model over path %d...", i);
          double costKinematic;
          std::vector<Point> kinematicPath = FollowPathWithKinematics(currentPath, robot, costKinematic);
          newPathMsg = ConvertPointVectorToPathMsg(kinematicPath);
          newPathMsg.header = pathMsgHeader;
          pubPathCurrentKinematic.publish(newPathMsg);
          ROS_INFO("First path unicycle kinematics model integrated in %0.4f seconds.", (double)(clock() - tStartGain)/CLOCKS_PER_SEC);
          sleep(1.0);
        }
      }
      ROS_INFO("True path data calculated in %0.3f seconds.", (double)(clock() - tStart)/CLOCKS_PER_SEC);
      ROS_INFO("Outputting data to file...");
      // Write paths data to file
      std::ofstream f;
      f.open(filename, std::ios::out | std::ios::app);
      double currentTime = ros::Time::now().toSec();
      if (f.is_open()) {
        f << currentTime;
        f << ",";
        for (int i=0; i<stats.size(); i++) {
          f << stats[i].costReach;
          if (i < (stats.size()-1)) f << ",";
          else f << "\n";
        }
        f << currentTime;
        f << ",";
        for (int i=0; i<stats.size(); i++) {
          f << stats[i].costLength;
          if (i < (stats.size()-1)) f << ",";
          else f << "\n";
        }
        f << currentTime;
        f << ",";
        for (int i=0; i<stats.size(); i++) {
          f << stats[i].costLengthTurn;
          if (i < (stats.size()-1)) f << ",";
          else f << "\n";
        }
        f << currentTime;
        f << ",";
        for (int i=0; i<stats.size(); i++) {
          f << stats[i].costKinematic;
          if (i < (stats.size()-1)) f << ",";
          else f << "\n";
        }
        f << currentTime;
        f << ",";
        for (int i=0; i<stats.size(); i++) {
          f << stats[i].gainPose;
          if (i < (stats.size()-1)) f << ",";
          else f << "\n";
        }
        f << currentTime;
        f << ",";
        for (int i=0; i<stats.size(); i++) {
          f << stats[i].gainPoseCorrect;
          if (i < (stats.size()-1)) f << ",";
          else f << "\n";
        }
        f << currentTime;
        f << ",";
        for (int i=0; i<stats.size(); i++) {
          f << stats[i].gainPath;
          if (i < (stats.size()-1)) f << ",";
          else f << "\n";
        }
        f << currentTime;
        f << ",";
        for (int i=0; i<stats.size(); i++) {
          f << stats[i].utility;
          if (i < (stats.size()-1)) f << ",";
          else f << "\n";
        }
      }
      
      f.close();

      
      ROS_INFO("Data outputted, loop complete");
      ROS_INFO("");
    }

  }

  return 0;
}