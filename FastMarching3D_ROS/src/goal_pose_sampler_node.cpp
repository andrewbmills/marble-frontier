#include <poseSampling.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <algorithm>

octomap_msgs::Octomap::ConstPtr octomapMsg;
bool mapUpdated = false;
bool edtUpdated = false;
bool frontierUpdated = false;
bool firstPose = false;
std_msgs::Header goalsMsgHeader;
pcl::PointCloud<pcl::PointXYZI>::Ptr edtCloud (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZLNormal>::Ptr frontierCloud (new pcl::PointCloud<pcl::PointXYZLNormal>);
pcl::PointCloud<pcl::PointXYZLNormal>::Ptr frontierOld (new pcl::PointCloud<pcl::PointXYZLNormal>);
std::vector<pcl::PointIndices> clusterIndices;
Pose robot;
std::vector<View> goals;

void CallbackOctomapBinary(const octomap_msgs::Octomap::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  ROS_INFO("Reading octomap msg...");
  octomapMsg = msg;
  mapUpdated = true;
  return;
}

void CallbackOctomapFull(const octomap_msgs::Octomap::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  ROS_INFO("Reading octomap msg...");
  octomapMsg = msg;
  mapUpdated = true;
  return;
}

void CallbackEDTPointCloud(const sensor_msgs::PointCloud2::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  ROS_INFO("Reading in EDT PointCloud...");
  pcl::fromROSMsg(*msg, *edtCloud);
  edtUpdated = true;
  return;
}

void ParseFrontierClusters()
{
  clusterIndices.clear();
  int num_clusters = 0;
  for (int i=0; i<frontierCloud->points.size(); i++){
    pcl::PointXYZLNormal p = frontierCloud->points[i];
    int cluster = (int)p.label;
    if ((cluster+1) > num_clusters) {
      for (int i=num_clusters; i<(cluster+1); i++){
        pcl::PointIndices newCluster;
        clusterIndices.push_back(newCluster);
      }
      num_clusters = (cluster+1);
    }
    clusterIndices[cluster].indices.push_back(i);// cluster id is stored in the label field
  }
}

void CallbackFrontier(const sensor_msgs::PointCloud2::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  ROS_INFO("Frontier callback...");
  pcl::fromROSMsg(*msg, *frontierCloud);
  ROS_INFO("Parsing Frontier into clusters...");
  ParseFrontierClusters();
  ROS_INFO("Success!");
  frontierUpdated = true;
  goalsMsgHeader = msg->header;
  return;
}

void CallbackPose(const geometry_msgs::PoseStamped msg)
{
  robot.position.x = msg.pose.position.x;
  robot.position.y = msg.pose.position.y;
  robot.position.z = msg.pose.position.z;
  robot.q.x = msg.pose.orientation.x;
  robot.q.y = msg.pose.orientation.y;
  robot.q.z = msg.pose.orientation.z;
  robot.q.w = msg.pose.orientation.w;
  firstPose = true;
  return;
}

void CallbackOdometry(const nav_msgs::Odometry msg)
{
  ROS_INFO("Received updated vehicle pose.");
  robot.position.x = msg.pose.pose.position.x;
  robot.position.y = msg.pose.pose.position.y;
  robot.position.z = msg.pose.pose.position.z;
  robot.q.x = msg.pose.pose.orientation.x;
  robot.q.y = msg.pose.pose.orientation.y;
  robot.q.z = msg.pose.pose.orientation.z;
  robot.q.w = msg.pose.pose.orientation.w;
  firstPose = true;
  return;
}

void LoadFibonacci50()
{
  fibonacciSphere50 << 0.98,0.072111692630594,-0.185472110534039,
    0.94,-0.30595937918124,0.150959790311958,
    0.9,0.418517327009501,0.121832864994721,
    0.86,-0.264933775465341,-0.43613082282462,
    0.82,-0.111320249301854,0.561433702315218,
    0.78,0.504380591080092,-0.370405479632385,
    0.74,-0.668673358221108,-0.0726356662067993,
    0.7,0.471412530700303,0.536442192504221,
    0.66,0.0155236451629398,-0.751105196654141,
    0.62,-0.541832329175321,0.567466058069068,
    0.58,0.812795688438898,-0.0544349966027162,
    0.54,-0.657221652859273,-0.525794350495443,
    0.5,0.133192374223317,0.855721795590574,
    0.46,0.491949917567831,-0.739178786630815,
    0.42,-0.881090807080997,0.217437323560969,
    0.38,0.811893086846763,0.443203808118155,
    0.34,-0.304280186736367,-0.889839068573459,
    0.3,-0.38212419102167,0.874060125298045,
    0.26,0.882854951181119,-0.391110131772349,
    0.22,-0.924555559380856,-0.311122190815685,
    0.18,0.475526459442382,0.861089186072031,
    0.14,0.232541985119214,-0.962457388748622,
    0.1,-0.825612568952115,0.55530521876558,
    0.06,0.987059408180681,0.148706841544036,
    0.02,-0.628383482417565,-0.777646577196077,
    -0.0200000000000001,-0.0619422918284693,0.997879327615838,
    -0.0600000000000001,0.71857916320349,-0.692852066612904,
    -0.1,-0.994662865198296,0.0254122922129577,
    -0.14,0.746951063378237,0.649972390889122,
    -0.18,-0.110996672397271,-0.977384130583637,
    -0.22,-0.573566192213418,0.78906389041053,
    -0.26,0.946241986632696,-0.192421679478712,
    -0.3,-0.817705483480962,-0.491281734125304,
    -0.34,0.267254131911314,0.901651389937558,
    -0.38,0.405228219178532,-0.831498701370842,
    -0.42,-0.844227536614828,0.332986285635399,
    -0.46,0.829129003680105,0.317718578708323,
    -0.5,-0.38697520002379,-0.774758152307252,
    -0.54,-0.231303504777414,0.80925811004752,
    -0.58,0.694153323341112,-0.426322839752328,
    -0.62,-0.770354364362138,-0.148842713319149,
    -0.66,0.447630630881683,0.603346350197353,
    -0.7,0.0736568579149558,-0.71033419408198,
    -0.74,-0.503069704462769,0.446453662155148,
    -0.78,0.625701539137303,0.00987845743070272,
    -0.82,-0.415887241337377,-0.393240133369912,
    -0.86,0.0365826888707612,0.50898104765795,
    -0.9,0.270639983440429,-0.341692843594015,
    -0.94,-0.33685530847208,0.0541156276336163,
    -0.98,0.166198214968986,0.109444750176164;
  return;
}

// Could put this function inside of mapGrid3D header file
int main(int argc, char **argv)
{
  ros::init(argc, argv, "goal_pose_sampler");
  ros::NodeHandle n;

  ros::Subscriber subFrontierPointCloud = n.subscribe("frontier", 1, &CallbackFrontier);
  ros::Subscriber subPose = n.subscribe("pose", 1, &CallbackPose);
  ros::Subscriber subOdometry = n.subscribe("odometry", 1, &CallbackOdometry);
  ros::Subscriber subEDTPointCloud = n.subscribe("edt", 1, &CallbackEDTPointCloud);
  ros::Subscriber subOctomapBinary = n.subscribe("octomap_binary", 1, &CallbackOctomapBinary);
  ros::Subscriber subOctomapFull = n.subscribe("octomap_full", 1, &CallbackOctomapFull);
  // ros::Subscriber subReSampleBool = n.subscribe("resample_trigger", 1, &CallbackResampleBool);
  ros::Publisher pubGoalPoses = n.advertise<sensor_msgs::PointCloud2>("goals", 5);
  ros::Publisher pubGoalPoseArray = n.advertise<geometry_msgs::PoseArray>("goal_poses", 5);
  ros::Publisher pubFrontierDiff = n.advertise<sensor_msgs::PointCloud2>("frontier_diff", 5);
  ros::Publisher pubFrontierDiffClusters = n.advertise<sensor_msgs::PointCloud2>("frontier_diff_clusters", 5);
  ros::Publisher pubGoalCloudCurrentDebug = n.advertise<sensor_msgs::PointCloud2>("debug_goal_cloud", 5);
  ros::Publisher pubMapCloudDebug = n.advertise<sensor_msgs::PointCloud2>("debug_map_cloud", 5);
  sensor_msgs::PointCloud2 goalsMsg;
  geometry_msgs::PoseArray goalsPoseArrayMsg;

  // Params
  float voxelSize, robotProximityFilterRadius;
  n.param("goal_pose_sampler/voxel_size", voxelSize, (float)0.2);
  n.param("goal_pose_sampler/robot_proximity_filter_radius", robotProximityFilterRadius, (float) 1.0);
  float groupRadius, minObstacleProximity;
  n.param("goal_pose_sampler/group_radius", groupRadius, (float)4.1*voxelSize);
  n.param("goal_pose_sampler/min_obstacle_proximity", minObstacleProximity, (float)4.0*voxelSize);
  int sampleLimit;
  n.param("goal_pose_sampler/pose_sampling_limit", sampleLimit, 100);
  SensorFoV robotSensor;
  n.param("goal_pose_sampler/sensor_horizontal_FoV", robotSensor.horizontalFoV, (float)80.0);
  n.param("goal_pose_sampler/sensor_vertical_FoV", robotSensor.verticalFoV, (float)40.0);
  n.param("goal_pose_sampler/sensor_rMin", robotSensor.rMin, (float)1.0);
  n.param("goal_pose_sampler/sensor_rMax", robotSensor.rMax, (float)3.0);
  n.param<std::string>("goal_pose_sampler/sensor_type", robotSensor.type, "camera");
  std::string sampleMode, gainType, gainDebugMode;
  n.param<std::string>("goal_pose_sampler/sample_mode", sampleMode, "gaussian");
  n.param<std::string>("goal_pose_sampler/gain_type", gainType, "frontier");
  n.param<std::string>("goal_pose_sampler/gain_debug_mode", gainDebugMode, "normal");
  bool checkLoS;
  n.param("goal_pose_sampler/check_LoS", checkLoS, true);

  float update_rate;
  n.param("goal_pose_sampler/update_rate", update_rate, (float)1.0);
  ros::Rate r(update_rate); // 5 Hz
  ROS_INFO("Finished reading params.");

  // Import Fibonacci Sphere coordinates for sampling
  LoadFibonacci50();

  // Main Loop
  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
    if (frontierUpdated && mapUpdated && edtUpdated) {
      ROS_INFO("Frontier, map and EDT have been updated.");
      frontierUpdated = false;
      mapUpdated = false;
      edtUpdated = false;
      clock_t tStart = clock();
      ROS_INFO("Converting octomap msg to octomap...");
      octomap::OcTree* map = (octomap::OcTree*)octomap_msgs::fullMsgToMap(*octomapMsg);
      // map->expand();
      MapGrid3D<float> edtGrid;
      edtGrid.voxelSize = voxelSize;
      ROS_INFO("Converting edt pointcloud of size %d to gridmap...", (int)edtCloud->points.size());
      ConvertPointCloudToEDTGrid(edtCloud, &edtGrid);
      ROS_INFO("Success! EDT map created with bounds (%0.1f, %0.1f, %0.1f) to (%0.1f, %0.1f, %0.1f) ...", edtGrid.minBounds.x, 
        edtGrid.minBounds.y, edtGrid.minBounds.z, edtGrid.maxBounds.x, edtGrid.maxBounds.y, edtGrid.maxBounds.z);
      // Get new frontier pointcloud
      pcl::PointCloud<pcl::PointXYZLNormal>::Ptr frontierDiff (new pcl::PointCloud<pcl::PointXYZLNormal>); // current frontiers that weren't previously
      std::vector<pcl::PointIndices> clusterIndicesDiff;
      goals = FilterGoalsNewFrontier(goals, voxelSize, frontierCloud);
      GetFrontierDiff(frontierOld, frontierCloud, voxelSize, frontierDiff, clusterIndicesDiff);
      ROS_INFO("%d goals remaining after frontier difference checking.", (int)goals.size());
      if (frontierDiff->points.size() < 100) {
        ROS_INFO("Frontier cloud is unchanged.");
        goalsMsg = ConvertGoalsToPointCloud2(goals);
        goalsMsg.header = goalsMsgHeader;
        pubGoalPoses.publish(goalsMsg);
        goalsPoseArrayMsg = ConvertGoalsToPoseArray(goals);
        goalsPoseArrayMsg.header = goalsMsgHeader;
        pubGoalPoseArray.publish(goalsPoseArrayMsg);
        ROS_INFO("***_____________________________________***");
        delete map;
        continue;
      }

      // Group frontiers within clusters that have had one of their frontiers change.
      // Resample goal poses on these clusters.
      // ROS_INFO("Grouping frontiers based on proximity...");
      std::vector<Group> groups;
      greedyGrouping(groupRadius, frontierDiff, clusterIndicesDiff, groups);
      // ROS_INFO("Success!");
      // // Sample goal poses
      // ROS_INFO("Converting edt pointcloud to gridmap...");
      std::vector<View> goalsNew = SampleGoals(groups, robotSensor, edtGrid, map, sampleLimit, minObstacleProximity, sampleMode);

      // Find the goals whose gains need to be updated and add them to the goalsNew vector
      pcl::PointCloud<pcl::PointXYZLNormal>::Ptr frontierDiffOld (new pcl::PointCloud<pcl::PointXYZLNormal>); // voxels that used to be frontiers
      std::vector<pcl::PointIndices> clusterIndicesDiffOld;
      GetFrontierDiff(frontierCloud, frontierOld, voxelSize, frontierDiffOld, clusterIndicesDiffOld);
      std::vector<View> goalsUpdateGain = FindGoalsCloseToCloud(goals, frontierDiffOld, robotSensor.rMax);
      goalsNew.insert(goalsNew.end(), goalsUpdateGain.begin(), goalsUpdateGain.end());

      ROS_INFO("Frontier diff created, groups made, and goal poses sampled in: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);

      tStart = clock();
      // Get a pointcloud of the map within a sensor range box of all the newly sampled goal poses
      pcl::PointCloud<pcl::PointXYZI>::Ptr mapCloud (new pcl::PointCloud<pcl::PointXYZI>);
      FilterCloudNearGoals(edtCloud, mapCloud, goalsNew, robotSensor, voxelSize, frontierCloud, gainType);
      sensor_msgs::PointCloud2 mapCloudMsg;
      pcl::toROSMsg(*mapCloud, mapCloudMsg);
      mapCloudMsg.header = goalsMsgHeader;
      pubMapCloudDebug.publish(mapCloudMsg);

      // Calculate gains for each goal pose
      ROS_INFO("Calculating gains for %d goal poses...", (int)goalsNew.size());
      for (int i=0; i<goalsNew.size(); i++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr seenCloud (new pcl::PointCloud<pcl::PointXYZI>);
        geometry_msgs::Pose goalPose = ConvertPoseToGeometryMsgPose(goalsNew[i].pose);
        goalsNew[i].gain = GainDebug(goalPose, robotSensor, mapCloud, seenCloud, pubGoalCloudCurrentDebug, goalsMsgHeader, map, voxelSize, gainType, gainDebugMode, checkLoS);
        for (int j=0; j<seenCloud->points.size(); j++) {
          goalsNew[i].cloud.points.push_back(seenCloud->points[j]);
          pcl::toROSMsg(*seenCloud, goalsNew[i].cloud_msg);
        }
      }

      // Append goals vector with the newly sampled goals
      goals.insert(goals.end(), goalsNew.begin(), goalsNew.end());

      ROS_INFO("Gains calculated in: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);
      goalsMsg = ConvertGoalsToPointCloud2(goals);
      goalsMsg.header = goalsMsgHeader;
      pubGoalPoses.publish(goalsMsg);

      goalsPoseArrayMsg = ConvertGoalsToPoseArray(goals);
      goalsPoseArrayMsg.header = goalsMsgHeader;
      pubGoalPoseArray.publish(goalsPoseArrayMsg);

      sensor_msgs::PointCloud2 frontierDiffMsg;
      pcl::toROSMsg(*frontierDiff, frontierDiffMsg);
      frontierDiffMsg.header = goalsMsgHeader;
      pubFrontierDiff.publish(frontierDiffMsg);

      // // Copy frontierCloud to frontierCloudOld
      frontierOld->points.clear();
      pcl::copyPointCloud(*frontierCloud, *frontierOld);

      // Delete ocTree to prevent memory leaks
      delete map;
      ROS_INFO("***_____________________________________***");
    }
  }

  return 0;
}