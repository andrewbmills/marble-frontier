#include <frontier.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <algorithm>

octomap::OcTree* map;
bool mapUpdated = false;
std_msgs::Header frontierMsgHeader;
pcl::PointCloud<pcl::PointXYZI>::Ptr edtCloud (new pcl::PointCloud<pcl::PointXYZI>);

void CallbackOctomapBinary(const octomap_msgs::Octomap::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  delete map;
  map = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
  mapUpdated = true;
  frontierMsgHeader = msg->header;
  return;
}

void CallbackOctomapFull(const octomap_msgs::Octomap::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  delete map;
  map = (octomap::OcTree*)octomap_msgs::fullMsgToMap(*msg);
  mapUpdated = true;
  frontierMsgHeader = msg->header;
  return;
}

void CallbackEDTPointCloud(const sensor_msgs::PointCloud2::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  pcl::fromROSMsg(*msg, *edtCloud);
  mapUpdated = true;
  frontierMsgHeader = msg->header;
  return;
}
// Could put this function inside of mapGrid3D header file
int main(int argc, char **argv)
{
  ros::init(argc, argv, "frontier_node");
  ros::NodeHandle n;

  ros::Subscriber subOctomapBinary = n.subscribe("octomap_binary", 1, &CallbackOctomapBinary);
  ros::Subscriber subOctomapFull = n.subscribe("octomap_full", 1, &CallbackOctomapFull);
  ros::Subscriber subEDTPointCloud = n.subscribe("edt", 1, &CallbackEDTPointCloud);
  ros::Publisher pubFrontier = n.advertise<sensor_msgs::PointCloud2>("frontier", 5);
  sensor_msgs::PointCloud2 frontierMsg;

  // Params
  std::string mapType;
  n.param<std::string>("frontier_node/map_type", mapType, "octomap");
  // n.param<std::string>("frontier_node/map_type", mapType, "edt");
  // n.param<std::string>("frontier_node/map_type", mapType, "edt_ground");
  int clusterSize;
  n.param("frontier_node/cluster_size", clusterSize, 30);
  float voxelSize, normalZ;
  n.param("frontier_node/voxel_size", voxelSize, (float)0.2);
  n.param("frontier_node/normal_z_filter", normalZ, (float)0.4);
  bool filterByNormal, filterByCluster;
  n.param("frontier_node/filterByNormal", filterByNormal, true);
  n.param("frontier_node/filterByCluster", filterByCluster, true);

  float update_rate;
  n.param("frontier_node/update_rate", update_rate, (float)1.0);
  
  ros::Rate r(update_rate); // 5 Hz
  ROS_INFO("Finished reading params.");
  // Main Loop
  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
    if (mapUpdated) {
      mapUpdated = false;
      clock_t tStart = clock();
      Frontier frontier;
      if (mapType == "octomap") frontier = CalculateFrontier(map, filterByNormal, filterByCluster, normalZ, clusterSize);
      else if (mapType == "edt") frontier = CalculateFrontier(edtCloud, voxelSize, filterByNormal, filterByCluster, normalZ, clusterSize);
      else if (mapType == "edt_ground") frontier = CalculateFrontier(edtCloud, voxelSize, filterByNormal, filterByCluster, normalZ, clusterSize, true);
      ROS_INFO("Frontier calculated in: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);
      tStart = clock();
      frontierMsg = ConvertFrontierToROSMsg(frontier);
      ROS_INFO("Converted frontier to ROS message in: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);
      frontierMsg.header = frontierMsgHeader;
      pubFrontier.publish(frontierMsg);
    }
  }

  return 0;
}