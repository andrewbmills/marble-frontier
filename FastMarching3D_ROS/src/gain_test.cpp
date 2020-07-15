#include "gain.hpp"
// #include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

class NodeManager
{
  public:
    NodeManager():
      cloud(new pcl::PointCloud<pcl::PointXYZI>)
    {
      map = new octomap::OcTree(0.1);
    }
    bool map_updated = false;
    bool pose_updated = false;
    octomap::OcTree* map;
    Pose pose;
    SensorFoV sensor;
    std_msgs::Header header_map;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    void CallbackOctomap(const octomap_msgs::Octomap::ConstPtr msg);
    void CallbackPose(const geometry_msgs::PoseStamped msg);
    sensor_msgs::PointCloud2 GetCloudMsg();
};

void NodeManager::CallbackOctomap(const octomap_msgs::Octomap::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  delete map;
  map = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
  header_map = msg->header;
  map->expand();
  map_updated = true;
  return;
}

void NodeManager::CallbackPose(const geometry_msgs::PoseStamped msg)
{
  pose.position.x = msg.pose.position.x;
  pose.position.y = msg.pose.position.y;
  pose.position.z = msg.pose.position.z + 0.4;
  pose.q.x = msg.pose.orientation.x;
  pose.q.y = msg.pose.orientation.y;
  pose.q.z = msg.pose.orientation.z;
  pose.q.w = msg.pose.orientation.w;
  pose.R = Quaternion2RotationMatrix(pose.q);
  pose_updated = true;
  return;
}

sensor_msgs::PointCloud2 NodeManager::GetCloudMsg()
{
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header = header_map;
  return msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gain_test_node");
  ros::NodeHandle n;
  ROS_INFO("Node created.");
  NodeManager node_manager;
  ROS_INFO("Node manager initialized.");
  // Subscribers and Publishers
  ros::Subscriber sub_map = n.subscribe("map", 1, &NodeManager::CallbackOctomap, &node_manager);
  ros::Subscriber sub_pose = n.subscribe("pose", 1, &NodeManager::CallbackPose, &node_manager);
  ros::Publisher pub_points = n.advertise<sensor_msgs::PointCloud2>("points", 5);
  ROS_INFO("Subscribers Created.");

  // Params
  n.param("gain_test_node/verticalFoV", node_manager.sensor.verticalFoV, (float)30.0);
  n.param("gain_test_node/horizontalFoV", node_manager.sensor.horizontalFoV, (float)60.0);
  n.param("gain_test_node/range_min", node_manager.sensor.rMin, (float)0.5);
  n.param("gain_test_node/range_max", node_manager.sensor.rMax, (float)5.0);
  n.param<std::string>("gain_test_node/sensor_type", node_manager.sensor.type, "camera");
  ROS_INFO("Params Read.");

  ros::Rate r(1.0); // 1 Hz
  // Main Loop
  ROS_INFO("Starting main loop.");
  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
    if ((node_manager.map_updated) && (node_manager.pose_updated)) {
      ROS_INFO("Calculating view gain and voxels in FoV.");
      node_manager.cloud->points.clear();
      Gain(node_manager.pose, node_manager.sensor, node_manager.map, node_manager.cloud);
      if (node_manager.cloud->points.size() > 0) pub_points.publish(node_manager.GetCloudMsg());
    }
  }
  return 0;
}