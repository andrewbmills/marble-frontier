#include <frontier.h>
#include <ros/ros.h>
#include <algorithm>

octomap::OcTree* map;
bool mapUpdated = false;
std_msgs::Header frontierMsgHeader;

void CallbackOctomapBinary(const octomap_msgs::Octomap::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  delete map;
  map = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
  mapUpdated = true;
  frontierMsgHeader = msg->header;
}

void CallbackOctomapFull(const octomap_msgs::Octomap::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  delete map;
  map = (octomap::OcTree*)octomap_msgs::fullMsgToMap(*msg);
  mapUpdated = true;
  frontierMsgHeader = msg->header;
}

// Could put this function inside of mapGrid3D header file
int main(int argc, char **argv)
{
  ros::init(argc, argv, "goal_pose_sampler");
  ros::NodeHandle n;

  ros::Subscriber subOctomapBinary = n.subscribe("octomap_binary", 1, &CallbackOctomapBinary);
  ros::Subscriber subOctomapFull = n.subscribe("octomap_full", 1, &CallbackOctomapFull);
  ros::Publisher pubFrontier = n.advertise<sensor_msgs::PointCloud2>("frontier", 5);
  sensor_msgs::PointCloud2 frontierMsg;

  // Params
  float update_rate;
  n.param("goal_pose_sampler/update_rate", update_rate, (float)1.0);
  
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
      Frontier frontier = CalculateFrontier(map);
      ROS_INFO("Frontier calculated in: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);
      tStart = clock();
      frontierMsg = ConvertFrontierToROSMsg(frontier);
      ROS_INFO("Converted frontier to ROS message in: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);
      frontierMsg.header = frontierMsgHeader;
    }
    pubFrontier.publish(frontierMsg);
  }

  return 0;
}