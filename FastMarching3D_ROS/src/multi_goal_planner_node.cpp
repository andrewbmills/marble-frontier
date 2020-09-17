#include <frontier.h>
#include <ros/ros.h>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <msfm3d/Goal.h>
#include <msfm3d/GoalArray.h>

MapGrid3D<float> costMap;
bool mapUpdated = false;
std_msgs::Header pathMsgHeader;

void CallbackCostMap(const sensor_msgs::PointCloud2::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  delete map;
  map = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
  mapUpdated = true;
  pathMsgHeader = msg->header;
}

// Could put this function inside of mapGrid3D header file
int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_goal_planner");
  ros::NodeHandle n;

  ros::Subscriber subCostMap = n.subscribe("costmap", 1, &CallbackCostMap);
  ros::Subscriber subGoals = n.subscribe("")
  ros::Publisher pubPath = n.advertise<nav_msgs::Path>("path", 5);

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