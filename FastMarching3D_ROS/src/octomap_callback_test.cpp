// C++ Standard Libraries
#include <math.h>
// ROS libraries
#include <ros/ros.h>
// Octomap libaries
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>


// class NodeManager
// {
//   public:
//     octomap::OcTree* map_octree;
//     void callback_octomap(const octomap_msgs::Octomap::ConstPtr msg);
// };

// void NodeManager::callback_octomap(const octomap_msgs::Octomap::ConstPtr msg)
// {
//   ROS_INFO("Occupancy octomap callback called");
//   if (msg->data.size() == 0) return;
//   delete map_octree;
//   map_octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
//   ROS_INFO("Occupancy octomap callback success!");
//   return;
// }


octomap::OcTree* map_octree;
void callback_octomap(const octomap_msgs::Octomap::ConstPtr msg)
{
  ROS_INFO("Occupancy octomap callback called");
  if (msg->data.size() == 0) return;
  delete map_octree;
  map_octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
  ROS_INFO("Occupancy octomap callback success!");
  return;
}

int main(int argc, char **argv)
{
  // Initialize ROS node with name and object instantiation
  ros::init(argc, argv, "octomap_callback_test");
  ros::NodeHandle n;

  // Declare subscribers and publishers
  // NodeManager node_manager;
  // ros::Subscriber sub = n.subscribe("octomap", 1, &NodeManager::callback_octomap, &node_manager);
  ros::Subscriber sub = n.subscribe("octomap", 1, callback_octomap);

  // Declare and read in the node update rate from the launch file parameters
  double updateRate;
  n.param("octomap_callback_test/rate", updateRate, (double)10.0); // Hz
  ros::Rate r(updateRate);

  // Run the node until ROS quits
  while (ros::ok())
  {
    r.sleep(); // Node sleeps to update at a rate as close as possible to the updateRate parameter
    ros::spinOnce(); // All subscriber callbacks are called here.
  }

  return 0;
}
