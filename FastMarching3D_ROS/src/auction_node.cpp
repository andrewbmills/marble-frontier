#include "auction.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "auction_node");
  ros::NodeHandle n;

  unsigned num_robots;
  n.param("auction_node/num_robots", num_robots, (unsigned)2);

  Auctioneer auctioneer(num_robots);

  ros::Subscriber subOctomapBinary = n.subscribe("goals_0", 1, &Auctioneer::callback_goal_array_robot0, &auctioneer);
  ros::Subscriber subOctomapBinary = n.subscribe("goals_1", 1, &Auctioneer::callback_goal_array_robot1, &auctioneer);
  ros::Subscriber subOctomapBinary = n.subscribe("goals_2", 1, &Auctioneer::callback_goal_array_robot2, &auctioneer);
  ros::Subscriber subOctomapBinary = n.subscribe("goals_3", 1, &Auctioneer::callback_goal_array_robot3, &auctioneer);
  std::vector<ros::Publisher> publishers;
  for (int i=0; i<num_robots; ++i) {
      ros::Publisher pub_goal_robot = n.advertise<geometry_msgs::PoseStamped>("goal_pose" + std::to_string(i), 5);
      ros::Publisher pub_path_robot = n.advertise<geometry_msgs::PoseStamped>("planned_path" + std::to_string(i), 5);
      publishers.push_back(pub_goal_robot);
      publishers.push_back(pub_path_robot);
  }

  float update_rate;
  n.param("auction_node/update_rate", update_rate, (float)1.0);
  
  ros::Rate r(update_rate); // 5 Hz
  // Main Loop
  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
    if (auctioneer.check_if_goals_updated()) {
        std::vector<msfm3d::Goal> goals;
        auctioneer.auction_goals(goals);
        for (int i=0; i<num_robots; ++i) {
            publishers[2*i].publish(goals[i].pose);
            publishers[2*i+1].publish(goals[i].path);
        }
    }
  }

  return 0;
}