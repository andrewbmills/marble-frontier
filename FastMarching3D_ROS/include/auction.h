#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <algorithm>
#include <string>
#include "GoalArray.h"

class Auctioneer {
    private:
        unsigned m_num_robots;
        std::vector<msfm3d::GoalArray> m_goal_sets;
        std::vector<msfm3d::Goal> m_best_goals;
        bool m_goals_updated = false;

    public:
        Auctioneer(unsigned t_num_robots);
        void auction_goals(std::vector<msfm3d::Goal> &goals);
        bool check_if_goals_updated();
        void callback_goal_array_robot0(const msfm3d::GoalArray::ConstPtr msg);
        void callback_goal_array_robot1(const msfm3d::GoalArray::ConstPtr msg);
        void callback_goal_array_robot2(const msfm3d::GoalArray::ConstPtr msg);
};

Auctioneer::Auctioneer(unsigned t_num_robots)
{
    m_num_robots = t_num_robots;
    m_goal_sets = std::vector<msfm3d::GoalArray>(m_num_robots);
    return;
}

void Auctioneer::auction_goals(std::vector<msfm3d::Goal> &goals)
{
    if (goals.size() < m_num_robots) goals.resize(m_num_robots);
    // Do the auction

    

    m_goals_updated = false;
    return;
}

bool Auctioneer::check_if_goals_updated()
{
    return m_goals_updated;
}

void Auctioneer::callback_goal_array_robot0(const msfm3d::GoalArray::ConstPtr msg)
{
    if (msg->goals.size() == 0) return;
    m_goal_sets[0] = *msg;
    m_goals_updated = true;
    return;
}