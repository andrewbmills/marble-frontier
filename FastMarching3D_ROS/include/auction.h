#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <algorithm>
#include <string>
#include "math.h"
#include "GoalArray.h"

float calculate_distance(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b)
{
    geometry_msgs::Point p_a = a.pose.position;
    geometry_msgs::Point p_b = b.pose.position;
    float distance = std::sqrt((p_b.x - p_a.x)*(p_b.x - p_a.x) +
                               (p_b.y - p_a.y)*(p_b.x - p_a.y) +
                               (p_b.z - p_a.z)*(p_b.x - p_a.z));
    return distance;
}

class Auctioneer {
    private:
        unsigned m_num_robots;
        std::vector<msfm3d::GoalArray::ConstPtr> m_goal_sets;
        std::vector<msfm3d::Goal> m_best_goals;
        bool m_goals_updated = false;
        float m_deconflict_radius = 5.0; // meters
    public:
        Auctioneer(unsigned t_num_robots);
        void auction_goals(std::vector<msfm3d::Goal> &goals);
        bool check_if_goals_updated();
        void callback_goal_array_robot(const msfm3d::GoalArray::ConstPtr msg, unsigned robot_id);
        void callback_goal_array_robot0(const msfm3d::GoalArray::ConstPtr msg);
        void callback_goal_array_robot1(const msfm3d::GoalArray::ConstPtr msg);
        void callback_goal_array_robot2(const msfm3d::GoalArray::ConstPtr msg);
        void callback_goal_array_robot3(const msfm3d::GoalArray::ConstPtr msg);
};

Auctioneer::Auctioneer(unsigned t_num_robots)
{
    m_num_robots = t_num_robots;
    m_goal_sets.resize(m_num_robots);
    return;
}

void Auctioneer::auction_goals(std::vector<msfm3d::Goal> &goals)
{
    if (goals.size() < m_num_robots) goals.resize(m_num_robots);

    for (int i=0; i<m_num_robots; ++i) {
        unsigned best_goal_id = 0;
        goals[i] = m_goal_sets[i]->goals[best_goal_id];
        for (int j=0; i<m_num_robots; ++j) {
            msfm3d::Goal neighbor_goal = m_goal_sets[j]->goals[0];
            if (calculate_distance(goals[i].pose, neighbor_goal.pose) < m_deconflict_radius && 
                neighbor_goal.cost.data < goals[i].cost.data) {
                best_goal_id++;
                goals[i] = m_goal_sets[i]->goals[(best_goal_id < m_goal_sets[i]->goals.size())*best_goal_id];
            }
        }
    }
    m_goals_updated = false;
    return;
}

bool Auctioneer::check_if_goals_updated()
{
    return m_goals_updated;
}

void Auctioneer::callback_goal_array_robot(const msfm3d::GoalArray::ConstPtr msg, unsigned robot_id)
{
    if (msg->goals.size() == 0) return;
    m_goal_sets[robot_id] = msg;
    m_goals_updated = true;
    return;
}