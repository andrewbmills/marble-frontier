#include <math.h>
// ROS Libraries
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

struct Point{
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
};

// Holder class for params, callback, and published msg
class LookaheadFinder
{
  public:
    bool reverse = false;
    float R = 1.0; // meters
    std::string vehicle_type = "ground";
    std::string fixed_frame = "world";
    float position[3] = {0.0, 0.0, 0.0};
    geometry_msgs::Path path = geometry_msgs::Path();
    geometry_msgs::PointStamped lookahead_point_msg = geometry_msgs::PointStamped();
    visualization_msgs::Marker lookahead_vec_msg = visualization_msgs::Marker();
    void callbackPath(const geometry_msgs::Path msg);
    void callbackOdometry(const nav_msgs::Odometry msg)
};

void LookaheadFinder::callbackPath(const geometry_msgs::Path msg)
{
  if (msg.poses.size() == 0) return;
  path = msg;
  return;
}

void LookaheadFinder::callbackOdometry(const nav_msgs::Odometry msg)
{
  point[0] = msg.pose.pose.position.x;
  point[1] = msg.pose.pose.position.y;
  point[2] = msg.pose.pose.position.z;
  return; 
}

Point find_Lookahead_Discrete_2D(const geometry_msgs::Path path, const Point p, const float R, const bool reverse)
{
  // Determine the last path index
  int i_max = path.poses.size()-1;
  // Loop initialization params
  bool intersection = false;
  float t_hat = -1.0;
  int i_closest = 0;
  float c_closest = 100.0*R;

    // while (t_hat < 0) or (t_hat > 1):
  for (int i=0; i<i_max; i++) {

    // Initialize Vectors in intersection geometry
    //     p1 = np.array([path[0,i], path[1,i]])
    //     p2 = np.array([path[0,i+1], path[1,i+1]])
    //     d = p2 - p1
    //     q = p1 - p_AC
        
    //     # Calculate coefficients for polynomial to solve for t_hat
    //     # Solve quadratic polynomial equation: a*t_hat^2 + b*t_hat + c = 0
    //     a = d[0]*d[0] + d[1]*d[1]
    //     b = 2*(d[0]*q[0] + d[1]*q[1])
    //     c = q[0]*q[0] + q[1]*q[1] - R*R
    //     discriminant = b*b - 4*a*c
    //     if (discriminant <= 0):
    //         # no intersection
    //         t_hat = -1
    //     else:
    //         # 1 or 2 solutions exist (numerically has to be 2)
    //         t_hat1 = (-b + math.sqrt(discriminant)) / (2*a)
    //         t_hat2 = (-b - math.sqrt(discriminant)) / (2*a)

    //         # Take larger of two intersections, furthest point down the path
    //         if (reverse):
    //             t_hat = min(t_hat1, t_hat2)
    //         else:
    //             t_hat = max(t_hat1, t_hat2)
    //     # If p1 is the closest point to p_AC, store it
    //     if (c < c_closest):
    //         c_closest = c
    //         i_closest = i
    //     # Next iteration
    //     i = i + 1
    // }
  }
    // # Check for intersection of circle around p_AC with path
    // if intersection:
    //     # Calculate lookahead point and its local tangent vector
    //     p_L2 = d*t_hat + p1
    //     v_L2 = d / np.linalg.norm(d)
    //     i_cut = i
    // else:
    //     # Calculate lookahead point and its local tangent vector
    //     # Set lookahead point to the closest path node
    //     p_L2 = np.array([path[0,i_closest], path[1,i_closest]])
    //     v_L2 = np.array([path[0,i_closest+1], path[1,i_closest+1]]) - p_L2
    //     v_L2 = v_L2 / np.linalg.norm(v_L2)  # Normalize the vector
    //     i_cut = i_closest
    
  return p_L2, v_L2
}


int main(int argc, char **argv)
{
  // Node declaration
  ros::init(argc, argv, "lookahead_finderr");
  ros::NodeHandle n;

  LookaheadFinder finder;

  // Subscribers and Publishers
  ros::Subscriber sub = n.subscribe("path", 1, &LookaheadFinder::callbackPath, &finder);
  ros::Publisher pub1 = n.advertise<geometry_msgs::PointStamped>("lookahead_point", 5);
  ros::Publisher pub2 = n.advertise<visualization_msgs::Marker>("lookahead_vec", 5);

  // Params
  n.param("lookahead_finder/min_cluster_size", finder.min_cluster_size, 100);
  n.param("lookahead_finder/normal_z_threshold", finder.normal_z_threshold, (float)0.8);
  n.param("lookahead_finder/vertical_padding", finder.vertical_padding, 2);

  float update_rate;
  n.param("ground_finder/update_rate", update_rate, (float)5.0);
  ros::Rate r(update_rate); // 5 Hz

  // Main Loop
  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
    if (finder.ground_msg.data.size() > 0) pub1.publish(finder.ground_msg);
    if (finder.edt_msg.data.size() > 0) pub2.publish(finder.edt_msg);
  }
}