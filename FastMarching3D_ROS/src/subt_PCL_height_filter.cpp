// g++ subt_PCL_height_filter.cpp -g -o subt_PCL_height_filter.o -I /opt/ros/melodic/include -I /usr/include/c++/7.3.0 -I /usr/include/eigen3 -I /usr/include/pcl-1.8 -I /home/andrew/catkin_ws/devel/include  -I /usr/local/include -L /usr/lib/x86_64-linux-gnu -L /home/andrew/catkin_ws/devel/lib -L /opt/ros/melodic/lib -L /usr/lib -L /usr/local/lib -Wl,-rpath,opt/ros/melodic/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization -lboost_system -lpcl_common -lpcl_io -lpcl_filters

// C++ Standard Libraries
#include <math.h>
// ROS libraries
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// pcl
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
// pcl ROS
#include <pcl_conversions/pcl_conversions.h>

class PC2_height_filter
{
  public:
    // Constructor

    // Property Definitions

      // Storing pcl2 message for publishing
      sensor_msgs::PointCloud2 filteredMsg;

      // Height difference to filter out all cloud pts below
      double z_min = 0.0;
      double z_max = 1000.0;

    // Method Definitions
    void callback_cloud(const sensor_msgs::PointCloud2ConstPtr& msg);
};

void PC2_height_filter::callback_cloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // ROS_INFO("PC2 recieved.  Filtering all detections below z = %0.2f meters.", z_min);

  // Convert from ROS PC2 msg to PCL object
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *inputCloud);

  // Iterate through inputCloud and store all points within height range to the outputCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(inputCloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_min, z_max);
  //pass.setFilterLimitsNegative (true);
  pass.filter(*outputCloud);


  outputCloud->is_dense = true;

  // Convert back to PC2 ROS msg
  sensor_msgs::PointCloud2 newPointCloud2;
  pcl::toROSMsg(*outputCloud, newPointCloud2);

  // Set Header params
  newPointCloud2.header.seq = 1;
  newPointCloud2.header.stamp = msg->header.stamp;
  newPointCloud2.header.frame_id = msg->header.frame_id;

  // Copy to publishable object property
  filteredMsg = newPointCloud2;
}

int main(int argc, char **argv)
{
  // Initialize ROS node with name and object instantiation
  ros::init(argc, argv, "filter_PC2_height");
  ros::NodeHandle n;

  PC2_height_filter PC2_filter;

  // Declare subscribers for the camera info and the raw depth image.  Use the image truncator callbacks
  ros::Subscriber sub1 = n.subscribe("X1/points", 1, &PC2_height_filter::callback_cloud, &PC2_filter);

  ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("X1/points_filtered", 5);

  // Get depth camera min and max range values
  double z_filter;
  n.param("z_filter", z_filter, (double)(-0.55));
  PC2_filter.z_min = z_filter;

  // Declare and read in the node update rate from the launch file parameters
  double updateRate;
  n.param("filter_PC2_height/updateRate", updateRate, (double)10.0); // Hz
  ros::Rate r(updateRate);

  // Run the node until ROS quits
  while (ros::ok())
  {
    r.sleep(); // Node sleeps to update at a rate as close as possible to the updateRate parameter
    ros::spinOnce(); // All subscriber callbacks are called here.
    pub1.publish(PC2_filter.filteredMsg);
  }

  return 0;
}