// C++ Standard Libraries
#include <math.h>
// ROS libraries
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
// pcl
#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/crop_box.h>
// Octomap libaries
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
// pcl ROS
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>


#define PI 3.14159265

class PC2_normal_filter
{
  public:
    // Constructor
    PC2_normal_filter(double voxel_size)
    {
      mytree = new octomap::OcTree(voxel_size);
    }

    // Property Definitions

      // Storing pcl2 message for publishing
      sensor_msgs::PointCloud2 normalsMsg;
      sensor_msgs::PointCloud2 filteredHitMsg;
      sensor_msgs::PointCloud2 filteredMissMsg;

      // Radius around each point to regress the normal
      double radiusSearch = 0.2; // meters

      // Any normal with a value above this one is traversable
      double z_nFilter = 0.85; // acos(slope)

      // Radius filter
      double rFilter = 2.5; // meters

      // Octree for holding traversability map
      octomap::OcTree* mytree; // OcTree object for holding Octomap

      // World frame
      std::string fixedFrameId = "world";

      // TF Listener
      tf::TransformListener tfListener;

      octomap_msgs::Octomap octomapMsg;
      bool pubMap = false;

    // Method Definitions
    void callback_cloud(const sensor_msgs::PointCloud2ConstPtr& msg);
};

void PC2_normal_filter::callback_cloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (msg->data.size() == 0) return;
  // ROS_INFO("PC2 recieved.  Calculating normals...");

  // Convert from ROS PC2 msg to PCL object
  pcl::PointCloud<pcl::PointXYZ>::Ptr msgCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
  // ROS_INFO("Converting ROS message into PointCloud object...");
  pcl::fromROSMsg(*msg, *msgCloud);
  // ROS_INFO("Conversion done!");

  // for (int i=0; i<50; i++) {
    // ROS_INFO("Lidar detection @ (%0.1f, %0.1f, %0.1f)", inputCloud->points[i].x, inputCloud->points[i].y, inputCloud->points[i].z);
  // }

  // Run cropbox filter to minimize compute
  pcl::CropBox<pcl::PointXYZ> cb;
  cb.setMin(Eigen::Vector4f(-rFilter, -rFilter, -rFilter, 1.0));
  cb.setMax(Eigen::Vector4f(rFilter, rFilter, rFilter, 1.0));
  cb.setInputCloud(msgCloud);
  cb.filter(*inputCloud);

  // Initialize output PCL objects
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr normalCloud(new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloudHit(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloudMiss(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloudMissOutput(new pcl::PointCloud<pcl::PointXYZI>);

  // Create the filtering object
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointXYZINormal> ne;
  ne.setInputCloud(inputCloud);
  // ne.setInputCloud(inputCloud);

  // KdTree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod(tree);

  // Radius search param
  ne.setRadiusSearch(radiusSearch);

  // Run Filter
  ROS_INFO("Running normals filter...");
  ne.compute(*normalCloud);
  ROS_INFO("Normals filter success!");

  // Prepare outputCloud with xyz coordinates of normals
  pcl::PointXYZI query;
  for (int i=0; i<inputCloud->points.size(); i++)
  {
    normalCloud->points[i].x = inputCloud->points[i].x;
    normalCloud->points[i].y = inputCloud->points[i].y;
    normalCloud->points[i].z = inputCloud->points[i].z;
    normalCloud->points[i].intensity = std::acos(normalCloud->points[i].normal_z);
    // Slope angle filter
    query.x = inputCloud->points[i].x;
    query.y = inputCloud->points[i].y;
    query.z = inputCloud->points[i].z;
    query.intensity = normalCloud->points[i].normal_z;
    if ((query.z*query.z + query.y*query.y + query.x*query.x) < rFilter*rFilter) {
      if (normalCloud->points[i].normal_z >= z_nFilter) {
        filteredCloudHit->points.push_back(query);
      } else {
        filteredCloudMiss->points.push_back(query);
        filteredCloudMissOutput->points.push_back(query);
      }
    } else {
      if (normalCloud->points[i].normal_z < z_nFilter) {
        filteredCloudMissOutput->points.push_back(query);
      }
    }
  }

  // Convert back to PC2 ROS msg
  sensor_msgs::PointCloud2 newMsgNormalsCloud;
  normalCloud->is_dense = true;
  pcl::toROSMsg(*normalCloud, newMsgNormalsCloud);

  // Set Header params
  newMsgNormalsCloud.header.seq = 1;
  newMsgNormalsCloud.header.stamp = msg->header.stamp;
  newMsgNormalsCloud.header.frame_id = msg->header.frame_id;

  // Copy to publishable object property
  normalsMsg = newMsgNormalsCloud;

  // Convert back to PC2 ROS msg
  sensor_msgs::PointCloud2 newMsgHitCloud;
  inputCloud->is_dense = true;
  pcl::toROSMsg(*filteredCloudHit, newMsgHitCloud);

  // Set Header params
  newMsgHitCloud.header.seq = 1;
  newMsgHitCloud.header.stamp = msg->header.stamp;
  newMsgHitCloud.header.frame_id = msg->header.frame_id;

  // Copy to publishable object property
  filteredHitMsg = newMsgHitCloud;

  // Convert back to PC2 ROS msg
  sensor_msgs::PointCloud2 newMsgMissCloud;
  inputCloud->is_dense = true;
  pcl::toROSMsg(*filteredCloudMiss, newMsgMissCloud);

  // Set Header params
  newMsgMissCloud.header.seq = 1;
  newMsgMissCloud.header.stamp = msg->header.stamp;
  newMsgMissCloud.header.frame_id = msg->header.frame_id;

  // Copy to publishable object property
  filteredMissMsg = newMsgMissCloud;

  // Get transform from world to PointCloud
  tf::StampedTransform transform;
  try {
    tfListener.lookupTransform(fixedFrameId, msg->header.frame_id,  
                             msg->header.stamp, transform);
  }
  catch (tf::TransformException ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  // Clear a bubble around the robot

  // Transform cloud into fixed frame
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloudHit(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloudMiss(new pcl::PointCloud<pcl::PointXYZI>);
  // std::cout << msg->header.frame_id << std::endl;
  // std::cout << fixedFrameId << std::endl;
  // pcl_ros::transformPointCloud(msg->header.frame_id, msg->header.stamp, *filteredCloud, fixedFrameId, *transformedCloud, tfListener);
  pcl_ros::transformPointCloud(*filteredCloudHit, *transformedCloudHit, transform);
  pcl_ros::transformPointCloud(*filteredCloudMiss, *transformedCloudMiss, transform);

  // Add filtered cloud to map
  for (int i=0; i<transformedCloudHit->points.size(); i++) {
    query.x = transformedCloudHit->points[i].x;
    query.y = transformedCloudHit->points[i].y;
    query.z = transformedCloudHit->points[i].z;
    // mytree->updateNode((double)query.x, (double)query.y, (double)(query.z - mytree->getResolution()), (float)0.41);
    mytree->updateNode((double)query.x, (double)query.y, (double)query.z, (float)0.41);
    mytree->updateNode((double)query.x, (double)query.y, (double)(query.z + mytree->getResolution()), (float)-0.85);
    mytree->updateNode((double)query.x, (double)query.y, (double)(query.z + 2.0*mytree->getResolution()), (float)-0.85);
    // mytree->updateNode((double)query.x, (double)query.y, (double)(query.z + 3.0*mytree->getResolution()), (float)-0.85);
    // mytree->updateNode((double)query.x, (double)query.y, (double)(query.z + 4.0*mytree->getResolution()), (float)-0.85);
  }

  for (int i=0; i<transformedCloudMiss->points.size(); i++) {
    query.x = transformedCloudMiss->points[i].x;
    query.y = transformedCloudMiss->points[i].y;
    query.z = transformedCloudMiss->points[i].z;
    mytree->updateNode((double)query.x, (double)query.y, (double)query.z, (float)0.41);
    mytree->updateNode((double)query.x, (double)query.y, (double)(query.z + mytree->getResolution()), (float)0.41);
    mytree->updateNode((double)query.x, (double)query.y, (double)(query.z + 2.0*mytree->getResolution()), (float)0.41);
    // mytree->updateNode((double)query.x, (double)query.y, (double)(query.z + 3.0*mytree->getResolution()), (float)0.41);
    // mytree->updateNode((double)query.x, (double)query.y, (double)(query.z + 4.0*mytree->getResolution()), (float)0.41);
  }

  // Update octomapMsg for publishing
  octomap_msgs::binaryMapToMsg(*mytree, octomapMsg);
  octomapMsg.header.frame_id = fixedFrameId;
  octomapMsg.header.stamp = msg->header.stamp;
  pubMap = true;
  return;
}

int main(int argc, char **argv)
{
  // Initialize ROS node with name and object instantiation
  ros::init(argc, argv, "filter_PC2_normal");
  ros::NodeHandle n;

  double voxel_size;
  n.param("normal_filter/resolution", voxel_size, (double)0.2);
  PC2_normal_filter PC2_filter(voxel_size);
  ROS_INFO("Default odds hit = %0.2f", PC2_filter.mytree->getProbHitLog());
  ROS_INFO("Default odds miss = %0.2f", PC2_filter.mytree->getProbMissLog());
  // PC2_filter.mytree->setProbHit((double)0.4);
  // PC2_filter.mytree->setProbMiss((double)0.7);

  // Declare subscribers and publishers
  ros::Subscriber sub1 = n.subscribe("points", 1, &PC2_normal_filter::callback_cloud, &PC2_filter);
  ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("points_normals", 5);
  ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2>("points_filtered_hit", 5);
  ros::Publisher pub3 = n.advertise<sensor_msgs::PointCloud2>("points_filtered_miss", 5);
  ros::Publisher pub4 = n.advertise<octomap_msgs::Octomap>("traversability_map", 5);

  // Get normal estimation radius
  n.param("normal_filter/radiusSearch", PC2_filter.radiusSearch, (double)(0.2));

  // Slope threshold for filtering
  n.param("normal_filter/normalThreshold", PC2_filter.z_nFilter, (double)(0.85));
  ROS_INFO("Filtering normals with z-components less than %0.2f", PC2_filter.z_nFilter);

  // Radius for integrating normal cloud
  n.param("normal_filter/radiusThreshold", PC2_filter.rFilter, (double)(2.0));

  // Fixed frame id
  n.param<std::string>("normal_filter/fixedFrame", PC2_filter.fixedFrameId, "world");

  // Declare and read in the node update rate from the launch file parameters
  double updateRate;
  n.param("normal_filter/rate", updateRate, (double)10.0); // Hz
  ros::Rate r(updateRate);

  // Run the node until ROS quits
  while (ros::ok())
  {
    r.sleep(); // Node sleeps to update at a rate as close as possible to the updateRate parameter
    ros::spinOnce(); // All subscriber callbacks are called here.
    pub1.publish(PC2_filter.normalsMsg);
    pub2.publish(PC2_filter.filteredHitMsg);
    pub3.publish(PC2_filter.filteredMissMsg);
    if (PC2_filter.pubMap) pub4.publish(PC2_filter.octomapMsg);
  }

  return 0;
}