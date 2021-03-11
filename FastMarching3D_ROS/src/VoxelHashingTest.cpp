#include <math.h>
#include <voxelHashMap.h>
// Octomap libaries
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
// ROS Libraries
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
// Eigen
#include <Eigen/Core>

void index3_xyz(const int index, double point[3], double min[3], int size[3], double voxel_size)
{
  // x+y*sizx+z*sizx*sizy
  point[2] = min[2] + (index/(size[1]*size[0]))*voxel_size;
  point[1] = min[1] + ((index % (size[1]*size[0]))/size[0])*voxel_size;
  point[0] = min[0] + ((index % (size[1]*size[0])) % size[0])*voxel_size;
}

int xyz_index3(const double point[3], double min[3], int size[3], double voxel_size)
{
  int ind[3];
  for (int i=0; i<3; i++) ind[i] = round((point[i]-min[i])/voxel_size);
  return (ind[0] + ind[1]*size[0] + ind[2]*size[0]*size[1]);
}

class NodeManager
{
  public:
    NodeManager():
    cloud (new pcl::PointCloud<pcl::PointXYZI>)
    {
      map_octree = new octomap::OcTree(0.1);
    }
    sensor_msgs::PointCloud2 ground_msg;
    octomap::OcTree* map_octree;
    bool octomap_updated = false;
    bool pcl_updated = false;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    void CallbackOctomap(const octomap_msgs::Octomap::ConstPtr msg);
    void CallbackPCL(const sensor_msgs::PointCloud2::ConstPtr msg);
};

void NodeManager::CallbackOctomap(const octomap_msgs::Octomap::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  delete map_octree;
  map_octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
  octomap_updated = true;
  return;
}

void NodeManager::CallbackPCL(const sensor_msgs::PointCloud2::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  // Convert from ROS PC2 msg to PCL object
  pcl::fromROSMsg(*msg, *cloud);
  return;
}

// void NodeManager::GetGroundMsg()
// {
//   sensor_msgs::PointCloud2 msg;
//   pcl::toROSMsg(*ground_cloud, msg);
//   msg.header.seq = 1;
//   msg.header.stamp = ros::Time();
//   msg.header.frame_id = fixed_frame_id;
//   ground_msg = msg;
// }

int main(int argc, char **argv)
{
  // Node declaration
  ros::init(argc, argv, "VoxelHashingTest");
  ros::NodeHandle n;

  NodeManager node_manager;

  // Subscribers and Publishers
  ros::Subscriber sub_octomap = n.subscribe("octomap_binary", 1, &NodeManager::CallbackOctomap, &node_manager);
  ros::Subscriber sub_pcl = n.subscribe("edt", 1, &NodeManager::CallbackPCL, &node_manager);
  ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("ground", 5);

  ROS_INFO("Initialized subscriber and publishers.");

  // Params
  double voxel_size;
  n.param("VoxelHashingTest/voxel_size", voxel_size, 5.0);

  float update_rate;
  n.param("VoxelHashingTest/update_rate", update_rate, (float)0.2);
  
  ros::Rate r(update_rate); // 5 Hz
  ROS_INFO("Finished reading params.");
  // Main Loop
  clock_t t_start;
  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();

    pcl::PointXYZI min_p, max_p;
    pcl::getMinMax3D(*node_manager.cloud, min_p, max_p);
    
    // Copy pointcloud to flat array
    int size[3];
    size[0] = std::roundf((max_p.x - min_p.x)/voxel_size) + 3;
    size[1] = std::roundf((max_p.y - min_p.y)/voxel_size) + 3;
    size[2] = std::roundf((max_p.z - min_p.z)/voxel_size) + 3;
    double min[3];
    min[0] = min_p.x - voxel_size;
    min[1] = min_p.y - voxel_size;
    min[2] = min_p.z - voxel_size;
    float map_flat[size[0]*size[1]*size[2]];
    ROS_INFO("Beginning pointcloud copying into flat matrix map...");
    t_start = clock();
    for (int i=0; i<node_manager.cloud->points.size(); i++) {
      double p[3] = {node_manager.cloud->points[i].x, node_manager.cloud->points[i].y, node_manager.cloud->points[i].z};
      int id = xyz_index3(p, min, size, voxel_size);
      map_flat[id] = node_manager.cloud->points[i].intensity;
    }
    ROS_INFO("Pointcloud added to flat matrix map in: %.5fs", (double)(clock() - t_start)/CLOCKS_PER_SEC);

    // Copy pointcloud to voxel hash object
    int local_map_size[3] = {10,10,5};
    int max_size[3] = {1000,1000,1000};
    VoxelHashMap hashmap(local_map_size, max_size);
    hashmap.min[0] = min_p.x - voxel_size;
    hashmap.min[1] = min_p.y - voxel_size;
    hashmap.min[2] = min_p.z - voxel_size;
    hashmap.voxel_size = voxel_size;
    ROS_INFO("Beginning pointcloud copying into voxel hash map...");
    t_start = clock();
    for (int i=0; i<node_manager.cloud->points.size(); i++) {
      hashmap.Add(node_manager.cloud->points[i].x, node_manager.cloud->points[i].y, node_manager.cloud->points[i].z, node_manager.cloud->points[i].intensity);
    }
    ROS_INFO("Pointcloud added to voxel hash map in: %.5fs", (double)(clock() - t_start)/CLOCKS_PER_SEC);

    // Time the time to query the first 2000 points
    int cloud_length = node_manager.cloud->points.size();
    if (cloud_length > 0) {
      ROS_INFO("Reading %d points from flat matrix map...", cloud_length);
      t_start = clock();
      for (int i=0; i<cloud_length; i++) {
        double p[3] = {node_manager.cloud->points[(i%cloud_length)].x, node_manager.cloud->points[(i%cloud_length)].y, node_manager.cloud->points[(i%cloud_length)].z};
        int id = xyz_index3(p, min, size, voxel_size);
        float value = map_flat[id];
      }
      ROS_INFO("%d points read from flat matrix map in: %.5fs", cloud_length, (double)(clock() - t_start)/CLOCKS_PER_SEC);

      ROS_INFO("Reading %d points from voxel hash map...", cloud_length);
      t_start = clock();
      for (int i=0; i<cloud_length; i++) {
        float value = hashmap(node_manager.cloud->points[(i%cloud_length)].x, node_manager.cloud->points[(i%cloud_length)].y, node_manager.cloud->points[(i%cloud_length)].z);
      }
      ROS_INFO("%d points read from hash map in: %.5fs", cloud_length, (double)(clock() - t_start)/CLOCKS_PER_SEC);
    }

    // Memory comparison
    ROS_INFO("Flat matrix is %d bytes.", sizeof(map_flat));

    size_t hashmap_size;
    for (int i=0; i<hashmap.data.bucket_count(); i++) {
      size_t bucket_size = hashmap.data.bucket_size(i);
      if (bucket_size == 0) {
        hashmap_size++;
      }
      else {
        hashmap_size += bucket_size;
      }
    }

    ROS_INFO("Voxel hash map is %d bytes.", (int)hashmap_size);


  }
}