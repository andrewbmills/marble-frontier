// g++ msfm3d_node.cpp -g -o msfm3d_node.o -I /opt/ros/melodic/include -I /usr/include/c++/7.3.0 -I /home/andrew/catkin_ws/devel/include -I /home/andrew/catkin_ws/src/octomap_msgs/include -I /usr/include/pcl-1.8 -I /usr/include/eigen3 -L /usr/lib/x86_64-linux-gnu -L /home/andrew/catkin_ws/devel/lib -L /opt/ros/melodic/lib -Wl,-rpath,opt/ros/melodic/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization -loctomap -lboost_system -lpcl_common -lpcl_io -lpcl_filters -lpcl_features -lpcl_kdtree -lpcl_segmentation

#include <math.h>
#include <algorithm>
#include <random>
// ROS libraries
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
// #include <marble_common/ArtifactArray.msg>
// Octomap libaries
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
// pcl libraries
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
//pcl ROS
#include <pcl_conversions/pcl_conversions.h>
// Custom libraries
#include "msfm3d.c"

// Converts 4 integer byte values (0 to 255) to a float
float byte2Float(const int a[4])
{
  // Declare union holder variable for byte2float conversion
  union {
    unsigned char bytes[4];
    float f;
  } byteFloat;
  // Store values in a in union variable.
  for (int i = 0; i < 4; i++) {
    byteFloat.bytes[i] = a[i];
  }
  return byteFloat.f;
}

float dist2(const float a[2], const float b[2]){
  float sum = 0.0;
  for (int i=0; i < 2; i++) sum += (b[i] - a[i])*(b[i] - a[i]);
  return std::sqrt(sum);
}

float dist3(const float a[3], const float b[3]){
  float sum = 0.0;
  for (int i=0; i < 3; i++) sum += (b[i] - a[i])*(b[i] - a[i]);
  return std::sqrt(sum);
}

int sign(float a){
  if (a>0.0) return 1;
  if (a<0.0) return -1;
  return 0;
}

float angle_diff(float a, float b)
{
    // Computes a-b, preserving the correct sign (counter-clockwise positive angles)
    // All angles are in degrees
    a = std::fmod(360000.0 + a, 360.0);
    b = std::fmod(360000.0 + b, 360.0);
    float d = a - b;
    d = std::fmod(d + 180.0, 360.0) - 180.0;
    return d;
}

// Some orientation and pose structures
struct Quaternion {
  float w, x, y, z;
};

Quaternion euler2Quaternion(float yaw, float pitch, float roll) // yaw (Z), pitch (Y), roll (X)
{
  // From Wikipedia
  // Abbreviations for the various angular functions
  float cy = std::cos(yaw * 0.5);
  float sy = std::sin(yaw * 0.5);
  float cp = std::cos(pitch * 0.5);
  float sp = std::sin(pitch * 0.5);
  float cr = std::cos(roll * 0.5);
  float sr = std::sin(roll * 0.5);

  Quaternion q;
  q.w = cy * cp * cr + sy * sp * sr;
  q.x = cy * cp * sr - sy * sp * cr;
  q.y = sy * cp * sr + cy * sp * cr;
  q.z = sy * cp * cr - cy * sp * sr;
  return q;
}

struct Pose {
  pcl::PointXYZ position;
  Eigen::Matrix3f R;
  Quaternion q;
};
struct SensorFoV {
  float verticalFoV; // in degrees
  float horizontalFoV; // in degrees
  float rMin; // in meters
  float rMax; // in meters
};
struct View {
  Pose pose;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  int index = -1;
};

// Frontier class declaration
// class Frontier
// {
//   public:
//     std::vector<bool> data;
//     float x_min;
//     float y_min;
//     float z_min;
//     float x_max;
//     float y_max;
//     float z_max;
//     float x_size;
//     float y_size;
//     float z_size;
//     float voxel_size;

//     int xyz_index3(const float xyz[3]);
//     void index3_xyz(const int index, float xyz[3]);
//     void expand(const float limits[6]); // 
// }

// void Frontier::index3_xyz(const int index, float xyz[3])
// {
//   // x+y*sizx+z*sizx*sizy
//   xyz[2] = z_min + (index/(y_size*x_size))*voxel_size;
//   xyz[1] = y_min + ((index % (y_size*x_size))/x_size)*voxel_size;
//   xyz[0] = x_min + ((index % (y_size*x_size)) % x_size)*voxel_size;
// }

// int Frontier::xyz_index3(const float xyz[3])
// {
//   int ind[3];
//   ind[0] = roundf((xyz[0]-x_min)/voxel_size);
//   ind[1] = roundf((xyz[1]-y_min)/voxel_size);
//   ind[2] = roundf((xyz[2]-z_min)/voxel_size);
//   return mindex3(ind[0], ind[1], ind[2], x_size, y_size);
// }

// void Frontier::expand(const float limits[6])
// {
//   // Calculate the new required size of the vector
//   // Copy the old data to a holding vector of the old size
//   // Resize the data property to the new size and clear the data
//   // Copy the previous data into the data property
// }

//Msfm3d class declaration
class Msfm3d
{
  public:
    // Constructor
    Msfm3d(float map_resolution):
    frontierCloud(new pcl::PointCloud<pcl::PointXYZ>)
    {
      reach = NULL;
      esdf.data = NULL;
      esdf.seen = NULL;
      frontier = NULL;
      entrance = NULL;
      camera.verticalFoV = 30.0;
      camera.horizontalFoV = 60.0;
      camera.rMin = 0.5;
      camera.rMax = 1.5;
      voxel_size = map_resolution;
      mytree = new octomap::OcTree(voxel_size);
      robot2camera.position.x = 0.0;
      robot2camera.position.y = 0.0;
      robot2camera.position.z = 0.0;
      robot2camera.q.x = 0.0;
      robot2camera.q.y = 0.0;
      robot2camera.q.z = 0.0;
      robot2camera.q.w = 1.0;
      robot2camera.R.setIdentity();
    }

    // Structure definitions
    struct ESDF {
      double * data; // esdf matrix pointer
      bool * seen; // seen matrix pointer
      int size[3]; // number of elements in each dimension
      float max[4]; // max and min values in each dimension
      float min[4];
    };
    struct Boundary {
      bool set = 0;
      float xmin = 0.0, xmax = 0.0, ymin = 0.0, ymax = 0.0, zmin = 0.0, zmax = 0.0;
    };

    // Vehicle parameters
    bool ground = false; // whether the vehicle is a ground vehicle
    bool fixGoalHeightAGL = false; // whether or not the vehicle has a fixed goal point height above ground level
    float goalHeightAGL = 0.64; // meters
    float position[3] = {69.0, 420.0, 1337.0}; // robot position
    float euler[3]; // robot orientation in euler angles
    float R[9]; // Rotation matrix
    Boundary vehicleVolume; // xyz boundary of the vehicle bounding box in rectilinear coordinates for collision detection/avoidance

    // Vehicle linear and angular velocities
    float speed = 1.0; // m/s
    float turnRate = 5.0; // deg/s

    // Environment/Sensor parameters
    std::string frame = "world";
    bool esdf_or_octomap = 0; // Boolean to use an esdf PointCloud2 or an Octomap as input
    bool receivedPosition = 0;
    bool receivedMap = 0;
    bool updatedMap = 0;
    float voxel_size;
    float bubble_radius = 1.0; // map voxel size, and bubble radius
    float origin[3]; // location in xyz coordinates where the robot entered the environment
    float entranceRadius; // radius around the origin where frontiers can't exist

    double * reach; // reachability grid (output from reach())
    sensor_msgs::PointCloud2 PC2msg;
    nav_msgs::Path pathmsg;
    // visualization_msgs::MarkerArray frontiermsg;
    octomap::OcTree* mytree; // OcTree object for holding Octomap

    // Frontier and frontier filter parameters
    // std::vector<bool> frontier;
    // float 
    bool * frontier;
    bool * entrance;
    int frontier_size = 0;
    int dzFrontierVoxelWidth = 0;
      // Clustering/Filtering
      pcl::PointCloud<pcl::PointXYZ>::Ptr frontierCloud; // Frontier PCL
      std::vector<pcl::PointIndices> frontierClusterIndices;
      float cluster_radius;
      float min_cluster_size;
      // ROS Interfacing
      sensor_msgs::PointCloud2 frontiermsg;
      // Frontier Grouping
      std::vector<pcl::PointIndices> greedyGroups;
      pcl::PointCloud<pcl::PointXYZ> greedyCenters;
      std::vector<int> greedyClusterNumber;

    // Vector of possible goal poses
    std::vector<View> goalViews;

    // Artifact detected flag
    bool artifactDetected = false;

    // Sensor parameters
    SensorFoV camera;
    Pose robot2camera;

    Quaternion q; // robot orientation in quaternions
    ESDF esdf; // ESDF struct object
    Boundary bounds; // xyz boundary of possible goal locations

    void callback(sensor_msgs::PointCloud2 msg); // Subscriber callback function for PC2 msg (ESDF)
    void callback_Octomap(const octomap_msgs::Octomap::ConstPtr msg); // Subscriber callback function for Octomap msg
    void callback_Octomap_freePCL(const sensor_msgs::PointCloud2 msg);
    void callback_Octomap_occupiedPCL(const sensor_msgs::PointCloud2 msg);
    void callback_position(const nav_msgs::Odometry msg); // Subscriber callback for robot position
    void callback_artifactDetected(const std_msgs::Bool msg);
    // void callback_artifactDetected(const marble_common::ArtifactArray msg);
    void parsePointCloud(); // Function to parse pointCloud2 into an esdf format that msfm3d can use
    int xyz_index3(const float point[3]);
    void index3_xyz(const int index, float point[3]);
    void getEuler(); // Updates euler array given the current quaternion values
    bool updatePath(const float goal[3]); // Updates the path vector from the goal frontier point to the robot location
    void updateFrontierMsg(); // Updates the frontiermsg MarkerArray with the frontier matrix for publishing
    bool clusterFrontier(const bool print2File); // Clusters the frontier pointCloud with euclidean distance within a radius
    bool inBoundary(const float point[3]); // Checks if a point is inside the planner boundaries
    // bool collisionCheck(const float point[3]); // Checks if the robot being at the current point (given vehicleVolume) intersects with the obstacle environment (esdf)
    void greedyGrouping(const float r, const bool print2File);
    bool raycast(const pcl::PointXYZ start, const pcl::PointXYZ end);
    Pose samplePose(const pcl::PointXYZ centroid, const SensorFoV camera, const int sampleLimit);
    void updateGoalPoses();
    void inflateObstacles(const float radius, sensor_msgs::PointCloud2& inflatedOccupiedMsg);
    float heightAGL(const float point[3]);
};

// void Msfm3d::callback_artifactDetected(const marble_common::ArtifactArray msg)
void Msfm3d::callback_artifactDetected(const std_msgs::Bool msg)
{
  // If any of the artifacts have yet to be reported, set artifactDetected to true and return the vehicle to the origin.
  // for (int i=0; i<(int)msg.artifacts.size(); i++) {
  //   if (!(msg.artifacts[i].has_been_reported)) {
  //     artifactDetected = true;
  //     return;
  //   }
  // }

  // artifactDetected = false;
  // return;

  // Switch to commented out text once the marble common library shows up
  artifactDetected = msg.data;
  return;
}

float Msfm3d::heightAGL(const float point[3])
{
  // heightAGL finds the distance that the query point is above/below the "ground" of the obstacle map.
  // Function returns NaN if the point is unseen or outside of the current esdf.
  // Input -
  //    - point: a 3 element float of the (x,y,z) position of the point that you want the AGL height of.
  // Output -
  //    - dz: The query point's height above the obstacle map.

  int query_idx;
  int npixels = esdf.size[0]*esdf.size[1]*esdf.size[2];
  float query[3];
  float height;

  // If the query point is out of bounds, return NaN
  for (int i = 0; i < 3; i++) {
    if (point[i] < esdf.min[i] || point[i] > esdf.max[i]) {
      return std::sqrt(-1.0);
    }
  }

  // If the query point is unseen, return NaN
  query_idx = xyz_index3(point);
  if (query_idx < 0 || query_idx >= npixels) {
    return std::sqrt(-1.0);
  } else if (!esdf.seen[query_idx]) {
    return std::sqrt(-1.0);
  }

  // Copy point entries into query
  for (int i = 0; i < 3; i++) {
    query[i] = point[i];
  }

  // Find the height above the ground by return the dz vertical distance downward (minus half a voxel_size) until an unseen, occupied, or out of bounds voxel is queried.
  for (float dz = 0.0; dz < 100.0*voxel_size; dz = dz + 0.01) {
    query[2] = point[2] - dz;
    query_idx = xyz_index3(query);
    if (query_idx < 0 || query_idx >= npixels) { // If the query location is at the z limit of the map, then it is the bottom of the map.
      return dz;
    } else {
      if (!esdf.seen[query_idx] || (esdf.data[query_idx] <= 0.01)) { // If the query location is unseen or occupied, then it is the bottom of the local map
      // if (!esdf.seen[query_idx]) {
        return dz;
      }
    }
  }

  return 100.0*voxel_size;
}

bool Msfm3d::inBoundary(const float point[3])
{
  if (bounds.set){
    if ((bounds.xmin <= point[0]) && (bounds.xmax >= point[0]) && (bounds.ymin <= point[1]) && (bounds.ymax >= point[1]) && (bounds.zmin <= point[2]) && (bounds.zmax >= point[2])) {
      return 1;
    } else {
      return 0;
    }
  } else {
    // No boundary exists so every point is inside.
    return 1;
  }
}

bool Msfm3d::clusterFrontier(const bool print2File)
{
  // clusterFrontier takes as input the frontierCloud and the voxel_size and extracts contiguous clusters of minimum size round(12.0/voxel_size) (60 for voxel_size=0.2) voxels.
  // The indices of the cluster members are stored in frontierClusterIndices and the unclustered voxels are filtered from frontierCloud.
  // Input:
  //    - Msfm3d.frontierCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr)
  //    - Msfm3d.voxel_size (float)
  // Output:
  //    - Msfm3d.frontierClusterIndices (std::vector<pcl::PointIndices>)
  //    - Msfm3d.frontierCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr)
  //
  // Lines commented out include debugging ROS_INFO() text and .pcd file storage of the frontier cluster PointClouds.

  ROS_INFO("Frontier cloud before clustering has: %d data points.", (int)frontierCloud->points.size());

  // Create cloud pointer to store the removed points
  pcl::PointCloud<pcl::PointXYZ>::Ptr removed_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  kdtree->setInputCloud(frontierCloud);

  // Clear previous frontierClusterIndices
  frontierClusterIndices.clear();

  // Initialize euclidean cluster extraction object
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_radius); // Clusters must be made of contiguous sections of frontier (within sqrt(2)*voxel_size of each other)
  // ec.setClusterTolerance(2.9*voxel_size);
  ec.setMinClusterSize(roundf(min_cluster_size)); // Cluster must be at least 15 voxels in size
  // ec.setMaxClusterSize (30);
  ec.setSearchMethod(kdtree);
  ec.setInputCloud(frontierCloud);
  ec.extract(frontierClusterIndices);

  // Iterate through clusters and write to file
  int j = 0;
  pcl::PCDWriter writer;
  for (std::vector<pcl::PointIndices>::const_iterator it = frontierClusterIndices.begin(); it != frontierClusterIndices.end(); ++it){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit=it->indices.begin(); pit!=it->indices.end(); ++pit){
      if (print2File) cloud_cluster->points.push_back(frontierCloud->points[*pit]);
      inliers->indices.push_back(*pit); // Indices to keep in frontierCloud
    }
    if (print2File) {
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
      std::stringstream ss;
      ss << "pcl_clusters/cloud_cluster_" << j << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
      j++;
    }
  }

  // Loop through the remaining points in frontierCloud and remove them from the frontier bool array
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(frontierCloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*removed_points);
  float query[3];
  int idx;
  for (pcl::PointCloud<pcl::PointXYZ>::iterator it=removed_points->begin(); it!=removed_points->end(); ++it){
    query[0] = it->x;
    query[1] = it->y;
    query[2] = it->z;
    idx = xyz_index3(query);
    frontier[idx] = 0;
    // ROS_INFO("Removed frontier voxel at (%f, %f, %f).", query[0], query[1], query[2]);
  }

  // Filter frontierCloud to keep only the inliers
  extract.setNegative(false);
  extract.filter(*frontierCloud);
  ROS_INFO("Frontier cloud after clustering has %d points.", (int)frontierCloud->points.size());

  if ((int)frontierCloud->points.size() < 1) {
    return 0;
  } else {
    // Get new indices of Frontier Clusters after filtering (extract filter does not preserve indices);
    frontierClusterIndices.clear();
    kdtree->setInputCloud(frontierCloud);
    ec.setSearchMethod(kdtree);
    ec.setInputCloud(frontierCloud);
    ec.extract(frontierClusterIndices);
    return 1;
  }
}

bool filterCloudRadius(const float radius, const pcl::PointXYZ point, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers)
{
  // filterRadius returns the indices (inside inliers->indices) in cloud that are within euclidean distance r of point (x,y,z).

  // Find all indices within r of point (x,y,z)
  float distance_squared;
  float delta_x;
  float delta_y;
  float delta_z;
  float radius_squared = radius*radius;
  for (int i = 0; i<(int)cloud->points.size(); i++) { // This line is vague, consider modifying with a more explicit range of values
    delta_x = point.x - cloud->points[i].x;
    delta_y = point.y - cloud->points[i].y;
    delta_z = point.z - cloud->points[i].z;
    distance_squared = (delta_x*delta_x) + (delta_y*delta_y) + (delta_z*delta_z);
    if (distance_squared < radius_squared) inliers->indices.push_back(i);  // Add indices that are within the radius to inliers
  }

  // Return false if there are no points in cloud within r of point (x,y,z)
  if (inliers->indices.size() > 0)
    return 1;
  else
    return 0;
}

void Msfm3d::greedyGrouping(const float radius, const bool print2File)
{
  // greedGrouping generates a vector of (pcl::PointIndices) where each entry in the vector is a greedily sampled group of points within radius of a randomly sampled frontier member.
  // Algorithm:
  //    0) clusterCount = 0; groupCount = 0;
  //    1) Add all indices in the current cluster (frontierClusterIndices[clusterCount]) to ungrouped
  //    2) While ungroupedIndices.size > 0
  //      a) Sample an index from ungroupedIndices and get its point location (x, y, z)
  //      b) Filter the ungrouped to ranges (x-r,y-r,z-r):(x+r, y+r, z+r) where r is radius
  //      c) Remove all indices that are within r of (x,y,z) from ungroupedIndices and add them to greedyCluster[groupCount]
  //      d) groupCount++;
  //    3) clusterCount++, GoTo 1)
  // Inputs:
  //    - Msfm3d.frontierClusterIndices (std::vector<pcl::PointIndices>)
  //    - Msfm3d.frontierCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr)
  //    - radius (float)
  // Outputs:
  //    - Msfm3d.greedyGroups (std::vector<pcl::PointIndices>)
  //    - Msfm3d.greedyCenters (pcl::PointCloud<pcl::PointXYZ>)
  //
  // Each greedy group is used as a source for the pose sampling function.

  // Intialize random seed:
  std::random_device rd;
  // std::mt19937 mt(19937);
  std::mt19937 mt(rd()); // Really random (much better than using rand())

  // Initialize counts of the current group and cluster in the algorithm
  int groupCount = 0;
  int clusterCount = 0;

  // Clear the greedy cluster arrays in the msfm3d object
  greedyGroups.clear();
  greedyCenters.clear();

  ROS_INFO("Beginning greedy grouping of %d clusters with sizes:", (int)frontierClusterIndices.size());
  for (int i = 0; i < frontierClusterIndices.size(); i++) {
  	std::cout << frontierClusterIndices[i].indices.size() << " ";
  }
  std::cout << std::endl;

  // Loop through all of the frontier clusters
  for (std::vector<pcl::PointIndices>::const_iterator it = frontierClusterIndices.begin(); it != frontierClusterIndices.end(); ++it) {

    // Initialize a PointIndices pointer of all the ungrouped points in the current cluster
    pcl::PointIndices::Ptr ungrouped(new pcl::PointIndices);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
    	ungrouped->indices.push_back(*pit);
    }

    // Go until every member of ungrouped is in a greedy group
    while (ungrouped->indices.size() > 0) {
      // Reset/Update ungrouped PointCloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr ungrouped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(frontierCloud);
      extract.setIndices(ungrouped);
      extract.filter(*ungrouped_cloud);
      // ROS_INFO("Initializing ungrouped_cloud for group %d containing %d points.", groupCount, (int)ungrouped_cloud->points.size());

      // Sample a random index from ungrouped
      std::uniform_int_distribution<int> dist(0, (int)ungrouped->indices.size()-1);
      int sample_id = ungrouped->indices[dist(mt)];
      pcl::PointXYZ sample_point = frontierCloud->points[sample_id];
      // ROS_INFO("Sampled index %d which is located at (%f, %f, %f).", sample_id, sample_point.x, sample_point.y, sample_point.z);

      // Find all the voxels in ungrouped_cloud that are within r of sample_point
      pcl::PointIndices::Ptr group(new pcl::PointIndices);
      filterCloudRadius(radius, sample_point, ungrouped_cloud, group);
      // ROS_INFO("Found %d points within %f of the sample point.", (int)group->indices.size(), radius);

      // Resize greedyGroups to make room for a new group
      greedyGroups.resize(greedyGroups.size() + 1);

      // Add grouped indices to greedyGroups
      for (std::vector<int>::const_iterator git = group->indices.begin(); git != group->indices.end(); ++git) {
        greedyGroups[groupCount].indices.push_back(ungrouped->indices[*git]); // Add indices to group
        ungrouped->indices[*git] = -1;
      }

      // Removed grouped indices from ungrouped  (This command removes all members of ungrouped->indices that have a value of -1)
      ungrouped->indices.erase(std::remove(ungrouped->indices.begin(), ungrouped->indices.end(), -1), ungrouped->indices.end());

      // Add sample_point to greedyCenters
      greedyCenters.points.push_back(sample_point);

      // Add the current cluster number to greedyClusterNumber
      greedyClusterNumber.push_back(clusterCount);

      // Add group clouds to file for debugging
      int j = 0;
      pcl::PCDWriter writer;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator fit = greedyGroups[groupCount].indices.begin(); fit != greedyGroups[groupCount].indices.end(); ++fit) {
        if (print2File) cloud_cluster->points.push_back(frontierCloud->points[*fit]);
      }
      if (print2File) {
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // std::cout << "PointCloud representing the group: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "pcl_clusters/cloud_cluster_" << clusterCount << "_group_" << groupCount << ".pcd";
        // std::cout << "Writing to file " << ss.str() << std::endl;
        writer.write<pcl::PointXYZ> (ss.str(), *cloud_cluster, false); //*
      }
      groupCount++;
    }
    clusterCount++;
  }
  ROS_INFO("%d groups generated from the frontier clusters.", groupCount);
}

bool Msfm3d::raycast(const pcl::PointXYZ start, const pcl::PointXYZ end) {
  float dx = end.x - start.x;
  float dy = end.y - start.y;
  float dz = end.z - start.z;
  float radius = std::sqrt(dx*dx + dy*dy + dz*dz);

  if (esdf_or_octomap) {
    // Perform a raycast in Octomap
    octomap::point3d origin(start.x, start.y, start.z);

    octomap::point3d direction(dx, dy, dz);

    octomap::point3d stop;

    // ROS_INFO("Calling Octomap castRay function...");
    if (mytree->castRay(origin, direction, stop, false, (double)radius)) {
      return false;
    }

    // ROS_INFO("Checking if the stop cell is the same as the end cell...");
    // Calculate the distance between the end and centroid
    float diff_x = (end.x - stop.x());
    float diff_y = (end.y - stop.y());
    float diff_z = (end.z - stop.z());
    float distance = std::sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);
    if (distance > voxel_size) {
      // castRay hit an unseen voxel before hitting the centroid, sample another point
      return false;
    } else {
      return true;
    }
  } else {
    // I don't have this built out yet
    // ROS_INFO("Occlusion detection is not defined for ESDF at the moment.  Use Octomap for pose sampling.  Returning True for all raycasts.");
    return true;
  }
}

Pose Msfm3d::samplePose(const pcl::PointXYZ centroid, const SensorFoV camera, const int sampleLimit)
{
  // samplePose takes as input a greedy frontier group centroid and the Octomap/ESDF and outputs a pose that can view the centroid with no occlusion.
  //
  // Inputs-
  //    - centroid: (x,y,z) location used to generate robotPose
  //    - sampleLimit: The max number of random samples before the function quits
  //    - tree: The Octomap ocTree pointer used for raycasting
  //      --OR--
  //    - esdf: Raycasting is performed using a bresenham function
  // Outputs-
  //    - Pose: robot pose from which the centroid is viewable in the occupancy/esdf map
  //
  // If no sample is found, the Pose is populated with NaN's

  // Initialize output pose
  Pose robotPose; // Pose is returned in ENU (East-North-Up) with a 3-2-1 rotation order
  robotPose.position.x = std::sqrt(-1.0);
  robotPose.position.y = std::sqrt(-1.0);
  robotPose.position.z = std::sqrt(-1.0);
  robotPose.R.setZero();
  // Intialize random seed:
  std::random_device rd;
  std::mt19937 mt(rd());
  // std::mt19937 mt(19937);

  // Uniform continuous distributions over spherical coordinates
  std::uniform_real_distribution<float> radius_dist(camera.rMin, camera.rMax);
  std::uniform_real_distribution<float> azimuth_dist(0, 2*M_PI);
  std::uniform_real_distribution<float> elevation_dist((M_PI - (M_PI/180.0)*camera.verticalFoV)/2.0, (M_PI + (M_PI/180.0)*camera.verticalFoV)/2.0);

  // ROS_INFO("Initialized uniform distributions, sampling poses near the centroid.");
  for (int i = 0; i < sampleLimit; i++) {
    // Sample in spherical coordinates
    float radius_sample = radius_dist(mt);
    float azimuth_sample = azimuth_dist(mt);
    float elevation_sample = elevation_dist(mt);

    // Generate sample
    pcl::PointXYZ sample;
    sample.x = centroid.x + radius_sample*std::sin(elevation_sample)*std::cos(azimuth_sample);
    sample.y = centroid.y + radius_sample*std::sin(elevation_sample)*std::sin(azimuth_sample);
    sample.z = centroid.z + radius_sample*std::cos(elevation_sample);

    // ROS_INFO("Sample #%d is at [%f, %f, %f] from centroid [%f, %f, %f].", i, sample.x, sample.y, sample.z, centroid.x, centroid.y, centroid.z);

    // Find the index of the sample
    float sample_query[3] = {sample.x, sample.y, sample.z};
    int sample_idx = xyz_index3(sample_query);

    // ROS_INFO("Sampled point is at index %d", sample_idx);
    // See if the sample is within the current map range
    if ((sample_idx < 0) || (sample_idx >= esdf.size[0]*esdf.size[1]*esdf.size[2])) {
      continue;
    }

    // ROS_INFO("Checking if sampled point is reachable...");
    // See if sample is at an unreachable point (occupied or unseen)
    if (esdf.data[sample_idx] <= 0.0) {
      continue;
    }

    // If the vehicle is a ground vehicle, move the sampled point vertically until it's wheel_bottom_dist off the ground
    if (ground || fixGoalHeightAGL) {
      // ROS_INFO("Sample point is at height %0.2f.", sample.z);
      float height = heightAGL(sample_query);
      if (!std::isnan(height)) {
        sample.z = sample.z - (height - goalHeightAGL);
      } else {
        continue;
      }

      // Check to make sure the centroid is within r_min to r_max
      float r_new = std::sqrt((sample.x - centroid.x)*(sample.x - centroid.x) + (sample.y - centroid.y)*(sample.y - centroid.y) + (sample.z - centroid.z)*(sample.z - centroid.z));
      // ROS_INFO("Sample point is now at height %0.2f.  This point is %0.2f meters from the sample source.", sample.z, r_new);
      if (r_new < camera.rMin || r_new > camera.rMax) {
        continue;
      }

      // Check to make sure the centroid is still within the verticalFoV
      if ((std::abs(sample.z - centroid.z)/r_new) > std::sin((M_PI/180.0)*camera.verticalFoV/2.0)) {
        continue;
      }
      // ROS_INFO("Sample point is still within the verticalFoV.");
    }

    // ROS_INFO("Checking if the point is occluded from viewing the centroid...");
    // Check for occlusion
    if (!raycast(sample, centroid)) {
      continue;
    }

    // ROS_INFO("Returning sample pose...");
    // This pose is valid! Save it and exit the loop.
    float sample_yaw = M_PI + azimuth_sample;
    robotPose.position = sample;
    robotPose.R(0,0) = std::cos(sample_yaw);
    robotPose.R(0,1) = -std::sin(sample_yaw);
    robotPose.R(1,0) = std::sin(sample_yaw);
    robotPose.R(1,1) = std::cos(sample_yaw);
    robotPose.R(2,2) = 1.0;
    // Calculate Quaternions
    robotPose.q = euler2Quaternion(sample_yaw, 0.0, 0.0);
    break;
  }

  return robotPose;
}

void Msfm3d::updateGoalPoses()
{
  // Conversion matrix from ENU to EUS
  // Eigen::Matrix4f ENU_to_PCL;
  // ENU_to_PCL << 1, 0, 0, 0,
  //               0, 0, 1, 0,
  //               0, -1, 0, 0,
  //               0, 0, 0, 1;

  // Clear previous goalViews vector
  goalViews.clear();

  // Track the current cluster for pruning unviewable clusters;
  // std::vector<int> clusterViewCounts;
  // clusterViewCounts.resize((int)frontierClusterIndices.size());
  // std::fill(clusterViewCounts.begin(), clusterViewCounts.end(), 0); // Set all counts to zero to start
  // pcl::PointIndices::Ptr removeablePoints(new pcl::PointIndices());

  ROS_INFO("Finding feasible poses from which to view frontier groups.");
  // Loop through the frontier group centroids
  for (int it=0; it<(int)greedyCenters.points.size(); ++it) {
    // Sample an admissable pose that sees the centroid
    // ROS_INFO("Sampling an admissable pose that sees the group centroid.");
    Pose goalPose = samplePose(greedyCenters.points[it], camera, 1500);

    if (std::isnan(goalPose.position.x)) {
      // ROS_INFO("Pose is invalid, next loop.");
      continue;
    }

    // At least one group in this cluster is viewable, so add a view count to the vector
    // clusterViewCounts[greedyClusterNumber[it]] = clusterViewCounts[greedyClusterNumber[it]] + 1;

    // Convert the goalPose into a 4x4 view matrix in EUS (East Up South) or (Forward-Up-Right)
    // ROS_INFO("Converting to PCL coordinate system...");
    Eigen::Matrix4f camera_pose;
    camera_pose.setZero();
    camera_pose.topLeftCorner(3,3) = goalPose.R*robot2camera.R; // rotate into camera frame (Husky's cam is angled downward)
    camera_pose(0,3) = goalPose.position.x;
    camera_pose(1,3) = goalPose.position.y;
    camera_pose(2,3) = goalPose.position.z;
    camera_pose(3,3) = 1.0;
    camera_pose = camera_pose; // Rotate camera_pose into weird PCL EUS coordinates

    // Cull the view frustum points with the pcl function
    // ROS_INFO("Frustum Culling...");
    pcl::FrustumCulling<pcl::PointXYZ> fc;
    fc.setInputCloud(frontierCloud);
    fc.setVerticalFOV(camera.verticalFoV);
    fc.setHorizontalFOV(camera.horizontalFoV);
    fc.setNearPlaneDistance(camera.rMin);
    fc.setFarPlaneDistance(camera.rMax);
    fc.setCameraPose(camera_pose);
    pcl::PointCloud<pcl::PointXYZ>::Ptr viewed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    fc.filter(*viewed_cloud);

    // ROS_INFO("Sampled view has %d frontier voxels within its frustum.", (int)viewed_cloud->points.size());

    // Perform raycasting and check which culled cloud points are visible from goalPose.position
    // ROS_INFO("Raycasting...");
    pcl::PointIndices::Ptr seen(new pcl::PointIndices());
    for (int i = 0; i < viewed_cloud->points.size(); i++) {
      if (raycast(goalPose.position, viewed_cloud->points[i])) {
        seen->indices.push_back(i);
      }
    }

    // Extract seen cloud members from viewed_cloud
    // ROS_INFO("Extracting viewed frontier indices...");
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(viewed_cloud);
    extract.setIndices(seen);
    extract.setNegative(false);
    extract.filter(*viewed_cloud);

    // Add the pose and the viewed frontier indices to the goalViews vector.
    // ROS_INFO("Adding view to vector of potential goal views");
    View sampleView;
    sampleView.pose = goalPose;
    float posePoint[3] = {goalPose.position.x, goalPose.position.y, goalPose.position.z};
    sampleView.index = xyz_index3(posePoint);
    for (int i = 0; i < viewed_cloud->points.size(); i++) {
      sampleView.cloud.points.push_back(viewed_cloud->points[i]);
    }
    goalViews.push_back(sampleView);

    ROS_INFO("Sampled view from [%0.2f, %0.2f, %0.2f] sees %d frontier voxels.", sampleView.pose.position.x, sampleView.pose.position.y, sampleView.pose.position.z, (int)sampleView.cloud.points.size());
  }
  ROS_INFO("Sampled %d possible goal poses for viewing the frontier.", (int)goalViews.size());

  // // Get all the indices in the unviewable clusters
  // for (int cluster=0; cluster<(int)frontierClusterIndices.size(); cluster++) {
  //   if (clusterViewCounts[cluster] == 0) {
  //     // Remove this cluster from the frontiers and the frontierCloud
  //     for (int remove_index=0; remove_index<(int)frontierClusterIndices[cluster].indices.size(); remove_index++) {
  //       removeablePoints->indices.push_back(frontierClusterIndices[cluster].indices[remove_index]);
  //     }
  //   }
  // }

  // ROS_INFO("Removing %d unviewable voxels from the frontier...", (int)removeablePoints->indices.size());
  // // Remove the filtered frontier voxels from the boolean storage array
  // for (int i=0; i<(int)removeablePoints->indices.size(); i++) {
  //   float query[3];
  //   query[0] = frontierCloud->points[removeablePoints->indices[i]].x;
  //   query[1] = frontierCloud->points[removeablePoints->indices[i]].y;
  //   query[2] = frontierCloud->points[removeablePoints->indices[i]].z;
  //   int idx = xyz_index3(query);
  //   frontier[idx] = false;
  // }
  // // Remove the filtered frontier voxels from the PointCloud pointer.
  // pcl::ExtractIndices<pcl::PointXYZ> extract;
  // extract.setInputCloud(frontierCloud);
  // extract.setIndices(removeablePoints);
  // extract.setNegative(true);
  // extract.filter(*frontierCloud);
}

Eigen::Matrix3f quaternion2RotationMatrix(Quaternion q)
{
	// Compute the 3-2-1 rotation matrix for the vehicle from its current orientation in quaternions (q)
  Eigen::Matrix3f R;

  // First Row
  R(0,0) = q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z;
  R(0,1) = 2.0*(q.x*q.y - q.w*q.z);
  R(0,2) = 2.0*(q.w*q.y + q.x*q.z);
  // Second Row
  R(1,0) = 2.0*(q.x*q.y + q.w*q.z);
  R(1,1) = q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z;
  R(1,2) = 2.0*(q.y*q.z - q.w*q.x);
  // Third Row
  R(2,0) = 2.0*(q.x*q.z - q.w*q.y);
  R(2,1) = 2.0*(q.w*q.x + q.y*q.z);
  R(2,2) = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
}

void Msfm3d::getEuler()
{
  if (receivedPosition){
    // Roll
    float sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    euler[0] = atan2(sinr_cosp, cosr_cosp);
    // Pitch
    float sinp = +2.0 * (q.w*q.y - q.z*q.x);
    if (std::fabs(sinp) >= 1)
      euler[1] = std::copysign(M_PI / 2, sinp);
    else
      euler[1] = std::asin(sinp);
    // Yaw
    float siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    euler[2] = std::atan2(siny_cosp, cosy_cosp);
  }
}

void Msfm3d::callback_position(const nav_msgs::Odometry msg)
{
  if (!receivedPosition) receivedPosition = 1;
  position[0] = msg.pose.pose.position.x;
  position[1] = msg.pose.pose.position.y;
  position[2] = msg.pose.pose.position.z;
  q.x = msg.pose.pose.orientation.x;
  q.y = msg.pose.pose.orientation.y;
  q.z = msg.pose.pose.orientation.z;
  q.w = msg.pose.pose.orientation.w;
  ROS_INFO("Robot pose updated!");
}

void Msfm3d::callback_Octomap(const octomap_msgs::Octomap::ConstPtr msg)
{
  ROS_INFO("Getting OctoMap message...");
  if (!receivedMap) receivedMap = 1;
  if (!updatedMap) updatedMap = 1;

  // Free/Allocate the tree memory
  // ROS_INFO("Converting Octomap msg to AbstractOcTree...");
  // octomap::AbstractOcTree* abstree = new octomap::AbstractOcTree(msg->resolution);
  // abstree = octomap_msgs::binaryMsgToMap(*msg); // OcTree object for storing Octomap data.
  // ROS_INFO("Octomap converted to AbstractOcTree.");
  delete mytree;
  mytree = new octomap::OcTree(msg->resolution);
  mytree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
  ROS_INFO("AbstractOcTree cast into OcTree.");

  ROS_INFO("Parsing Octomap...");
  // Make sure the tree is at the same resolution as the esdf we're creating.  If it's not, change the resolution.
  ROS_INFO("Tree resolution is %f meters.", mytree->getResolution());
  if ((mytree->getResolution() - (double)voxel_size) > (double)0.01) {
    ROS_INFO("Planner voxel_size is %0.2f and Octomap resolution is %0.2f.  They must be equivalent.", voxel_size, mytree->getResolution());
    receivedMap = 0;
    return;
  }

  // Parse out ESDF struct dimensions from the the AbstractOcTree object
  double x, y, z;
  mytree->getMetricMin(x, y, z);
  ROS_INFO("Got Minimum map dimensions.");
  esdf.min[0] = (float)x - 1.5*voxel_size;
  esdf.min[1] = (float)y - 1.5*voxel_size;
  esdf.min[2] = (float)z - 1.5*voxel_size;
  mytree->getMetricMax(x, y, z);
  ROS_INFO("Got Maximum map dimensions.");
  esdf.max[0] = (float)x + 1.5*voxel_size;
  esdf.max[1] = (float)y + 1.5*voxel_size;
  esdf.max[2] = (float)z + 1.5*voxel_size;
  for (int i=0; i<3; i++) esdf.size[i] = roundf((esdf.max[i]-esdf.min[i])/voxel_size) + 1;

  // Print out the max and min values with the size values.
  ROS_INFO("The (x,y,z) ranges are (%0.2f to %0.2f, %0.2f to %0.2f, %0.2f to %0.2f).", esdf.min[0], esdf.max[0], esdf.min[1], esdf.max[1], esdf.min[2], esdf.max[2]);
  ROS_INFO("The ESDF dimension sizes are %d, %d, and %d.", esdf.size[0], esdf.size[1], esdf.size[2]);

  // Free and allocate memory for the esdf.data and esdf.seen pointer arrays
  delete[] esdf.data;
  esdf.data = NULL;
  esdf.data = new double [esdf.size[0]*esdf.size[1]*esdf.size[2]] { }; // Initialize all values to zero.
  delete[] esdf.seen;
  esdf.seen = NULL;
  esdf.seen = new bool [esdf.size[0]*esdf.size[1]*esdf.size[2]] { }; // Initialize all values to zero.

  // Initialize the voxels within a vehicle volume around the robot the free.  They may become overwritten by occupied, but that's okay.
  if (receivedPosition) {
    float query_point[3];
    int query_idx;
    if (vehicleVolume.set) {
      for(float dx = vehicleVolume.xmin; dx <= vehicleVolume.xmax; dx = dx + voxel_size) {
        query_point[0] = position[0] + dx;
        for(float dy = vehicleVolume.ymin; dx <= vehicleVolume.ymax; dy = dy + voxel_size) {
          query_point[1] = position[1] + dy;
          for(float dz = vehicleVolume.zmin; dz <= vehicleVolume.zmax; dz = dz + voxel_size) {
            query_point[2] = position[2] + dz;
            query_idx = xyz_index3(query_point);
            if ((query_idx >= 0) && (query_idx < esdf.size[0]*esdf.size[1]*esdf.size[2])) { // Check for valid array indices
              esdf.data[query_idx] = 1.0;
              esdf.seen[query_idx] = true;
            }
          }
        }
      }
    }
  }

  // Loop through tree and extract occupancy info into esdf.data and seen/not into esdf.seen
  double size, value;
  float point[3], lower_corner[3];
  int idx, depth, width;
  int lowest_depth = (int)mytree->getTreeDepth();
  int count = 0;
  int freeCount = 0;
  int occCount = 0;
  ROS_INFO("Starting tree iterator on OcTree with max depth %d", lowest_depth);
  for(octomap::OcTree::leaf_iterator it = mytree->begin_leafs(),
       end=mytree->end_leafs(); it!=end; ++it)
  {
    // Get data from node
    depth = (int)it.getDepth();
    point[0] = (float)it.getX();
    point[1] = (float)it.getY();
    point[2] = (float)it.getZ();
    size = it.getSize();
    // if (!(count % 500)) {
    //   ROS_INFO("Binary occupancy at [%0.2f, %0.2f, %0.2f] is: %d", point[0], point[1], point[2], (int)it->getValue());
    //   std::cout << it->getValue() << std::endl;
    // }
    if (it->getValue() > 0) {
      value = 0.0;
      occCount++;
    } else {
      value = 1.0;
      freeCount++;
    }

    // Put data into esdf
    if (depth == lowest_depth){
      idx = xyz_index3(point);
      // std::cout << "Node value: " << it->getValue() << std::endl;
      // ROS_INFO("Assigning an ESDF value at index %d with depth %d and size %f. (%f, %f, %f)", idx, depth, size, point[0], point[1], point[2]);
      esdf.data[idx] = value;
      esdf.seen[idx] = 1;
    } else{ // Fill in all the voxels internal to the leaf
      width = (int)std::pow(2.0, (double)(lowest_depth-depth));
      for (int i=0; i<3; i++){
        lower_corner[i] = point[i] - size/2.0 + voxel_size/2.0;
      }
      // ROS_INFO("Point (%f, %f, %f) is not at the base depth.  It is %d voxels wide.", point[0], point[1], point[2], width);
      // ROS_INFO("Filling in leaf at depth %d with size %f.  The lower corner is at (%f, %f, %f)", depth, size, lower_corner[0], lower_corner[1], lower_corner[2]);
      for (int i=0; i<width; i++){
        point[0] = lower_corner[0] + i*voxel_size;
        for (int j=0; j<width; j++){
          point[1] = lower_corner[1] + j*voxel_size;
          for (int k=0; k<width; k++){
            point[2] = lower_corner[2] + k*voxel_size;
            idx = xyz_index3(point);
            esdf.data[idx] = value;
            esdf.seen[idx] = 1;
          }
        }
      }
    }
  }

  ROS_INFO("Octomap message received.  %d leaves labeled as occupied.  %d leaves labeled as free.", occCount, freeCount);
}

void Msfm3d::callback_Octomap_freePCL(const sensor_msgs::PointCloud2 msg)
{
  pcl::PointCloud<pcl::PointXYZ> freeCloud;

}

void Msfm3d::callback(sensor_msgs::PointCloud2 msg)
{
  ROS_INFO("Getting ESDF PointCloud2...");
  if (!receivedMap) receivedMap = 1;
  PC2msg = msg;
  ROS_INFO("ESDF PointCloud2 received!");
}

void Msfm3d::parsePointCloud()
{
  ROS_INFO("Allocating memory for esdf parsing.");
  // integer array to store uint8 byte values
  int bytes[4];
  // local pointcloud storage array
  // float xyzis[5*(PC2msg.width)];
  float * xyzis = new float[5*(PC2msg.width)];
  // pointcloud xyz limits
  float xyzi_max[4], xyzi_min[4];
  int offset;

  // parse pointcloud2 data
  ROS_INFO("Parsing ESDF data into holder arrays.");
  for (int i=0; i<(PC2msg.width); i++) {
    for (int j=0; j<5; j++) {
      if (j>3){ offset = 12; }
      else{ offset = PC2msg.fields[j].offset; }
      for (int k=0; k<4; k++) {
        bytes[k] = PC2msg.data[32*i+offset+k];
      }
      xyzis[5*i+j] = byte2Float(bytes);
      if (j<4){
        if (i < 1){
          xyzi_max[j] = xyzis[5*i+j];
          xyzi_min[j] = xyzis[5*i+j];
        }
        else {
          if (xyzis[5*i+j] > xyzi_max[j]) xyzi_max[j] = xyzis[5*i+j];
          if (xyzis[5*i+j] < xyzi_min[j]) xyzi_min[j] = xyzis[5*i+j];
        }
      }
    }
  }

  // Replace max, min, and size esdf properties
  for (int i=0; i<4; i++) { esdf.max[i] = xyzi_max[i] + voxel_size; esdf.min[i] = xyzi_min[i] - voxel_size; } // Add 1 voxel to the maxes and subtract 1 voxel from the mins to get desired frontier performance
  for (int i=0; i<3; i++) esdf.size[i] = roundf((esdf.max[i]-esdf.min[i])/voxel_size) + 1;

  // Print out the max and min values with the size values.
  ROS_INFO("The (x,y,z) ranges are (%0.2f to %0.2f, %0.2f to %0.2f, %0.2f to %0.2f).", esdf.min[0], esdf.max[0], esdf.min[1], esdf.max[1], esdf.min[2], esdf.max[2]);
  ROS_INFO("The ESDF dimension sizes are %d, %d, and %d.", esdf.size[0], esdf.size[1], esdf.size[2]);

  // Empty current esdf and seen matrix and create a new ones.
  delete[] esdf.data;
  esdf.data = NULL;
  esdf.data = new double [esdf.size[0]*esdf.size[1]*esdf.size[2]] { }; // Initialize all values to zero.
  delete[] esdf.seen;
  esdf.seen = NULL;
  esdf.seen = new bool [esdf.size[0]*esdf.size[1]*esdf.size[2]] { };
  ROS_INFO("ESDF Data is of length %d", esdf.size[0]*esdf.size[1]*esdf.size[2]);
  ROS_INFO("Message width is %d", PC2msg.width);

  // Parse xyzis into esdf_mat
  int index;
  float point[3];
  for (int i=0; i<(5*(PC2msg.width)); i=i+5) {
    point[0] = xyzis[i]; point[1] = xyzis[i+1]; point[2] = xyzis[i+2];
    index = xyz_index3(point);
    if (index > (esdf.size[0]*esdf.size[1]*esdf.size[2])) ROS_INFO("WARNING: Parsing index is greater than array sizes!");
    // Use the hyperbolic tan function from the btraj paper
    // esdf = v_max*(tanh(d - e) + 1)/2
    if (xyzis[i+3] >= 0.0) {
      esdf.data[index] = (double)(5.0*(tanh(xyzis[i+3] - exp(1.0)) + 1.0)/2.0);
    }
    else {
      esdf.data[index] = (double)(0.0);
    }
    esdf.seen[index] = (xyzis[i+4]>0.0);
  }

  // Free xyzis holder array memory
  delete[] xyzis;
}

void Msfm3d::inflateObstacles(const float radius, sensor_msgs::PointCloud2& inflatedOccupiedMsg) {
  int npixels = esdf.size[0]*esdf.size[1]*esdf.size[2];
  ROS_INFO("Allocating boolean of size %d.", npixels);
  std::vector<bool> changed_value(npixels, 0);
  ROS_INFO("Success!");
  float current_point[3];
  float query_point[3];
  int pointsInflated = 0;
  int query_idx;
  pcl::PointXYZ changed_point;
  pcl::PointCloud<pcl::PointXYZ>::Ptr inflatedOccupied(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < npixels; i++) {
    // if ((i % (npixels/10)) == 0) {
    //   ROS_INFO("Checking if pixel %d needs to be inflated.", i);
    // }
    if (esdf_or_octomap) {
      if ((esdf.data[i] <= 0.05) && !changed_value[i] && esdf.seen[i]) { // Check if the entry is occupied and has not been modified by this function.
        // Get the xyz coords of the current voxels index
        index3_xyz(i, current_point);
        query_point[2] = current_point[2];
        // Loop through all voxels within a side length of 2*radius box of the current index.  Only do x-y directions for ground vehicle.
        if (ground) {
          for(float dx = -radius; dx <= radius; dx = dx + voxel_size) {
            query_point[0] = current_point[0] + dx;
            for(float dy = -radius; dy <= radius; dy = dy + voxel_size) {
              query_point[1] = current_point[1] + dy;
              query_idx = xyz_index3(query_point);
              if ((query_idx >= 0) && (query_idx < npixels)) { // Check for valid array indices
                if ((esdf.data[query_idx] > 0.05) && esdf.seen[query_idx] && (dist3(query_point, position) >= 2*voxel_size)) { // Check if the voxel has been labeled as free/seen so it may be changed and that it's not within 2 voxel radius of the vehicle
                  esdf.data[query_idx] = 0.0;
                  changed_value[query_idx] = true;
                  changed_point.x = query_point[0];
                  changed_point.y = query_point[1];
                  changed_point.z = query_point[2];
                  inflatedOccupied->points.push_back(changed_point);
                  pointsInflated++;
                }
              }
            }
          }
        } else {
          for(float dx = -radius; dx <= radius; dx = dx + voxel_size) {
            query_point[0] = current_point[0] + dx;
            for(float dy = -radius; dy <= radius; dy = dy + voxel_size) {
              query_point[1] = current_point[1] + dy;
              for(float dz = -radius; dz <= radius; dz = dz + voxel_size) {
                query_point[2] = current_point[2] + dz;
                query_idx = xyz_index3(query_point);
                if ((query_idx >= 0) && (query_idx < npixels)) { // Check for valid array indices
                  if ((esdf.data[query_idx] > 0.05) && esdf.seen[query_idx]) { // Check if the voxel has been labeled as free/seen so it may be changed
                    esdf.data[query_idx] = 0.0;
                    changed_value[query_idx] = true;
                    changed_point.x = query_point[0];
                    changed_point.y = query_point[1];
                    changed_point.z = query_point[2];
                    inflatedOccupied->points.push_back(changed_point);
                    pointsInflated++;
                  }
                }
              }
            }
          }
        }
      }
    } else if (esdf.seen[i]) {
      esdf.data[i] = esdf.data[i] - (double)radius; // Just subtract the value for an esdf since the distance is included.
      pointsInflated++;
    }
  }
  // Convert PointCloud to ROS msg
  pcl::toROSMsg(*inflatedOccupied, inflatedOccupiedMsg);

  ROS_INFO("%d points were made occupied by inflating the map by %0.2f meters.", pointsInflated, radius);
}

void Msfm3d::index3_xyz(const int index, float point[3])
{
  // x+y*sizx+z*sizx*sizy
  point[2] = esdf.min[2] + (index/(esdf.size[1]*esdf.size[0]))*voxel_size;
  point[1] = esdf.min[1] + ((index % (esdf.size[1]*esdf.size[0]))/esdf.size[0])*voxel_size;
  point[0] = esdf.min[0] + ((index % (esdf.size[1]*esdf.size[0])) % esdf.size[0])*voxel_size;
}

int Msfm3d::xyz_index3(const float point[3])
{
  int ind[3];
  for (int i=0; i<3; i++) ind[i] = roundf((point[i]-esdf.min[i])/voxel_size);
  return mindex3(ind[0], ind[1], ind[2], esdf.size[0], esdf.size[1]);
}

bool Msfm3d::updatePath(const float goal[3])
{
  int npixels = esdf.size[0]*esdf.size[1]*esdf.size[2], idx; // size of the reachability array, index in reachability array of the current path voxel.
  int neighbor[6]; // neighbor voxels to the current voxel
  float point[3]; // current point x,y,z coordinates
  float query[3]; // intermediate x,y,z coordinates for another operation
  float grad[3]; // gradient at the current point
  float grad_norm = 1.0; // norm of the gradient vector
  float dist_robot2path = 10.0*voxel_size; // distance of the robot to the path
  float position2D[2];
  float point2D[2];
  std::vector<float> path;

  // If the goal point isn't in the reachable map, return false
  int goal_idx = xyz_index3(goal);
  if (goal_idx < 0 || goal_idx > npixels){
    ROS_WARN("Goal point is not reachable.");
    return false;
  }
  if (reach[goal_idx] <= 0.0 || reach[goal_idx] >= 1e6) {
    ROS_WARN("Goal point is either too far away or is blocked by an obstacle.");
    return false;
  }

  // 3D Interpolation intermediate values from https://en.wikipedia.org/wiki/Trilinear_interpolation
  float step = voxel_size/2.0;

  // Path message for ROS
  nav_msgs::Path newpathmsg;
  geometry_msgs::PoseStamped pose;
  newpathmsg.header.frame_id = frame;

  // Clear the path vector to prep for adding new elements.
  for (int i=0; i<3; i++){
    point[i] = goal[i];
    path.push_back(point[i]);
  }

  // Run loop until the path is within a voxel of the robot.
  while ((dist_robot2path > 2.0*voxel_size) && (path.size() < 30000) && grad_norm >= 0.00001) {
  	// Find the current point's grid indices and it's 6 neighbor voxel indices.
  	idx = xyz_index3(point);
    neighbor[0] = idx - 1; // i-1
    neighbor[1] = idx + 1; // i+1
    neighbor[2] = idx - esdf.size[0]; // j-1
    neighbor[3] = idx + esdf.size[0]; // j+1
    neighbor[4] = idx - esdf.size[0]*esdf.size[1]; // k-1
    neighbor[5] = idx + esdf.size[0]*esdf.size[1]; // k+1
    // ROS_INFO("Neighbor indices found of corner point %d", i);
    for (int j=0; j<3; j++) {
      grad[j] = 0; // Initialize to zero in case neither neighbor has been assigned a reachability value.
      // Check if the neighbor voxel has no reachability matrix value
      if (reach[neighbor[2*j]] > 0.0 && reach[neighbor[2*j+1]] > 0.0){
        grad[j] = 0.5*(float)(reach[neighbor[2*j]] - reach[neighbor[2*j+1]]); // Central Difference Operator (Try Sobel if this is bad)
        // ROS_INFO("Using Central Difference Operator for %d coordinate for path gradient.", j);
      } else {
        if (reach[neighbor[2*j]] > 0.0) {
          grad[j] = 0.5*(float)(reach[neighbor[2*j]] - reach[idx]); // Intermediate Difference Operator (with previous neighbor)
          // ROS_INFO("Using Backward Intermediate Difference Operator for %d coordinate for path gradient.", j);
        }
        if (reach[neighbor[2*j + 1]] > 0.0) {
          grad[j] = 0.5*(float)(reach[idx] - reach[neighbor[2*j+1]]); // Intermediate Difference Operator (with next neighbor)
          // ROS_INFO("Using Forward Intermediate Difference Operator for %d coordinate for path gradient.", j);
        }
      }
    }

    // Normalize the size of the gradient vector if it is too large
    grad_norm = std::sqrt(grad[0]*grad[0] + grad[1]*grad[1] + grad[2]*grad[2]);
    if (grad_norm > 1.0){
      for (int i=0; i<3; i++) grad[i] = std::sqrt(1.0)*grad[i]/grad_norm;
    }
    if (grad_norm < 0.05){
      for (int i=0; i<3; i++) grad[i] = std::sqrt(0.05)*grad[i]/grad_norm;
    }

    // ROS_INFO("3D Interpolation performed.");
    // ROS_INFO("Gradients: [%f, %f, %f]", grad[0], grad[1], grad[2]);
    // Update point and add to path
    for (int i=0; i<3; i++) {
      if (grad_norm >= 0.00001){
        point[i] = point[i] + step*grad[i];
        path.push_back(point[i]);
      }
    }
    // ROS_INFO("[%f, %f, %f] added to path.", point[0], point[1], point[2]);

    // Update the robot's distance to the path
    if (ground){
      position2D[0] = position[0];
      position2D[1] = position[1];
      point2D[0] = point[0];
      point2D[1] = point[1];
      dist_robot2path = dist2(position2D, point2D);
    }
    else {
      dist_robot2path = dist3(position, point);
    }
  }

  // Check if the path made it back to the vehicle
  if (dist_robot2path > 2.0*voxel_size) {
    ROS_WARN("Path did not make it back to the robot.  Select a different goal point.");
    return false;
  }

  // Add path vector to path message for plotting in rviz
  ROS_INFO("Path finished of length %d", (int)(path.size()/3.0));
  for (int i=(path.size()-3); i>=0; i=i-3){
    pose.header.frame_id = frame;
    pose.pose.position.x = path[i];
    pose.pose.position.y = path[i+1];
    pose.pose.position.z = path[i+2];
    newpathmsg.poses.push_back(pose);
  }
  pathmsg = newpathmsg;
  return true;
}


void Msfm3d::updateFrontierMsg() {
  // Declare a new cloud to store the converted message
  sensor_msgs::PointCloud2 newPointCloud2;

  // Convert from pcl::PointCloud to sensor_msgs::PointCloud2
  pcl::toROSMsg(*frontierCloud, newPointCloud2);
  newPointCloud2.header.seq = 1;
  newPointCloud2.header.stamp = ros::Time();
  newPointCloud2.header.frame_id = frame;

  // Update the old message
  frontiermsg = newPointCloud2;
}

bool updateFrontier(Msfm3d& planner){
  ROS_INFO("Beginning Frontier update step...");

  // Expand the frontier vector to be at least as big as the esdf space
  // Make sure to copy the previous frontier data



  //**Plan on updating the frontier only in a sliding window of grid spaces surrounding the vehicle**


  int npixels = planner.esdf.size[0]*planner.esdf.size[1]*planner.esdf.size[2];
  delete[] planner.frontier;
  ROS_INFO("Previous frontier deleted.  Allocating new frontier of size %d...", npixels);
  planner.frontier = NULL;
  planner.frontier = new bool [npixels] { }; // Initialize the frontier array as size npixels with all values false.

  // Initialize the pointCloud version of the frontier
  planner.frontierCloud->clear();
  ROS_INFO("New frontier array added with all values set to false.");

  delete[] planner.entrance;
  planner.entrance = NULL;
  planner.entrance = new bool [npixels] { }; // Initialize the entrance array as size npixels with all values false.

  float point[3], query[3];
  int neighbor[6];
  bool frontier = 0;
  pcl::PointXYZ _point;

  int frontierCount = 0;
  int pass1 = 0;
  int pass2 = 0;
  int pass3 = 0;
  int pass4 = 0;
  int pass5 = 0;

  // Extra variables for ground vehicle case so that only frontier close to vehicle plane are chosen.
  for (int i=0; i<npixels; i++){

    // Check if the voxel has been seen and is unoccupied
    // if (planner.esdf.seen[i] && (planner.esdf.data[i]>0.0) && (dist3(planner.position, point) >= planner.bubble_radius)) {
    if (planner.esdf.seen[i] && (planner.esdf.data[i]>0.0)) {
      pass1++;
      // Get the 3D point location
      planner.index3_xyz(i, point);
      _point.x = point[0];
      _point.y = point[1];
      _point.z = point[2];

      // Check if the voxel is a frontier by querying adjacent voxels
      for (int j=0; j<3; j++) query[j] = point[j];

      // Create an array of neighbor indices
      for (int j=0; j<3; j++){
        if (point[j] < (planner.esdf.max[j] - planner.voxel_size)) {
          query[j] = point[j] + planner.voxel_size;
        }
        neighbor[2*j] = planner.xyz_index3(query);
        if (point[j] > (planner.esdf.min[j] + planner.voxel_size)) {
          query[j] = point[j] - planner.voxel_size;
        }
        neighbor[2*j+1] = planner.xyz_index3(query);
        query[j] = point[j];
      }

      // Check if the neighbor indices are unseen voxels
      if (planner.ground) {
        for (int j=0; j<4; j++) {
          if (!planner.esdf.seen[neighbor[j]] && !(i == neighbor[j]) && !frontier) {
            frontier = 1;
            pass2++;
          }
        }
        // Eliminate frontiers with unseen top/bottom neighbors
        if ((!planner.esdf.seen[neighbor[4]] && i != neighbor[4]) || (!planner.esdf.seen[neighbor[5]] && i != neighbor[5])) {
          frontier = 0;
          pass3++;
        }
      }
      else {
        // For the time being, exclude the top/bottom neighbor (last two neighbors)
        // for (int j=0; j<6; j++) {
        for (int j=0; j<4; j++) {
          if (!planner.esdf.seen[neighbor[j]]  && !(i == neighbor[j])) {
            frontier = 1;
          }
        }
        // if (!planner.esdf.seen[neighbor[5]]  && !(i == neighbor[5])) frontier = 1;
      }
      // Check if the point is on the ground if it is a ground robot
      if (frontier) {
        // Only consider frontiers on the floor
        // if (planner.esdf.data[neighbor[5]] > (0.01)) frontier = 0;
        // if (!planner.esdf.seen[neighbor[5]]) frontier = 0;

        // Only consider frontiers close in z-coordinate (temporary hack)
        if (planner.dzFrontierVoxelWidth > 0) {
          if (abs(planner.position[2] - point[2]) >= planner.dzFrontierVoxelWidth*planner.voxel_size) {
            pass3++;
            frontier = 0;
          }
        }

        // Eliminate frontiers that are adjacent to occupied cells
        for (int j=0; j<6; j++) {
          if (planner.esdf.data[neighbor[j]] < (0.01) && planner.esdf.seen[neighbor[j]]) {
            pass4++;
            frontier = 0;
          }
        }
      }

      // Check if the voxel is at the entrance

      if (frontier && (dist3(point, planner.origin) <= planner.entranceRadius)) {
        pass5++;
        planner.entrance[i] = 1;
        frontier = 0;
      }

      // If the current voxel is a frontier, add the  current voxel location to the planner.frontier array
      if (frontier) {
        frontier = 0; // reset for next loop
        planner.frontier[i] = 1;
        frontierCount++;
        planner.frontierCloud->push_back(_point);
      }
    }
  }
  ROS_INFO("%d points are free.  %d points are free and adjacent to unoccupied voxels.  %d points filtered for being too high/low.  %d points filtered for being adjacent to occupied voxels.  %d points at entrance.", pass1, pass2, pass3, pass4, pass5);
  ROS_INFO("Frontier updated. %d voxels initially labeled as frontier.", frontierCount);

  if (frontierCount == 0) {
    return false;
  }

  // Cluster the frontier into euclidean distance groups
  if (planner.clusterFrontier(false)) {
    // Group frontier within each cluster with greedy algorithm
    planner.greedyGrouping(4*planner.voxel_size, false);
    return true;
  } else {
    return false;
  }
}

void findFrontier(Msfm3d& planner, float frontierList[15], double cost[5])
{
  // findFrontier scans through the environment arrays and returns the 5 closest frontier locations in reachability distance.
  int npixels = planner.esdf.size[0]*planner.esdf.size[1]*planner.esdf.size[2];
  float point[3];
  int slot = 0;

  // Initialize cost list with large descending values.
  for (int i=0; i<5; i++) cost[i] = 1e6 - 100*i;
  // Main
  for (int i=0; i<npixels; i++){
    // Check if the voxel has been seen, is unoccupied and it costs less to reach it than the 5 cheapest voxels
    if (planner.esdf.seen[i] && (planner.reach[i]<cost[0]) && (planner.esdf.data[i]>0 && planner.reach[i] > (double)0.0)){
      // // Check if the voxel is a frontier by querying adjacent voxels
      planner.index3_xyz(i, point);

      // If the current voxel is a frontier, add the current voxel location to the cost and frontierList
      if (planner.frontier[i]) {
        // frontier = 0; // reset for next loop
        // Put the new point in the correct slot
        for (int j=0; j<5; j++) {
          if (planner.reach[i]<cost[j]) slot = j;
        }
        for (int j=0; j<slot; j++) {
          cost[j] = cost[j+1];
          for (int k=0; k<3; k++) frontierList[3*j+k] = frontierList[3*(j+1)+k];
        }
        cost[slot] = planner.reach[i];
        for (int j=0; j<3; j++) frontierList[3*slot+j] = point[j];
        slot = 0;
      }
    }
  }
}

void closestGoalView(Msfm3d& planner, int viewIndices[5], double cost[5])
{
  // findFrontier scans through the environment arrays and returns the 5 closest frontier locations in reachability distance.
  float point[3];
  int slot = 0;
  int idx; // index of the current view pose in the reach matrix

  // Initialize cost list with large descending values.
  for (int i=0; i<5; i++) {
    cost[i] = 1e6 - 100*i;
    viewIndices[i] = -1;
  }

  ROS_INFO("Finding the closest viewpoint to see frontiers.");
  // Main
  for (int i=0; i<planner.goalViews.size(); i++){
    point[0] = planner.goalViews[i].pose.position.x;
    point[1] = planner.goalViews[i].pose.position.y;
    point[2] = planner.goalViews[i].pose.position.z;
    idx = planner.xyz_index3(point);
    if ((idx < 0) || (idx > planner.esdf.size[0]*planner.esdf.size[1]*planner.esdf.size[2])) {
      continue;
    }
    if (planner.reach[idx] > (double)0.0) {
      // Put the new point in the correct slot
      for (int j=0; j<5; j++) {
        if (planner.reach[idx]<cost[j]) slot = j;
      }
      for (int j=0; j<slot; j++) {
        cost[j] = cost[j+1];
        viewIndices[j] = viewIndices[j+1];
      }
      cost[slot] = planner.reach[idx];
      viewIndices[slot] = i;
      slot = 0;
    }
  }
}

void infoGoalView(Msfm3d& planner, int viewIndices[5], double utility[5])
{
   // infoGoalView finds the most "informative" View in the planner.goalViews vector.
  float point[3];
  float cost_time; // seconds
  int slot = 0;
  int idx; // index of the current view pose in the reach matrix
  double current_utility;
  bool calculate_actual_cost = false;

  // Initialize utility list with negative ascending values.
  for (int i=0; i<5; i++) {
    utility[i] = -5+i;
    viewIndices[i] = -1;
  }

  // Find the 5 views with the greatest utility (frontier voxels/second) and arrange them in ascending order in utility[5]
  for (int i=0; i<planner.goalViews.size(); i++) {
    point[0] = planner.goalViews[i].pose.position.x;
    point[1] = planner.goalViews[i].pose.position.y;
    point[2] = planner.goalViews[i].pose.position.z;
    idx = planner.xyz_index3(point);
    // Check to see if the goal view has a cost to travel to it (if not we can't generate a path anyways)
    if (planner.reach[idx] > (double)0.0) {

      // Convert the reachability to the time it takes the robot to get there in seconds (point to point)
      cost_time = (planner.reach[idx]*planner.voxel_size)/planner.speed; // (voxels*meters/voxel)/(meters/second)

      // Do a preliminary calculation the point's utility
      current_utility = std::sqrt((double)(planner.goalViews[i].cloud.points.size())/cost_time);

      // ROS_INFO("[%.2f, %.2f, %.2f] sees %d voxels at a %.2f point-to-point seconds to travel and has a utility of %.2f.", point[0], point[1], point[2], (int)planner.goalViews[i].cloud.points.size(), cost_time, current_utility);

      // Check to see if the optimisitic current_utility is less than any of the top 5 values.
      // The optimistic current_utility doesn't account for the cost to turn in place at the start and end of the path.
      for (int j=0; j<5; j++) {
        if (current_utility>utility[j]) {
          calculate_actual_cost = true;
        }
      }

      if (calculate_actual_cost) {

        // Calculate the feasability and path to arrive at the current goal view
        if (planner.updatePath(point)) {
          // Get the angle between the current robot pose and the path start
          float start_path_vec[3] = {(float)(planner.pathmsg.poses[1].pose.position.x - planner.pathmsg.poses[0].pose.position.x),
                                     (float)(planner.pathmsg.poses[1].pose.position.y - planner.pathmsg.poses[0].pose.position.y),
                                     (float)(planner.pathmsg.poses[1].pose.position.z - planner.pathmsg.poses[0].pose.position.z)};
          float start_path_yaw = std::atan2(start_path_vec[1], start_path_vec[0]); // degrees
          float vehicle_yaw = std::atan2(2.0*(planner.q.w*planner.q.z + planner.q.x*planner.q.y), 1.0 - 2.0*(planner.q.y*planner.q.y + planner.q.z*planner.q.z)); // degrees

          // Get the angle between the path end and the goal pose
          int end_path_idx = (int)planner.pathmsg.poses.size()-1;
          float end_path_vec[3] = {(float)(planner.pathmsg.poses[end_path_idx].pose.position.x - planner.pathmsg.poses[end_path_idx-1].pose.position.x),
                                   (float)(planner.pathmsg.poses[end_path_idx].pose.position.y - planner.pathmsg.poses[end_path_idx-1].pose.position.y),
                                   (float)(planner.pathmsg.poses[end_path_idx].pose.position.z - planner.pathmsg.poses[end_path_idx-1].pose.position.z)};;
          float end_path_yaw = std::atan2(end_path_vec[1], end_path_vec[0]); // degrees
          Quaternion goal_q = planner.goalViews[i].pose.q;
          float goal_yaw = std::atan2(2.0*(goal_q.w*goal_q.z + goal_q.x*goal_q.y), 1.0 - 2.0*(goal_q.y*goal_q.y + goal_q.z*goal_q.z)); // degrees

          // Add these angle differences to the cost_time
          // cost_time = cost_time + (std::abs(angle_diff((180.0/M_PI)*end_path_yaw, (180.0/M_PI)*goal_yaw)) + std::abs(angle_diff((180.0/M_PI)*start_path_yaw, (180.0/M_PI)*vehicle_yaw)))/planner.turnRate;
          cost_time = cost_time + std::abs(angle_diff((180.0/M_PI)*start_path_yaw, (180.0/M_PI)*vehicle_yaw))/planner.turnRate;

          // Calculate the actual utility
          current_utility = std::sqrt((double)(planner.goalViews[i].cloud.points.size())/cost_time); // frontier voxels/second

          ROS_INFO("[%.2f, %.2f, %.2f] sees %d voxels at a %.2f actual seconds to travel and has a utility of %.2f.", point[0], point[1], point[2], (int)planner.goalViews[i].cloud.points.size(), cost_time, current_utility);

          // See which slot the current view actually belongs in after accounting for the view's reachability and time to turn.
          for (int j=0; j<5; j++) {
            if (current_utility>utility[j]) {
              slot = j;
            }
          }

          // Add the current view utility and index to the correct slot in the top 5 list
          for (int j=0; j<slot; j++) {
            utility[j] = utility[j+1];
            viewIndices[j] = viewIndices[j+1];
          }
          utility[slot] = current_utility;
          viewIndices[slot] = i;

          // Reset flag variables
          slot = 0;
          calculate_actual_cost = false;
        }
      }
    }
  }
}

void findEntrance(Msfm3d& planner, float entranceList[15], double cost[5])
{
  // findEntrance scans through the environment arrays and returns the 5 closest entrance locations in reachability distance.
  int npixels = planner.esdf.size[0]*planner.esdf.size[1]*planner.esdf.size[2];
  float point[3];
  int slot = 0;

  // Initialize cost list with large descending values.
  for (int i=0; i<5; i++) cost[i] = 1e6 - 100*i;
  // Main
  for (int i=0; i<npixels; i++){
    // Check if the voxel has been seen, is unoccupied and it costs less to reach it than the 5 cheapest voxels
    if (planner.esdf.seen[i] && (planner.reach[i]<cost[0]) && (planner.esdf.data[i]>0 && planner.reach[i] > (double)0.0)){
      // // Check if the voxel is a frontier by querying adjacent voxels
      planner.index3_xyz(i, point);

      // If the current voxel is an entrance frontier, add the current voxel location to the cost and entrancelist
      if (planner.entrance[i]) {
        // frontier = 0; // reset for next loop
        // Put the new point in the correct slot
        for (int j=0; j<5; j++) {
          if (planner.reach[i]<cost[j]) slot = j;
        }
        for (int j=0; j<slot; j++) {
          cost[j] = cost[j+1];
          for (int k=0; k<3; k++) entranceList[3*j+k] = entranceList[3*(j+1)+k];
        }
        cost[slot] = planner.reach[i];
        for (int j=0; j<3; j++) entranceList[3*slot+j] = point[j];
        slot = 0;
      }
    }
  }
}

void view2MarkerMsg(const View cameraView, const SensorFoV camera, visualization_msgs::Marker& marker, Pose robot2camera)
{
  marker.type = 11; // Traingle List
  marker.action = visualization_msgs::Marker::ADD;

  // View Frustum Geometry
  Eigen::Vector3f camera_position;
  camera_position << cameraView.pose.position.x, cameraView.pose.position.y, cameraView.pose.position.z;
  Eigen::Vector3f origin;
  origin << 0.0, 0.0, 0.0;
  origin = origin + camera_position;
  Eigen::Vector3f corner1;
  corner1 << 1.0, std::sin((M_PI/180.0)*camera.horizontalFoV/2.0), std::sin((M_PI/180.0)*camera.verticalFoV/2.0);
  corner1 = cameraView.pose.R*robot2camera.R*(camera.rMax*corner1) + camera_position;
  Eigen::Vector3f corner2;
  corner2 << 1.0, -std::sin((M_PI/180.0)*camera.horizontalFoV/2.0), std::sin((M_PI/180.0)*camera.verticalFoV/2.0);
  corner2 = cameraView.pose.R*robot2camera.R*(camera.rMax*corner2) + camera_position;
  Eigen::Vector3f corner3;
  corner3 << 1.0, -std::sin((M_PI/180.0)*camera.horizontalFoV/2.0), -std::sin((M_PI/180.0)*camera.verticalFoV/2.0);
  corner3 = cameraView.pose.R*robot2camera.R*(camera.rMax*corner3) + camera_position;
  Eigen::Vector3f corner4;
  corner4 << 1.0, std::sin((M_PI/180.0)*camera.horizontalFoV/2.0), -std::sin((M_PI/180.0)*camera.verticalFoV/2.0);
  corner4 = cameraView.pose.R*robot2camera.R*(camera.rMax*corner4) + camera_position;

  // Write to marker.points[]
  geometry_msgs::Point robot;
  robot.x = origin(0);
  robot.y = origin(1);
  robot.z = origin(2);
  geometry_msgs::Point UL;
  UL.x = corner1(0);
  UL.y = corner1(1);
  UL.z = corner1(2);
  geometry_msgs::Point UR;
  UR.x = corner2(0);
  UR.y = corner2(1);
  UR.z = corner2(2);
  geometry_msgs::Point BR;
  BR.x = corner3(0);
  BR.y = corner3(1);
  BR.z = corner3(2);
  geometry_msgs::Point BL;
  BL.x = corner4(0);
  BL.y = corner4(1);
  BL.z = corner4(2);

  // clear points
  marker.points.clear();

  // First triangle
  marker.points.push_back(robot);
  marker.points.push_back(UL);
  marker.points.push_back(BL);

  // 2nd triangle
  marker.points.push_back(robot);
  marker.points.push_back(UL);
  marker.points.push_back(UR);

  // 3rd triangle
  marker.points.push_back(robot);
  marker.points.push_back(UR);
  marker.points.push_back(BR);

  // 4th triangle
  marker.points.push_back(robot);
  marker.points.push_back(BR);
  marker.points.push_back(BL);
}

void reach( Msfm3d& planner, const bool usesecond, const bool usecross, const int nFront, bool reachOrigin) {

    /* The input variables */
    double *F;
    int SourcePoints[3];

    /* The output distance image */
    double *T;

    /* Euclidian distance image */
    double *Y;

    /* Current distance values */
    double Tt, Ty;

    /* Matrix containing the Frozen Pixels" */
    bool *Frozen;

    /* Augmented Fast Marching (For skeletonize) */
    bool Ed;

    /* Size of input image */
    int dims[3];

    /* Size of  SourcePoints array */
    int dims_sp[3];

    /* Number of pixels in image */
    int npixels;

    /* Neighbour list */
    int neg_free;
    int neg_pos;
    double *neg_listv;
    double *neg_listx;
    double *neg_listy;
    double *neg_listz;
    double *neg_listo;

    int *listprop;
    double **listval;

    /* Neighbours 6x3 */
    int ne[18]={-1,  0,  0, 1, 0, 0, 0, -1,  0, 0, 1, 0, 0,  0, -1, 0, 0, 1};

    /* Loop variables */
    int s, w, itt, q;

    /* Current location */
    int x, y, z, i, j, k;

    /* Index */
    int IJK_index, XYZ_index, index;

    /* Frontier View Count */
    int frontCount = 0;

    // Parse input arguments to relevent function variables
    for (int i=0; i<3; i++){
        SourcePoints[i] = roundf((planner.position[i]-planner.esdf.min[i])/planner.voxel_size);
        dims[i] = planner.esdf.size[i];
    }
    F = planner.esdf.data; // Source esdf
    dims_sp[0] = 3;
    dims_sp[1] = 1;
    dims_sp[2] = 1;
    npixels=dims[0]*dims[1]*dims[2];
    Ed = 0;
    delete[] planner.reach;
    planner.reach = NULL;
    planner.reach = new double [npixels] { };
    T = planner.reach;

    /* Pixels which are processed and have a final distance are frozen */
    Frozen = new bool [npixels];
    for(q=0;q<npixels;q++){Frozen[q]=0; T[q]=-1;}
    if(Ed) {
        for(q=0;q<npixels;q++){Y[q]=-1;}
    }

    /*Free memory to store neighbours of the (segmented) region */
    neg_free = 100000;
    neg_pos=0;

    neg_listx = (double *)malloc( neg_free*sizeof(double) );
    neg_listy = (double *)malloc( neg_free*sizeof(double) );
    neg_listz = (double *)malloc( neg_free*sizeof(double) );
    if(Ed) {
        neg_listo = (double *)malloc( neg_free*sizeof(double) );
        for(q=0;q<neg_free;q++) { neg_listo[q]=0; }
    }

    /* List parameters array */
    listprop=(int*)malloc(3* sizeof(int));
    /* Make jagged list to store a maximum of 2^64 values */
    listval= (double **)malloc( 64* sizeof(double *) );

    /* Initialize parameter list */
    initialize_list(listval, listprop);
    neg_listv=listval[listprop[1]-1];

    /*(There are 3 pixel classes: */
    /*  - frozen (processed) */
    /*  - narrow band (boundary) (in list to check for the next pixel with smallest distance) */
    /*  - far (not yet used) */
    /* set all starting points to distance zero and frozen */
    /* and add all neighbours of the starting points to narrow list */
    for (s=0; s<dims_sp[1]; s++) {
        /*starting point */
        x= SourcePoints[0+s*3];
        y= SourcePoints[1+s*3];
        z= SourcePoints[2+s*3];
        XYZ_index=mindex3(x, y, z, dims[0], dims[1]);

        Frozen[XYZ_index]=1;
        T[XYZ_index]=0;
        if(Ed) { Y[XYZ_index]=0; }
    }

    for (s=0; s<dims_sp[1]; s++) {
        /*starting point */
        x= SourcePoints[0+s*3];
        y= SourcePoints[1+s*3];
        z= SourcePoints[2+s*3];

        XYZ_index=mindex3(x, y, z, dims[0], dims[1]);
        for (w=0; w<6; w++) {
            /*Location of neighbour */
            i=x+ne[w];
            j=y+ne[w+6];
            k=z+ne[w+12];

            IJK_index=mindex3(i, j, k, dims[0], dims[1]);

            /*Check if current neighbour is not yet frozen and inside the */

            /*picture */
            if(isntfrozen3d(i, j, k, dims, Frozen)) {
                Tt=(1/(max(F[IJK_index],eps)));
                /*Update distance in neigbour list or add to neigbour list */
                if(T[IJK_index]>0) {
                    if(neg_listv[(int)T[IJK_index]]>Tt) {
                        listupdate(listval, listprop, (int)T[IJK_index], Tt);
                    }
                }
                else {
                    /*If running out of memory at a new block */
                    if(neg_pos>=neg_free) {
                        neg_free+=100000;
                        neg_listx = (double *)realloc(neg_listx, neg_free*sizeof(double) );
                        neg_listy = (double *)realloc(neg_listy, neg_free*sizeof(double) );
                        neg_listz = (double *)realloc(neg_listz, neg_free*sizeof(double) );
                        if(Ed) {
                            neg_listo = (double *)realloc(neg_listo, neg_free*sizeof(double) );
                        }
                    }
                    list_add(listval, listprop, Tt);
                    neg_listv=listval[listprop[1]-1];
                    neg_listx[neg_pos]=i;
                    neg_listy[neg_pos]=j;
                    neg_listz[neg_pos]=k;
                    T[IJK_index]=neg_pos;
                    neg_pos++;
                }
            }
        }
    }

    /*Loop through all pixels of the image */
    for (itt=0; itt<(npixels); itt++) /* */ {
        /*Get the pixel from narrow list (boundary list) with smallest */
        /*distance value and set it to current pixel location */
        index=list_minimum(listval, listprop);
        neg_listv=listval[listprop[1]-1];
        /* Stop if pixel distance is infinite (all pixels are processed) */
        if(IsInf(neg_listv[index])) {
          break;
        }

        /*index=minarray(neg_listv, neg_pos); */
        x=(int)neg_listx[index]; y=(int)neg_listy[index]; z=(int)neg_listz[index];
        XYZ_index=mindex3(x, y, z, dims[0], dims[1]);
        Frozen[XYZ_index]=1;
        T[XYZ_index]=neg_listv[index];
        if(Ed) {
          Y[XYZ_index]=neg_listo[index];
        }

        /*Remove min value by replacing it with the last value in the array */
        list_remove_replace(listval, listprop, index) ;
        neg_listv=listval[listprop[1]-1];
        if(index<(neg_pos-1)) {
            neg_listx[index]=neg_listx[neg_pos-1];
            neg_listy[index]=neg_listy[neg_pos-1];
            neg_listz[index]=neg_listz[neg_pos-1];
            if(Ed) {
                neg_listo[index]=neg_listo[neg_pos-1];
            }
            T[(int)mindex3((int)neg_listx[index], (int)neg_listy[index], (int)neg_listz[index], dims[0], dims[1])]=index;
        }
        neg_pos =neg_pos-1;

        if (planner.frontier[XYZ_index]) frontCount++;

        if (XYZ_index == planner.xyz_index3(planner.origin)) reachOrigin = false;

        /*Loop through all 6 neighbours of current pixel */
        for (w=0;w<6;w++) {
            /*Location of neighbour */
            i=x+ne[w]; j=y+ne[w+6]; k=z+ne[w+12];
            IJK_index=mindex3(i, j, k, dims[0], dims[1]);

            /*Check if current neighbour is not yet frozen and inside the */
            /*picture */
            if(isntfrozen3d(i, j, k, dims, Frozen)) {

                Tt=CalculateDistance(T, F[IJK_index], dims, i, j, k, usesecond, usecross, Frozen);
                if(Ed) {
                    Ty=CalculateDistance(Y, 1, dims, i, j, k, usesecond, usecross, Frozen);
                }

                /*Update distance in neigbour list or add to neigbour list */
                IJK_index=mindex3(i, j, k, dims[0], dims[1]);
                if((T[IJK_index]>-1)&&T[IJK_index]<=listprop[0]) {
                    if(neg_listv[(int)T[IJK_index]]>Tt) {
                        listupdate(listval, listprop, (int)T[IJK_index], Tt);
                    }
                }
                else {
                    /*If running out of memory at a new block */
                    if(neg_pos>=neg_free) {
                        neg_free+=100000;
                        neg_listx = (double *)realloc(neg_listx, neg_free*sizeof(double) );
                        neg_listy = (double *)realloc(neg_listy, neg_free*sizeof(double) );
                        neg_listz = (double *)realloc(neg_listz, neg_free*sizeof(double) );
                        if(Ed) {
                            neg_listo = (double *)realloc(neg_listo, neg_free*sizeof(double) );
                        }
                    }
                    list_add(listval, listprop, Tt);
                    neg_listv=listval[listprop[1]-1];
                    neg_listx[neg_pos]=i; neg_listy[neg_pos]=j; neg_listz[neg_pos]=k;
                    if(Ed) {
                        neg_listo[neg_pos]=Ty;
                    }

                    T[IJK_index]=neg_pos;
                    neg_pos++;
                }
            }
        }
        // Exit when nFront frontiers have been reached.
        if (frontCount >= nFront && !reachOrigin) break;
    }
    /* Free memory */
    /* Destroy parameter list */
    destroy_list(listval, listprop);
    free(neg_listx);
    free(neg_listy);
    free(neg_listz);
    delete[] Frozen;
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "msfm3d");
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // Initialize planner object
  ROS_INFO("Initializing msfm3d planner...");
  // Voxel size for Octomap or Voxblox
  float voxel_size;
  n.param("global_planning/resolution", voxel_size, (float)0.1);
  Msfm3d planner(voxel_size);

  // Set vehicle type, map type, and global frame name
  bool ground, esdf_or_octomap;
  std::string global_frame;
  n.param("global_planning/isGroundVehicle", ground, false);
  n.param("global_planning/useOctomap", esdf_or_octomap, true); // Use a TSDF/ESDF message PointCloud2 (0) or Use an octomap message (1)
  n.param<std::string>("global_planning/frame_id", global_frame, "world");
  planner.ground = ground;
  if (planner.ground) ROS_INFO("Vehicle type set to ground vehicle.");
  else ROS_INFO("Vehicle type set to air vehicle");
  planner.esdf_or_octomap = esdf_or_octomap;
  planner.frame = global_frame;

  // Vehicle speed and turnRate
  float speed, turnRate;
  n.param("global_planning/speed", speed, (float)1.0); // m/s
  n.param("global_planning/turnRate", turnRate, (float)5.0); // deg/s
  planner.speed = speed;
  planner.turnRate = turnRate;

  // Replanning ticks
  int replan_tick_limit;
  n.param("global_planning/replan_ticks", replan_tick_limit, 15);

  // Clustering Parameters
  float cluster_radius, min_cluster_size;
  n.param("global_planning/cluster_radius", cluster_radius, (float)(1.5*voxel_size)); // voxels
  n.param("global_planning/min_cluster_size", min_cluster_size, (float)(5.0/voxel_size)); // voxels
  planner.cluster_radius = cluster_radius;
  planner.min_cluster_size = min_cluster_size;

  // Goal Height fixing for air vehicle (For a quad with constrained AGL)
  bool fixGoalHeightAGL;
  float goalHeightAGL;
  int dzFrontierVoxelWidth;
  n.param("global_planning/fixGoalHeightAGL", fixGoalHeightAGL, false);
  n.param("global_planning/goalHeightAGL", goalHeightAGL, (float)0.64);
  n.param("global_planning/dzFrontierVoxelWidth", dzFrontierVoxelWidth, 0);
  planner.fixGoalHeightAGL = fixGoalHeightAGL;
  planner.goalHeightAGL = goalHeightAGL;
  planner.dzFrontierVoxelWidth = dzFrontierVoxelWidth;

  // Origin/Tunnel Entrance
  float origin_x, origin_y, origin_z, entranceRadius;
  n.param("global_planning/entrance_x", origin_x, (float)0.0);
  n.param("global_planning/entrance_y", origin_y, (float)0.0);
  n.param("global_planning/entrance_z", origin_z, (float)0.6);
  n.param("global_planning/entrance_radius", entranceRadius, (float)10.0);
  planner.origin[0] = origin_x;
  planner.origin[1] = origin_y;
  planner.origin[2] = origin_z;
  planner.entranceRadius = entranceRadius;

  // Vehicle camera field of View and max range
  float verticalFoV, horizontalFoV, rMax, rMin;
  n.param("global_planning/cameraVerticalFoV", verticalFoV, (float)30.0);
  n.param("global_planning/cameraHorizontalFoV", horizontalFoV, (float)45.0);
  n.param("global_planning/cameraMaxRange", rMax, (float)3.0);
  n.param("global_planning/cameraMinRange", rMin, (float)0.2);
  planner.camera.verticalFoV = verticalFoV;
  planner.camera.horizontalFoV = horizontalFoV;
  planner.camera.rMax = rMax;
  planner.camera.rMin = rMin;

  // robot2camera quaternion
  float q_w, q_x, q_y, q_z;
  n.param("global_planning/robot2camera_q_w", q_w, (float)1.0);
  n.param("global_planning/robot2camera_q_x", q_x, (float)0.0);
  n.param("global_planning/robot2camera_q_y", q_y, (float)0.0);
  n.param("global_planning/robot2camera_q_z", q_z, (float)0.0);
  planner.robot2camera.q.w = q_w;
  planner.robot2camera.q.x = q_x;
  planner.robot2camera.q.y = q_y;
  planner.robot2camera.q.z = q_z;
  // planner.robot2camera.R = quaternion2RotationMatrix(planner.robot2camera.q);

  // Hard coded pitch down by 15 degrees for the husky
  // if (planner.ground) {
  //   planner.robot2camera.R.setZero();
  //   planner.robot2camera.R(0,0) = std::cos((M_PI/180.0)*-15.0);
  //   planner.robot2camera.R(0,2) = std::sin((M_PI/180.0)*-15.0);
  //   planner.robot2camera.R(2,0) = -std::sin((M_PI/180.0)*-15.0);
  //   planner.robot2camera.R(2,2) = std::cos((M_PI/180.0)*-15.0);
  //   planner.robot2camera.R(1,1) = 1.0;
  // }

  // planner.bubble_radius = 3.0;
  // Set planner bounds so that the robot doesn't exit a defined volume
  bool fenceOn;
  float fence_x_min, fence_x_max, fence_y_min, fence_y_max, fence_z_min, fence_z_max;
  n.param("global_planning/fenceOn", fenceOn, false);
  n.param("global_planning/fence_xmin", fence_x_min, (float)-50.0);
  n.param("global_planning/fence_xmax", fence_x_max, (float)50.0);
  n.param("global_planning/fence_ymin", fence_y_min, (float)-50.0);
  n.param("global_planning/fence_ymax", fence_y_max, (float)50.0);
  n.param("global_planning/fence_zmin", fence_z_min, (float)-50.0);
  n.param("global_planning/fence_zmax", fence_z_max, (float)50.0);
  planner.bounds.set = fenceOn;
  planner.bounds.xmin = fence_x_min;
  planner.bounds.xmax = fence_x_max;
  planner.bounds.ymin = fence_y_min;
  planner.bounds.ymax = fence_y_max;
  planner.bounds.zmin = fence_z_min;
  planner.bounds.zmax = fence_z_max;

  // Get planner operating rate in Hz
  float updateRate;
  n.param("global_planning/updateRate", updateRate, (float)1.0); // Hz

  // Width to inflate obstacles for path planning
  float inflateWidth;
  n.param("global_planning/inflateWidth", inflateWidth, (float)0.6); // meters

  // if (planner.esdf_or_octomap) {
  ROS_INFO("Subscribing to Occupancy Grid...");
  ros::Subscriber sub1 = n.subscribe("/octomap_binary", 1, &Msfm3d::callback_Octomap, &planner);
  // }
  // else {
  // ROS_INFO("Subscribing to ESDF or TSDF PointCloud2...");
  // ros::Subscriber sub1 = n.subscribe("/X1/voxblox_node/tsdf_pointcloud", 1, &Msfm3d::callback, &planner);
  // }
  ROS_INFO("Subscribing to robot state...");
  ros::Subscriber sub2 = n.subscribe("odometry", 1, &Msfm3d::callback_position, &planner);

  ROS_INFO("Subscribing to artifact message...");
  ros::Subscriber sub3 = n.subscribe("artifact_list", 1, &Msfm3d::callback_artifactDetected, &planner);

  ros::Publisher pub1 = n.advertise<geometry_msgs::PointStamped>("nearest_frontier", 5);
  geometry_msgs::PointStamped frontierGoal;
  frontierGoal.header.frame_id = planner.frame;

  // Publish goal point to interface with btraj
  ros::Publisher pub4 = n.advertise<geometry_msgs::PoseStamped>("frontier_goal_pose", 5);
  geometry_msgs::PoseStamped goalPose;
  goalPose.header.frame_id = planner.frame;
  goalPose.pose.position.x = 0.0;
  goalPose.pose.position.y = 0.0;
  goalPose.pose.position.z = 0.0;
  goalPose.pose.orientation.w = 1.0;
  goalPose.header.seq = 1;

  ros::Publisher pub2 = n.advertise<nav_msgs::Path>("planned_path", 5);
  // ros::Publisher pub3 = n.advertise<visualization_msgs::MarkerArray>("/X1/frontier", 100);
  ros::Publisher pub3 = n.advertise<sensor_msgs::PointCloud2>("frontier", 5);
  ros::Publisher pub5 = n.advertise<visualization_msgs::Marker>("goalFrustum", 5);
  visualization_msgs::Marker cameraFrustum;
  cameraFrustum.header.frame_id = "world";
  cameraFrustum.id = 0;
  cameraFrustum.scale.x = 1;
  cameraFrustum.scale.y = 1;
  cameraFrustum.scale.z = 1;
  cameraFrustum.pose.position.x = 0.0;
  cameraFrustum.pose.position.y = 0.0;
  cameraFrustum.pose.position.z = 0.0;
  cameraFrustum.pose.orientation.x = 0.0;
  cameraFrustum.pose.orientation.y = 0.0;
  cameraFrustum.pose.orientation.z = 0.0;
  cameraFrustum.pose.orientation.w = 1.0;
  cameraFrustum.color.a = 0.85;
  cameraFrustum.color.r = 0.0;
  cameraFrustum.color.g = 191.0/255.0;
  cameraFrustum.color.b = 1.0;

  ros::Publisher pub6 = n.advertise<sensor_msgs::PointCloud2>("inflatedOctomap", 5);
  sensor_msgs::PointCloud2 inflatedOccupiedMsg;

  int i = 0;
  bool goalFound = 0;
  ros::Rate r(updateRate); // Hz
  clock_t tStart;
  int npixels;
  int spins = 0;
  int goalViewList[5] = {0, 0, 0, 0, 0};
  double goalViewCost[5];
  float frontierList[15];
  double frontierCost[5];
  float goal[3] = {0.0, 0.0, 0.0};
  int replan_ticks = 0;

  ROS_INFO("Starting planner...");
  r.sleep();
  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
    ROS_INFO("Planner Okay.");
    if (planner.receivedMap && !planner.esdf_or_octomap){
      planner.parsePointCloud();
    }
    // Heartbeat status update
    if (planner.receivedPosition){
      ROS_INFO("Position: [x: %f, y: %f, z: %f]", planner.position[0], planner.position[1], planner.position[2]);
      i = planner.xyz_index3(planner.position);
      ROS_INFO("Index at Position: %d", i);
      if (planner.receivedMap){
        ROS_INFO("ESDF or Occupancy at Position: %f", planner.esdf.data[i]);

        // Find frontier cells and add them to planner.frontier for output to file.
        // Publish frontiers as MarkerArray
        if (updateFrontier(planner)) {
          planner.updateFrontierMsg();
          pub3.publish(planner.frontiermsg);
          ROS_INFO("Frontier published!");

          // Check to make sure that at least 50% of the viewed frontier voxels are still frontiers, if not, resample goal poses.
          int stillFrontierCount = 0;
          bool replan = 0;
          if ((int)planner.goalViews.size() == 0 || goalViewCost[4] < 0.0) {
            replan = 1;
          } else {
            int viewableFrontierCount = (int)planner.goalViews[goalViewList[4]].cloud.size();
            for (int i = 0; i < viewableFrontierCount; i++) {
              pcl::PointXYZ _query = planner.goalViews[goalViewList[4]].cloud.points[i];
              float query[3] = {_query.x, _query.y, _query.z};
              int idx = planner.xyz_index3(query);
              if (planner.frontier[idx]) {
                stillFrontierCount++;
              }
            }
            if (((float)stillFrontierCount)/((float)viewableFrontierCount)<= 0.5) {
              replan = 1;
            }
          }

          // Inflate the obstacle map to avoid collisions
          if (planner.updatedMap && planner.esdf_or_octomap) {
            planner.inflateObstacles(inflateWidth, inflatedOccupiedMsg);
            planner.updatedMap = 0;
          }

          if (planner.esdf_or_octomap) {
            inflatedOccupiedMsg.header.seq = 1;
            inflatedOccupiedMsg.header.frame_id = "world";
            inflatedOccupiedMsg.header.stamp = ros::Time();
            pub6.publish(inflatedOccupiedMsg);
            ROS_INFO("Inflated occupancy grid published!");
          }

          // Call msfm3d function
          if (replan || replan_ticks >= replan_tick_limit) {
            ROS_INFO("At least 50 percent of the frontiers at the goal pose are no longer frontiers or 5 loops have occurred since last plan.  Replanning...");
            // Find goal poses from which to view the frontier
            planner.updateGoalPoses();

            replan_ticks = 0;
            tStart = clock();
            if ((int)planner.frontierCloud->points.size() > 2*(int)planner.frontierClusterIndices[0].indices.size()) {
              ROS_INFO("Reachability matrix calculating to %d closest frontier points...", 2*(int)planner.frontierClusterIndices[0].indices.size());
              reach(planner, 0, 0, 2*(int)planner.frontierClusterIndices[0].indices.size(), false);
            } else {
              ROS_INFO("Reachability matrix calculating to %d closest frontier points...", (int)planner.frontierCloud->points.size());
              reach(planner, 0, 0, (int)planner.frontierCloud->points.size(), false);
            }

            ROS_INFO("Reachability Grid Calculated in: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);

            // Find goal views with an information-based optimization
            // closestGoalView(planner, goalViewList, goalViewCost);
            infoGoalView(planner, goalViewList, goalViewCost);

            // If no reasonable goal views are found, expand search to all frontier cells to see if any goal view can be found
            if (goalViewCost[4] < 0.0) {
              tStart = clock();
              ROS_INFO("Reachability matrix calculating to %d closest frontier points...", (int)planner.frontierCloud->points.size());
              reach(planner, 0, 0, (int)planner.frontierCloud->points.size(), false);
              infoGoalView(planner, goalViewList, goalViewCost);
            }

            if (goalViewCost[4] < 0.0) {
              // The vehicle couldn't find any reasonable frontier views, head to the anchor node.
              ROS_INFO("No reachable frontier views after expanded search, heading to the anchor.");
              tStart = clock();
              ROS_INFO("Replanning to be able to reach the origin...");
              reach(planner, 0, 0, 1, true);
              ROS_INFO("Reachability Grid Calculated in: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);

              frontierGoal.point.x = planner.origin[0];
              frontierGoal.point.y = planner.origin[1];
              frontierGoal.point.z = planner.origin[2];
              goal[0] = planner.origin[0];
              goal[1] = planner.origin[1];
              goal[2] = planner.origin[2];

              // Write a new goal pose for publishing
              goalPose.header.stamp = ros::Time::now();
              goalPose.pose.position.x = goal[0];
              goalPose.pose.position.y = goal[1];
              goalPose.pose.position.z = goal[2];
              goalPose.pose.orientation.x = 0.0;
              goalPose.pose.orientation.y = 0.0;
              goalPose.pose.orientation.z = 0.0;
              goalPose.pose.orientation.w = 1.0;

            } else if (planner.artifactDetected) {
              // Head to the origin if a new artifact is detected.
              ROS_INFO("New artifact detected, heading to the anchor.");
              tStart = clock();
              ROS_INFO("Replanning to be able to reach the origin...");
              reach(planner, 0, 0, 1, true);
              ROS_INFO("Reachability Grid Calculated in: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);

              frontierGoal.point.x = planner.origin[0];
              frontierGoal.point.y = planner.origin[1];
              frontierGoal.point.z = planner.origin[2];
              goal[0] = planner.origin[0];
              goal[1] = planner.origin[1];
              goal[2] = planner.origin[2];

              // Write a new goal pose for publishing
              goalPose.header.stamp = ros::Time::now();
              goalPose.pose.position.x = goal[0];
              goalPose.pose.position.y = goal[1];
              goalPose.pose.position.z = goal[2];
              goalPose.pose.orientation.x = 0.0;
              goalPose.pose.orientation.y = 0.0;
              goalPose.pose.orientation.z = 0.0;
              goalPose.pose.orientation.w = 1.0;
            }
            else {
              // Write a new frontier goal location for publishing
              frontierGoal.point.x = planner.goalViews[goalViewList[4]].pose.position.x;
              frontierGoal.point.y = planner.goalViews[goalViewList[4]].pose.position.y;
              frontierGoal.point.z = planner.goalViews[goalViewList[4]].pose.position.z;
              goal[0] = planner.goalViews[goalViewList[4]].pose.position.x;
              goal[1] = planner.goalViews[goalViewList[4]].pose.position.y;
              goal[2] = planner.goalViews[goalViewList[4]].pose.position.z;

              // Write a new goal pose for publishing
              goalPose.header.stamp = ros::Time::now();
              goalPose.pose.position.x = goal[0];
              goalPose.pose.position.y = goal[1];
              goalPose.pose.position.z = goal[2];
              goalPose.pose.orientation.x = planner.goalViews[goalViewList[4]].pose.q.x;
              goalPose.pose.orientation.y = planner.goalViews[goalViewList[4]].pose.q.y;
              goalPose.pose.orientation.z = planner.goalViews[goalViewList[4]].pose.q.z;
              goalPose.pose.orientation.w = planner.goalViews[goalViewList[4]].pose.q.w;
            }

              // Output status for debugging
              // for (int i=0; i<5; i++) ROS_INFO("Frontier Pose Position: [x: %f, y: %f, z: %f, cost: %f]", planner.goalViews[goalViewList[i]].pose.position.x,
              // planner.goalViews[goalViewList[i]].pose.position.y, planner.goalViews[goalViewList[i]].pose.position.z, goalViewCost[i]);

            for (int i=0; i<5; i++) ROS_INFO("Frontier Pose Position: [x: %f, y: %f, z: %f, utility: %f]", planner.goalViews[goalViewList[i]].pose.position.x,
              planner.goalViews[goalViewList[i]].pose.position.y, planner.goalViews[goalViewList[i]].pose.position.z, goalViewCost[i]);
            // }
            // Find a path to the goal point
            goalFound = planner.updatePath(goal);
          }

          // Publish path, goal point, and goal point only path
          pub1.publish(frontierGoal);
          pub4.publish(goalPose);
          ROS_INFO("Goal point published!");

          // Output and publish path
          pub2.publish(planner.pathmsg);
          ROS_INFO("Path to goal published!");
          goalFound = 0;

          // Publish view frustum
          cameraFrustum.action = 2; // DELETE action
          pub5.publish(cameraFrustum);
          cameraFrustum.action = 0; // ADD action
          if (goalViewCost[4] >= 0.0) {
            view2MarkerMsg(planner.goalViews[goalViewList[4]], planner.camera, cameraFrustum, planner.robot2camera);
            pub5.publish(cameraFrustum);
          }

          // Height debugging
          ROS_INFO("The robot is %0.2f meters off of the ground.  The goal point is %0.2f meters off of the ground.", planner.heightAGL(planner.position), planner.heightAGL(goal));

        } else {

          // Inflate the obstacle map to avoid collisions
          if (planner.updatedMap) {
            planner.inflateObstacles(inflateWidth, inflatedOccupiedMsg);
            planner.updatedMap = 0;
          }

          if (planner.esdf_or_octomap) {
            inflatedOccupiedMsg.header.seq = 1;
            inflatedOccupiedMsg.header.frame_id = "world";
            inflatedOccupiedMsg.header.stamp = ros::Time();
            pub6.publish(inflatedOccupiedMsg);
            ROS_INFO("Inflated occupancy grid published!");
          }

          ROS_INFO("No frontiers after filtering, robot is heading to the anchor.");
          tStart = clock();
          ROS_INFO("Replanning to be able to reach the origin...");
          reach(planner, 0, 0, 1, true);
          ROS_INFO("Reachability Grid Calculated in: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);

          frontierGoal.point.x = planner.origin[0];
          frontierGoal.point.y = planner.origin[1];
          frontierGoal.point.z = planner.origin[2];
          goal[0] = planner.origin[0];
          goal[1] = planner.origin[1];
          goal[2] = planner.origin[2];

          // Write a new goal pose for publishing
          goalPose.header.stamp = ros::Time::now();
          goalPose.pose.position.x = goal[0];
          goalPose.pose.position.y = goal[1];
          goalPose.pose.position.z = goal[2];
          goalPose.pose.orientation.x = 0.0;
          goalPose.pose.orientation.y = 0.0;
          goalPose.pose.orientation.z = 0.0;
          goalPose.pose.orientation.w = 1.0;

          // Find a path to the goal point
          goalFound = planner.updatePath(goal);

           // Publish path, goal point, and goal point only path
          pub1.publish(frontierGoal);
          pub4.publish(goalPose);
          ROS_INFO("Goal point published!");

          // Output and publish path
          pub2.publish(planner.pathmsg);
          ROS_INFO("Path to goal published!");
          goalFound = 0;

          // Height debugging
          // ROS_INFO("The robot is %0.2f meters off of the ground.  The goal point is %0.2f meters off of the ground.", planner.heightAGL(planner.position), planner.heightAGL(goal));
        }
        // }
      }
    }
    replan_ticks++;
  }

  return 0;
}
