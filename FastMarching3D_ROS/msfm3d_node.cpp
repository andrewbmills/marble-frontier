// g++ msfm3d_node.cpp -g -o msfm3d_node.o -I /opt/ros/melodic/include -I /usr/include/c++/7.3.0 -I /home/andrew/catkin_ws/devel/include -I /home/andrew/catkin_ws/src/octomap_msgs/include -I /usr/include/pcl-1.8 -I /usr/include/eigen3 -L /usr/lib/x86_64-linux-gnu -L /home/andrew/catkin_ws/devel/lib -L /opt/ros/melodic/lib -Wl,-rpath,opt/ros/melodic/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization -loctomap -lboost_system -lpcl_common -lpcl_io -lpcl_filters -lpcl_features -lpcl_kdtree -lpcl_segmentation

#include <math.h>
#include <numeric>
#include <algorithm>
#include <random>
// ROS libraries
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <msfm3d/Goal.h>
#include <msfm3d/GoalArray.h>
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
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
//pcl ROS
#include <pcl_conversions/pcl_conversions.h>
// Custom libraries
#include "msfm3d.c"
#include "bresenham3d.cpp"

// Sleep library
#include <unistd.h>

template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> &v) {

  // initialize original index locations
  std::vector<size_t> idx(v.size());
  std::iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  std::sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

  return idx;
}

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
struct Node
{
  int id = -1;
  int parent = -1;
  float g = 1e10;
  float h;
  float f = 1e10;
  std::vector<int> neighbors;
  float position[3];
};

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
    float turnPenalty = 0.2; // Weight to place on an initial heading error

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
    float inflateWidth = 0.0;
    int minViewCloudSize = 10;

    float viewPoseObstacleDistance = 0.001; // view pose minimum distance from obstacles

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
      float normalThresholdZ;
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

    // Number of neighbors to plan with
    int numNeighbors = 0;

    // Multi-agent selected goal_id from candidate goal set
    int goalId = 0;
    bool goalSelected = false;

    // Sensor parameters
    SensorFoV camera;
    Pose robot2camera;

    Quaternion q; // robot orientation in quaternions
    ESDF esdf; // ESDF struct object
    Boundary bounds; // xyz boundary of possible goal locations

    std::string task = "explore";
    float customGoal[3] = {0.0, 0.0, 0.0};

    void callback(sensor_msgs::PointCloud2 msg); // Subscriber callback function for PC2 msg (ESDF)
    void callback_Octomap(const octomap_msgs::Octomap::ConstPtr msg); // Subscriber callback function for Octomap msg
    void callback_Octomap_freePCL(const sensor_msgs::PointCloud2 msg);
    void callback_Octomap_occupiedPCL(const sensor_msgs::PointCloud2 msg);
    void callback_position(const nav_msgs::Odometry msg); // Subscriber callback for robot position
    void callback_artifactDetected(const std_msgs::Bool msg);
    void callback_numNeighbors(const std_msgs::Int8 msg);
    void callback_task(const std_msgs::String msg);
    void callback_customGoal(const geometry_msgs::PoseStamped msg);
    // void callback_artifactDetected(const marble_common::ArtifactArray msg);
    void parsePointCloud(); // Function to parse pointCloud2 into an esdf format that msfm3d can use
    int xyz_index3(const float point[3]);
    void index3_xyz(const int index, float point[3]);
    void getEuler(); // Updates euler array given the current quaternion values
    bool updatePath(const float goal[3]); // Updates the path vector from the goal frontier point to the robot location
    bool updatePathOneVoxel(const float goal[3]);
    std::vector<int> getNeighbors(int id);
    void updateFrontierMsg(); // Updates the frontiermsg MarkerArray with the frontier matrix for publishing
    bool clusterFrontier(const bool print2File); // Clusters the frontier pointCloud with euclidean distance within a radius
    bool normalFrontierFilter(); // Filters frontiers based on they're local normal value
    bool inBoundary(const float point[3]); // Checks if a point is inside the planner boundaries
    // bool collisionCheck(const float point[3]); // Checks if the robot being at the current point (given vehicleVolume) intersects with the obstacle environment (esdf)
    void greedyGrouping(const float r, const bool print2File);
    bool raycast(const pcl::PointXYZ start, const pcl::PointXYZ end);
    Pose samplePose(const pcl::PointXYZ centroid, const SensorFoV camera, const int sampleLimit);
    void updateGoalPoses();
    void inflateObstacles(const float radius, sensor_msgs::PointCloud2& inflatedOccupiedMsg);
    float heightAGL(const float point[3]);
};

void Msfm3d::callback_customGoal(const geometry_msgs::PoseStamped msg)
{
  customGoal[0] = msg.pose.position.x;
  customGoal[1] = msg.pose.position.y;
  customGoal[2] = msg.pose.position.z;
  return;
}

void Msfm3d::callback_task(const std_msgs::String msg)
{
  task = msg.data;
  return;
}

void Msfm3d::callback_numNeighbors(const std_msgs::Int8 msg)
{
  numNeighbors = (int)msg.data;
  return;
}

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
      if (!esdf.seen[query_idx] || (esdf.data[query_idx] <= 0.001)) { // If the query location is unseen or occupied, then it is the bottom of the local map
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr frontierCloudPreFilter(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::copyPointCloud(*frontierCloud, *frontierCloudPreFilter);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  kdtree->setInputCloud(frontierCloud);

  // Clear previous frontierClusterIndices
  frontierClusterIndices.clear();

  // Initialize euclidean cluster extraction object
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_radius); // Clusters must be made of contiguous sections of frontier (within sqrt(2)*voxel_size of each other)
  ec.setMinClusterSize(roundf(min_cluster_size)); // Cluster must be at least 15 voxels in size
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
  frontierCloud->clear();
  idx = 0;
  for (int i=0; i<frontierClusterIndices.size(); i++) {
    for (int j=0; j<frontierClusterIndices[i].indices.size(); j++) {
      frontierCloud->points.push_back(frontierCloudPreFilter->points[frontierClusterIndices[i].indices[j]]);
      frontierClusterIndices[i].indices[j] = idx;
      idx++;
    }
  }
  ROS_INFO("Frontier cloud after clustering has %d points.", (int)frontierCloud->points.size());

  if ((int)frontierCloud->points.size() < 1) {
    return 0;
  } else {
    // Get new indices of Frontier Clusters after filtering (extract filter does not preserve indices);
    // frontierClusterIndices.clear();
    // kdtree->setInputCloud(frontierCloud);
    // ec.setSearchMethod(kdtree);
    // ec.setInputCloud(frontierCloud);
    // ec.extract(frontierClusterIndices);
    return 1;
  }
}

bool Msfm3d::normalFrontierFilter()
{
  // Create cloud pointer to store the removed points
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr normalCloud(new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr frontierCloudPreFilter(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*frontierCloud, *frontierCloudPreFilter);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  kdtree->setInputCloud(frontierCloud);

  // Initialize euclidean cluster extraction object
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointXYZINormal> ne;
  ne.setSearchMethod(kdtree);
  ne.setInputCloud(frontierCloud);
  ne.setRadiusSearch(2.1*voxel_size);
  ne.compute(*normalCloud);

  float query[3];
  int idx;
  frontierCloud->clear();
  for (int i=0; i<normalCloud->points.size(); i++) {
    float query[3] = {frontierCloudPreFilter->points[i].x, frontierCloudPreFilter->points[i].y, frontierCloudPreFilter->points[i].z};
    idx = xyz_index3(query);
    if (std::abs(normalCloud->points[i].normal_z) >= normalThresholdZ) {
      frontier[idx] = 0;
    } else {
      frontierCloud->points.push_back(frontierCloudPreFilter->points[i]);
    }
  }

  // Filter frontierCloud to keep only the inliers
  ROS_INFO("Frontier cloud after normal filtering has %d points.", (int)frontierCloud->points.size());

  if ((int)frontierCloud->points.size() < 1) {
    return 0;
  } else {
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

bool Msfm3d::raycast(const pcl::PointXYZ start, const pcl::PointXYZ end)
{
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
    // ROS_INFO("Casting ray from (%0.1f, %0.1f, %0.1f) to (%0.1f, %0.1f, %0.1f)", start.x, start.y, start.z, end.x, end.y, end.z);
    int id_start[3];
    int id_end[3];
    id_start[0] = roundf((start.x - esdf.min[0])/voxel_size);
    id_start[1] = roundf((start.y - esdf.min[1])/voxel_size);
    id_start[2] = roundf((start.z - esdf.min[2])/voxel_size);
    id_end[0] = roundf((end.x - esdf.min[0])/voxel_size);
    id_end[1] = roundf((end.y - esdf.min[1])/voxel_size);
    id_end[2] = roundf((end.z - esdf.min[2])/voxel_size);
    // ROS_INFO("Converted to ids (%d, %d, %d) to (%d, %d, %d)", id_start[0], id_start[1], id_start[2], id_end[0], id_end[1], id_end[2]);
    // Run the bresenham3d line tracing algorithm to find all the indices in between start and end
    // std::vector<int> voxels = Bresenham3D(1, 1, 1, 6, 1, 1);
    std::vector<int> voxels = Bresenham3D(id_start[0], id_start[1], id_start[2], id_end[0], id_end[1], id_end[2]);
    // ROS_INFO("A %d voxel long ray cast through the following voxels:", voxels.size()/3);
    for (int i=0; i<voxels.size(); i+=3) {
      // ROS_INFO("(%d, %d, %d)", voxels[i], voxels[i+1], voxels[i+2]);
      float query[3];
      query[0] = voxels[i]*voxel_size + esdf.min[0];
      query[1] = voxels[i+1]*voxel_size + esdf.min[1];
      query[2] = voxels[i+2]*voxel_size + esdf.min[2];
      // ROS_INFO("Voxel with ids (%d, %d, %d)", voxels[i], voxels[i+1], voxels[i+2]);
      // ROS_INFO("Querying (%0.1f, %0.1f, %0.1f)", query[0], query[1], query[2]);
      int idx = xyz_index3(query);
      if ((idx >= 0) || (idx < (esdf.size[0]*esdf.size[1]*esdf.size[2]))) {
        if (esdf.data[idx] <= 0.005) {
          return false;
        }
      } else {
        return false;
      }
    }
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

  int pass1 = 0;
  int pass2 = 0;
  int pass3 = 0;
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
      pass1++;
      continue;
    }

    // ROS_INFO("Checking if sampled point is reachable...");
    // See if sample is at an unreachable point (occupied or unseen)
    // if ((esdf.data[sample_idx] < viewPoseObstacleDistance && (i < 0.4*sampleLimit)) || esdf.data[sample_idx] <= 0.0) {
    if (esdf.data[sample_idx] < viewPoseObstacleDistance) {
      pass2++;
      continue;
    }

    // If the vehicle is a ground vehicle, move the sampled point vertically until it's wheel_bottom_dist off the ground
    // if (ground || fixGoalHeightAGL) {
    //   // ROS_INFO("Sample point is at height %0.2f.", sample.z);
    //   float height = heightAGL(sample_query);
    //   if (!std::isnan(height)) {
    //     sample.z = sample.z - (height - goalHeightAGL);
    //   } else {
    //     continue;
    //   }

    //   // Check to make sure the centroid is within r_min to r_max
    //   float r_new = std::sqrt((sample.x - centroid.x)*(sample.x - centroid.x) + (sample.y - centroid.y)*(sample.y - centroid.y) + (sample.z - centroid.z)*(sample.z - centroid.z));
    //   // ROS_INFO("Sample point is now at height %0.2f.  This point is %0.2f meters from the sample source.", sample.z, r_new);
    //   if (r_new < camera.rMin || r_new > camera.rMax) {
    //     continue;
    //   }

    //   // Check to make sure the centroid is still within the verticalFoV
    //   if ((std::abs(sample.z - centroid.z)/r_new) > std::sin((M_PI/180.0)*camera.verticalFoV/2.0)) {
    //     continue;
    //   }
    //   // ROS_INFO("Sample point is still within the verticalFoV.");
    // }

    // ROS_INFO("Checking if the point is occluded from viewing the centroid...");
    // Check for occlusion
    if (!raycast(sample, centroid)) {
      pass3++;
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
  // ROS_INFO("%d poses sampled outside of map.  %d poses sampled too close to walls or in the air.  %d poses were occluded.", pass1, pass2, pass3);
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
    // ROS_INFO("Sampling an admissable pose that sees the group centroid at (%0.1f, %0.1f, %0.1f)", greedyCenters.points[it].x, greedyCenters.points[it].y, greedyCenters.points[it].z);
    Pose goalPose = samplePose(greedyCenters.points[it], camera, 400);

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
    // TO-DO: Use a series of passthrough filters to restrict the x,y,z neighborhood (cropbox) and then compute
    // polar coords for each point to filter those within the sensor FoV.

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

    // ROS_INFO("Sampled view from [%0.2f, %0.2f, %0.2f] sees %d frontier voxels.", sampleView.pose.position.x, sampleView.pose.position.y, sampleView.pose.position.z, (int)sampleView.cloud.points.size());
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
  position[2] = msg.pose.pose.position.z + 0.4;
  q.x = msg.pose.pose.orientation.x;
  q.y = msg.pose.pose.orientation.y;
  q.z = msg.pose.pose.orientation.z;
  q.w = msg.pose.pose.orientation.w;
  ROS_INFO("Robot pose updated!");
}

void Msfm3d::callback_Octomap(const octomap_msgs::Octomap::ConstPtr msg)
{
  ROS_INFO("Getting OctoMap message...");


  // Free/Allocate the tree memory
  // ROS_INFO("Converting Octomap msg to AbstractOcTree...");
  // octomap::AbstractOcTree* abstree = new octomap::AbstractOcTree(msg->resolution);
  // abstree = octomap_msgs::binaryMsgToMap(*msg); // OcTree object for storing Octomap data.
  // ROS_INFO("Octomap converted to AbstractOcTree.");

  // Check if the message is empty (skip callback if it is)
  if (msg->data.size() == 0) {
    ROS_INFO("Octomap message is of length 0");
    return;
  }

  if (!receivedMap) receivedMap = 1;
  if (!updatedMap) updatedMap = 1;

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

  // Initialize the voxels within a vehicle volume around the robot the free.
  if (receivedPosition) {
    float query_point[3];
    int query_idx;
    ROS_INFO("Clearing vehicle volume with limits [%0.2f, %0.2f; %0.2f, %0.2f; %0.2f, %0.2f].", vehicleVolume.xmin,
      vehicleVolume.xmax, vehicleVolume.ymin, vehicleVolume.ymax, vehicleVolume.zmin, vehicleVolume.zmax);
    if (vehicleVolume.set) {
      for(float dx = vehicleVolume.xmin; dx <= vehicleVolume.xmax; dx = dx + voxel_size) {
        query_point[0] = position[0] + dx;
        for(float dy = vehicleVolume.ymin; dy <= vehicleVolume.ymax; dy = dy + voxel_size) {
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


  ROS_INFO("Octomap message received.  %d leaves labeled as occupied.  %d leaves labeled as free.", occCount, freeCount);
}

void Msfm3d::callback(sensor_msgs::PointCloud2 msg)
{
  ROS_INFO("Getting ESDF PointCloud2...");
  if (msg.data.size() == 0) {
    ROS_INFO("Input PointCloud2 message is empty.");
    return;
  }
  if (!receivedMap) receivedMap = 1;
  if (!updatedMap) updatedMap = 1;
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
    if (xyzis[i+3] > 0.0) {
      // esdf.data[index] = (double)(5.0*(tanh(xyzis[i+3] - exp(1.0)) + 1.0)/2.0); // doesn't work too well (numerical issues)
      esdf.data[index] = (double)xyzis[i+3];
    }
    else {
      esdf.data[index] = (double)(0.0);
    }
    esdf.seen[index] = (xyzis[i+4]>0.0);
  }

  if (receivedPosition) {
    float query_point[3];
    int query_idx;
    ROS_INFO("Clearing vehicle volume with limits [%0.2f, %0.2f; %0.2f, %0.2f; %0.2f, %0.2f].", vehicleVolume.xmin,
      vehicleVolume.xmax, vehicleVolume.ymin, vehicleVolume.ymax, vehicleVolume.zmin, vehicleVolume.zmax);
    if (vehicleVolume.set) {
      for(float dx = vehicleVolume.xmin; dx <= vehicleVolume.xmax; dx = dx + voxel_size) {
        query_point[0] = position[0] + dx;
        for(float dy = vehicleVolume.ymin; dy <= vehicleVolume.ymax; dy = dy + voxel_size) {
          query_point[1] = position[1] + dy;
          for(float dz = vehicleVolume.zmin; dz <= vehicleVolume.zmax; dz = dz + voxel_size) {
            query_point[2] = position[2] + dz;
            query_idx = xyz_index3(query_point);
            if ((query_idx >= 0) && (query_idx < esdf.size[0]*esdf.size[1]*esdf.size[2])) { // Check for valid array indices
              if (esdf.data[query_idx] < 1.0) esdf.data[query_idx] = 1.0 + inflateWidth;
              esdf.seen[query_idx] = true;
            }
          }
        }
      }
    }
  }

  // Free xyzis holder array memory
  delete[] xyzis;
}

void Msfm3d::inflateObstacles(const float radius, sensor_msgs::PointCloud2& inflatedOccupiedMsg)
{
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
      float newDistance = esdf.data[i] - (double)radius;
      if (newDistance < 0.0) {
        esdf.data[i] = 0.0;
        pointsInflated++;
      }
      else {
        esdf.data[i] = newDistance;
      } // Just subtract the value for an esdf since the distance is included.
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

std::vector<int> Msfm3d::getNeighbors(int id)
{
  std::vector<int> neighbors;
  float query[3], neighbor_position[3];
  index3_xyz(id, query);
  for (int i=0; i<3; i++) {
    neighbor_position[0] = query[0] + (float)(i-1)*voxel_size;
    for (int j=0; j<3; j++) {
      neighbor_position[1] = query[1] + (float)(j-1)*voxel_size;
      for (int k=0; k<3; k++) {
        neighbor_position[2] = query[2] + (float)(k-1)*voxel_size;
        int neighbor_id = xyz_index3(neighbor_position);
        if (neighbor_id == id) continue;
        if ((neighbor_id >= 0) && (neighbor_id <= esdf.size[0]*esdf.size[1]*esdf.size[2])) {
          if ((reach[neighbor_id] > 0.01) && (reach[neighbor_id] <= 1e8)) {
            if (esdf.data[neighbor_id] >= 1.5*voxel_size) neighbors.push_back(neighbor_id);
          }
        }
      }
    }
  }
  return neighbors;
}

std::vector<float> reconstructPath(std::vector<Node> visited, Node end) {
  std::vector<float> path;
  Node current = end;
  for (int i=0; i<3; i++) path.push_back(current.position[i]);
  while (current.parent != -1) {
    current = visited[current.parent];
    for (int i=0; i<3; i++) path.push_back(current.position[i]);
  }
  return path;
}

bool Msfm3d::updatePath(const float goal[3])
{
  int npixels = esdf.size[0]*esdf.size[1]*esdf.size[2]; // size of the reachability array, index in reachability array of the current path voxel.
  int goal_idx = xyz_index3(goal);
  if (goal_idx < 0 || goal_idx > npixels){
    ROS_INFO("Goal point is not reachable.");
    return false;
  }
  if (reach[goal_idx] <= 0.0 || reach[goal_idx] >= 1e12) {
    ROS_INFO("Goal point is either too far away or is blocked by an obstacle.");
    return false;
  }
  ROS_INFO("Attempting to find path to [%0.1f, %0.1f, %0.1f] from [%0.1f, %0.1f, %0.1f].", goal[0], goal[1], goal[2], position[0], position[1], position[2]);
  std::vector<int> node_id_list(npixels, -1);
  std::vector<Node> visited;

  // Initialize a star from goal position back to start.
  Node start;
  start.id = goal_idx;
  start.g = 0.0;
  start.h = reach[goal_idx];
  // start.h = dist3(goal, position)/esdf.data[goal_idx];
  start.f = start.g + start.h;
  start.neighbors = getNeighbors(start.id);
  start.parent = -1;
  index3_xyz(goal_idx, start.position);
  
  visited.push_back(start);
  node_id_list[start.id] = visited.size()-1;

  std::vector<Node> open_set; // priority queue of next nodes
  open_set.push_back(start);
  int itt = 0;
  while (open_set.size()>0) {
    itt++;
    Node current = open_set.back();
    // ROS_INFO("Current node, [%0.1f, %0.1f, %0.1f], g = %0.2f, h = %0.2f, f = %0.2f, has %d neighbors", current.position[0], current.position[1], current.position[2], current.g, current.h, current.f, current.neighbors.size());
    // if (current.id == xyz_index3(position)) {
    if (dist2(current.position, position) <= 3*voxel_size) {
      std::vector<float> path = reconstructPath(visited, current); // Stop if the robot's current position (the goal) has been reached.
      nav_msgs::Path newpathmsg;
      newpathmsg.header.frame_id = frame;
      geometry_msgs::PoseStamped pose;
      for (int i=0; i<(path.size()-2); i=i+3){
        pose.header.frame_id = frame;
        pose.pose.position.x = path[i]; pose.pose.position.y = path[i+1]; pose.pose.position.z = path[i+2];
        newpathmsg.poses.push_back(pose);
      }
      pathmsg = newpathmsg;
      ROS_INFO("Path found of length %d.", newpathmsg.poses.size());
      return true;
    }
    open_set.pop_back();
    for (int i=0; i<current.neighbors.size(); i++) {
      int neighbor_id = current.neighbors[i];
      bool neighbor_is_new = false;
      if (node_id_list[neighbor_id] == -1) {
        neighbor_is_new = true;
        Node neighbor;
        neighbor.id = neighbor_id; 
        index3_xyz(neighbor.id, neighbor.position);
        neighbor.h = reach[neighbor.id];
        // neighbor.h = dist3(neighbor.position, position)/esdf.data[neighbor_id];
        neighbor.neighbors = getNeighbors(neighbor.id);
        node_id_list[neighbor.id] = visited.size();
        visited.push_back(neighbor);
      }
      int visit_id = node_id_list[neighbor_id];
      float tentative_g = current.g + dist3(current.position, visited[visit_id].position)/(esdf.data[neighbor_id]*voxel_size);
      if (visited[visit_id].g > tentative_g) {
        // Change the neighbors parent to the current
        visited[visit_id].parent = node_id_list[current.id];
        visited[visit_id].g = tentative_g;
        visited[visit_id].f = tentative_g + visited[visit_id].h;
        if (neighbor_is_new) {
          int j;
          for (j=0; j<open_set.size(); j++) {
            if (open_set[j].f < visited[visit_id].f) break;
          }
          open_set.insert(open_set.begin() + j, visited[visit_id]);
        }
      }
    }
  }
  ROS_INFO("Not able to find the start position after running path updater for %d iterations.", itt);
  return false;
}

bool Msfm3d::updatePathOneVoxel(const float goal[3])
{
  int npixels = esdf.size[0]*esdf.size[1]*esdf.size[2], idx; // size of the reachability array, index in reachability array of the current path voxel.
  int neighbor[6]; // neighbor voxels to the current voxel
  float point[3]; // current point x,y,z coordinates
  float query[3]; // intermediate x,y,z coordinates for another operation
  float grad[3]; // gradient at the current point
  float grad_norm = 1.0; // norm of the gradient vector
  float dist_robot2path = dist3(position, goal); // distance of the robot to the path
  float position2D[2];
  float point2D[2];
  float reachValue;
  std::vector<float> path;

  float sobelKernel_x[27] = {1.0, 0.0, -1.0, 2.0, 0.0, -2.0, 1.0, 0.0, -1.0,
                            2.0, 0.0, -2.0, 4.0, 0.0, -4.0, 2.0, 0.0, -2.0,
                            1.0, 0.0, -1.0, 2.0, 0.0, -2.0, 1.0, 0.0, -1.0};
  float sobelKernel_y[27] = {1.0, 2.0, 1.0, 0.0, 0.0, 0.0, -1.0, -2.0, -1.0,
                            2.0, 4.0, 2.0, 0.0, 0.0, 0.0, -2.0, -4.0, -2.0,
                            1.0, 2.0, 1.0, 0.0, 0.0, 0.0, -1.0, -2.0, -1.0};
  float sobelKernel_z[27] = {1.0, 2.0, 1.0, 2.0, 4.0, 2.0, 1.0, 2.0, 1.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            -1.0, -2.0, -1.0, -2.0, -4.0, -2.0, -1.0, -2.0, -1.0};

  // If the goal point isn't in the reachable map, return false
  int goal_idx = xyz_index3(goal);
  if (goal_idx < 0 || goal_idx > npixels){
    ROS_INFO("Goal point is not reachable.");
    return false;
  }
  if (reach[goal_idx] <= 0.0 || reach[goal_idx] >= 1e12) {
    ROS_INFO("Goal point is either too far away or is blocked by an obstacle.");
    return false;
  }

  // 3D Interpolation intermediate values from https://en.wikipedia.org/wiki/Trilinear_interpolation
  float step = 0.5*voxel_size;

  // Path message for ROS
  nav_msgs::Path newpathmsg;
  geometry_msgs::PoseStamped pose;
  newpathmsg.header.frame_id = frame;

  // Clear the path vector to prep for adding new elements.
  for (int i=0; i<3; i++){
    point[i] = goal[i];
    path.push_back(point[i]);
  }

  float maxReach = reach[goal_idx] + 2.0;
  // ROS_INFO("Goal is (%0.1f, %0.1f, %0.1f), robot is at (%0.1f, %0.1f, %0.1f)", goal[0], goal[1], goal[2], position[0], position[1], position[2]);
  // Run loop until the path is within a voxel of the robot.
  while ((dist_robot2path > 3.0*voxel_size) && (path.size() < 9000) && grad_norm >= 0.001) {
    // Find the 26 neighbor indices

  	// Find the current point's grid indices and it's 6 neighbor voxel indices.
    // ROS_INFO("Current point is (%0.1f, %0.1f, %0.1f)", point[0], point[1], point[2]);
  	idx = xyz_index3(point);
    if ((idx < 0) || (idx >= esdf.size[0]*esdf.size[1]*esdf.size[2])) break;
    float neighbor[3] = {0.0, 0.0, 0.0};
    for (int i=0; i<3; i++) grad[i] = 0.0;
    for (int i=0; i<3; i++) {
      neighbor[0] = point[0] + (float)(i-1)*voxel_size;
      for (int j=0; j<3; j++) {
        neighbor[1] = point[1] + (float)(j-1)*voxel_size;
        for (int k=0; k<3; k++) {
          neighbor[2] = point[2] + (float)(k-1)*voxel_size;
          int voxel_id = xyz_index3(neighbor);
          int kernel_id = i + 3*j + 9*k;
          // ROS_INFO("Neighbor %d (%d, %d, %d) is (%0.1f, %0.1f, %0.1f) with index %d", kernel_id, i-1, j-1, k-1, neighbor[0], neighbor[1], neighbor[2], voxel_id);
          if ((voxel_id < 0) || (voxel_id >= esdf.size[0]*esdf.size[1]*esdf.size[2])) {
            reachValue = reach[idx];
          } else {
            if (reach[voxel_id] <= 0.0) {
              reachValue = reach[idx];
            } else {
              reachValue = reach[voxel_id];
            }
          }
          // ROS_INFO("Reach value = %0.2f", reachValue);
          grad[0] += sobelKernel_x[kernel_id]*reachValue;
          grad[1] += sobelKernel_y[kernel_id]*reachValue;
          grad[2] += sobelKernel_z[kernel_id]*reachValue;
        }
      }
    }

    // neighbor[0] = idx - 1; // i-1
    // neighbor[1] = idx + 1; // i+1
    // neighbor[2] = idx - esdf.size[0]; // j-1
    // neighbor[3] = idx + esdf.size[0]; // j+1
    // neighbor[4] = idx - esdf.size[0]*esdf.size[1]; // k-1
    // neighbor[5] = idx + esdf.size[0]*esdf.size[1]; // k+1
    // // ROS_INFO("Neighbor indices found of corner point %d", i);
    // for (int j=0; j<3; j++) {
    //   grad[j] = 0; // Initialize to zero in case neither neighbor has been assigned a reachability value.
    //   // Check if the neighbor voxel has no reachability matrix value
    //   if (reach[neighbor[2*j]] > 0.0 && reach[neighbor[2*j+1]] > 0.0){
    //     grad[j] = 0.5*(float)(reach[neighbor[2*j]] - reach[neighbor[2*j+1]]); // Central Difference Operator (Try Sobel if this is bad)
    //     // ROS_INFO("Using Central Difference Operator for %d coordinate for path gradient.", j);
    //   } else {
    //     if (reach[neighbor[2*j]] > 0.0) {
    //       grad[j] = 0.5*(float)(reach[neighbor[2*j]] - reach[idx]); // Intermediate Difference Operator (with previous neighbor)
    //       // ROS_INFO("Using Backward Intermediate Difference Operator for %d coordinate for path gradient.", j);
    //     }
    //     if (reach[neighbor[2*j + 1]] > 0.0) {
    //       grad[j] = 0.5*(float)(reach[idx] - reach[neighbor[2*j+1]]); // Intermediate Difference Operator (with next neighbor)
    //       // ROS_INFO("Using Forward Intermediate Difference Operator for %d coordinate for path gradient.", j);
    //     }
    //   }
    // }

    // Normalize the size of the gradient vector if it is too large
    grad_norm = std::sqrt(grad[0]*grad[0] + grad[1]*grad[1] + grad[2]*grad[2]);
    if (grad_norm > 1.0){
      for (int i=0; i<3; i++) grad[i] = std::sqrt(1.0)*grad[i]/grad_norm;
    }
    if (grad_norm < 0.05){
      for (int i=0; i<3; i++) grad[i] = std::sqrt(0.05)*grad[i]/grad_norm;
    }

    // ROS_INFO("3D Interpolation performed.");
    // if (path.size() < 18) ROS_INFO("Gradients: [%f, %f, %f]", grad[0], grad[1], grad[2]);
    // Update point and add to path
    // for (int i=0; i<3; i++) grad[i] = grad[i]/grad_norm;
    for (int i=0; i<3; i++) {
      point[i] = point[i] + step*grad[i];
    }

    // Check for NaNs and Infs
    if (std::isnan(point[0]) || std::isnan(point[1]) || std::isnan(point[2])) return false;
    if (std::isinf(point[0]) || std::isinf(point[1]) || std::isinf(point[2])) return false;

    for (int i=0; i<3; i++) {
      path.push_back(point[i]);
    }



    // Steps should be at the voxel level only
    // int idx_last = idx;
    // idx = xyz_index3(point);
    // if (idx == idx_last) break;
    // if ((idx >= 0) && (idx < esdf.size[0]*esdf.size[1]*esdf.size[2])) {
    //   index3_xyz(idx, point);
    //   for (int i=0; i<3; i++) path.push_back(point[i]);
    // } else {
    //   break;
    // }

    // if (path.size() < 18) ROS_INFO("[%f, %f, %f] added to path.", point[0], point[1], point[2]);

    // Update the robot's distance to the path
    // if (ground){
    //   position2D[0] = position[0];
    //   position2D[1] = position[1];
    //   point2D[0] = point[0];
    //   point2D[1] = point[1];
    //   dist_robot2path = dist2(position2D, point2D);
    // }
    // else {
    dist_robot2path = dist3(position, point);
    // }
  }

  // Check if the path made it back to the vehicle
  if (dist_robot2path > 3.0*voxel_size) {
    // ROS_INFO("Path did not make it back to the robot.  Select a different goal point.");
    return false;
  }

  // Add path vector to path message for plotting in rviz
  // ROS_INFO("Path finished of length %d", (int)(path.size()/3.0));
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

void updateFrontierClusterGroupMsgs(Msfm3d &planner, visualization_msgs::MarkerArray &cluster_marker_msg, visualization_msgs::MarkerArray &group_marker_msg)
{
  cluster_marker_msg.markers.clear();
  std::vector<std_msgs::ColorRGBA> color_table;
  std_msgs::ColorRGBA red; red.r = 1.0; red.a = 1.0;
  std_msgs::ColorRGBA lime; lime.g = 1.0; lime.a = 1.0;
  std_msgs::ColorRGBA blue; blue.b = 1.0; blue.a = 1.0;
  std_msgs::ColorRGBA yellow; yellow.r = 1.0; yellow.g = 1.0; yellow.a = 1.0;
  std_msgs::ColorRGBA cyan; cyan.g = 1.0; cyan.b = 1.0; cyan.a = 1.0;
  std_msgs::ColorRGBA magenta; magenta.r = 1.0; magenta.b = 1.0; magenta.a = 1.0;
  std_msgs::ColorRGBA silver; silver.r = 0.75; silver.g = 0.75; silver.b = 0.75; silver.a = 1.0;
  std_msgs::ColorRGBA maroon; maroon.r = 0.5; maroon.a = 1.0;
  std_msgs::ColorRGBA olive; olive.r = 0.5; olive.g = 0.5; olive.a = 1.0;
  std_msgs::ColorRGBA green; green.g = 0.5; green.a = 1.0;
  std_msgs::ColorRGBA purple; purple.r = 0.5; purple.b = 0.5; purple.a = 1.0;
  std_msgs::ColorRGBA teal; teal.g = 0.5; teal.b = 0.5; teal.a = 1.0;
  std_msgs::ColorRGBA navy; navy.b = 0.5; navy.a = 1.0;
  color_table.push_back(red);
  color_table.push_back(lime);
  color_table.push_back(blue);
  color_table.push_back(yellow);
  color_table.push_back(cyan);
  color_table.push_back(magenta);
  color_table.push_back(silver);
  color_table.push_back(maroon);
  color_table.push_back(olive);
  color_table.push_back(green);
  color_table.push_back(purple);
  color_table.push_back(teal);
  color_table.push_back(navy);

  for (int i=0; i<planner.frontierClusterIndices.size(); i++) {
    // Preliminaries
    visualization_msgs::Marker msg;
    msg.header.frame_id = planner.frame;
    msg.header.stamp = ros::Time();
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.orientation.w = 1.0;
    msg.id = i;
    // msg.type = visualization_msgs::Marker::POINTS;
    msg.type = visualization_msgs::Marker::CUBE_LIST;
    msg.scale.x = planner.voxel_size;
    msg.scale.y = planner.voxel_size;
    msg.scale.z = planner.voxel_size;
    int color_id = (i % color_table.size());
    msg.color = color_table[color_id];
    for (int j=0; j<planner.frontierClusterIndices[i].indices.size(); j++) {
      geometry_msgs::Point p;
      p.x = planner.frontierCloud->points[planner.frontierClusterIndices[i].indices[j]].x;
      p.y = planner.frontierCloud->points[planner.frontierClusterIndices[i].indices[j]].y;
      p.z = planner.frontierCloud->points[planner.frontierClusterIndices[i].indices[j]].z;
      msg.points.push_back(p);
    }
    cluster_marker_msg.markers.push_back(msg);
  }

  group_marker_msg.markers.clear();
  for (int i=0; i<planner.greedyGroups.size(); i++) {
    // Preliminaries
    visualization_msgs::Marker msg;
    msg.header.frame_id = planner.frame;
    msg.header.stamp = ros::Time();
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.orientation.w = 1.0;
    msg.id = i;
    // msg.type = visualization_msgs::Marker::POINTS;
    msg.type = visualization_msgs::Marker::CUBE_LIST;
    msg.scale.x = planner.voxel_size;
    msg.scale.y = planner.voxel_size;
    msg.scale.z = planner.voxel_size;
    int color_id = (i % color_table.size());
    msg.color = color_table[color_id];
    for (int j=0; j<planner.greedyGroups[i].indices.size(); j++) {
      geometry_msgs::Point p;
      p.x = planner.frontierCloud->points[planner.greedyGroups[i].indices[j]].x;
      p.y = planner.frontierCloud->points[planner.greedyGroups[i].indices[j]].y;
      p.z = planner.frontierCloud->points[planner.greedyGroups[i].indices[j]].z;
      msg.points.push_back(p);
    }
    group_marker_msg.markers.push_back(msg);
  }

  return;
}


void Msfm3d::updateFrontierMsg()
{
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

bool updateFrontier(Msfm3d& planner, ros::Publisher& frontier_publisher)
{
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
        // if (point[j] < (planner.esdf.max[j] - planner.voxel_size)) {
        query[j] = point[j] + planner.voxel_size;
        // }
        neighbor[2*j] = planner.xyz_index3(query);
        // if (point[j] > (planner.esdf.min[j] + planner.voxel_size)) {
          query[j] = point[j] - planner.voxel_size;
        // }
        neighbor[2*j+1] = planner.xyz_index3(query);
        query[j] = point[j];
      }

      // Check if the neighbor indices are unseen voxels or on the edge of the map
      if (planner.ground) {
        for (int j=0; j<4; j++) {
          // if (!planner.esdf.seen[neighbor[j]] && !(i == neighbor[j]) && !frontier) {
          bool out_of_bounds = ((neighbor[j] < 0) || neighbor[j] >= npixels);
          if (!planner.esdf.seen[neighbor[j]] || out_of_bounds) {
            frontier = 1;
            pass2++;
            break;
          }
        }
        // Eliminate frontiers with unseen top/bottom neighbors
        // if ((!planner.esdf.seen[neighbor[4]] && i != neighbor[4]) || (!planner.esdf.seen[neighbor[5]] && i != neighbor[5])) {
        //   frontier = 0;
        //   pass3++;
        // }
      }
      else {
        // For the time being, exclude the top/bottom neighbor (last two neighbors)
        for (int j=0; j<6; j++) {
        // for (int j=0; j<4; j++) {
          bool out_of_bounds = ((neighbor[j] < 0) || neighbor[j] >= npixels);
          if (!planner.esdf.seen[neighbor[j]] || out_of_bounds) {
            frontier = 1;
            pass2++;
            break;
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
        // if (planner.dzFrontierVoxelWidth > 0) {
        //   if (abs(planner.position[2] - point[2]) >= planner.dzFrontierVoxelWidth*planner.voxel_size) {
        //     pass3++;
        //     frontier = 0;
        //   }
        // }

        // // Eliminate frontiers that are adjacent to occupied cells
        // for (int j=0; j<6; j++) {
        //   if (planner.esdf.data[neighbor[j]] < (0.0217) && planner.esdf.seen[neighbor[j]]) {
        //     pass4++;
        //     frontier = 0;
        //   }
        // }
      }

      // Check if the voxel is at the entrance

      if (frontier && (dist3(point, planner.origin) <= planner.entranceRadius)) {
        pass5++;
        planner.entrance[i] = 1;
        frontier = 0;
      }

      // Check if frontier is in fence
      if (!(planner.inBoundary(point))) {
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
  ROS_INFO("%d points are free.  %d points are free and adjacent to unseen voxels.  %d points filtered for being too high/low.  %d points filtered for being adjacent to occupied voxels.  %d points at entrance.", pass1, pass2, pass3, pass4, pass5);
  ROS_INFO("Frontier updated. %d voxels initially labeled as frontier.", frontierCount);

  if (frontierCount == 0) {
    return false;
  }

  // Publish pre-filtered frontier
  // planner.updateFrontierMsg();
  // ROS_INFO("Raw frontier published.");
  // frontier_publisher.publish(planner.frontiermsg);
  // sleep(3);

  // Filter frontiers based upon normal vectors
  if (!planner.normalFrontierFilter()) return false;

  // Publish Normal Filtered Frontier
  // planner.updateFrontierMsg();
  // frontier_publisher.publish(planner.frontiermsg);
  // ROS_INFO("Normal filtered frontier published.");
  // sleep(3);

  // Cluster the frontier into euclidean distance groups
  if (planner.clusterFrontier(false)) {
    // Publish clustered frontier
    // planner.updateFrontierMsg();
    // frontier_publisher.publish(planner.frontiermsg);
    // ROS_INFO("Clustering filtered frontier published.");
    // sleep(3);
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
    if (planner.esdf.seen[i] && (planner.reach[i]<cost[0]) && (planner.esdf.data[i]>0.0 && planner.reach[i] > (double)0.0)){
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

void closestGoalView(Msfm3d& planner, int *viewIndices, double *cost, const int N, const float dMin, const float dAgentMin)
{
  double tStart = clock();
  // findFrontier scans through the environment arrays and returns the N closest frontier locations in reachability distance that are at least dMin apart.
  float point[3];
  float query[3];
  int slot = 0;
  int idx; // index of the current view pose in the reach matrix
  float d = 0.0; // distance between goal views
  float vehicle_yaw = std::atan2(2.0*(planner.q.w*planner.q.z + planner.q.x*planner.q.y), 1.0 - 2.0*(planner.q.y*planner.q.y + planner.q.z*planner.q.z)); // rad

  // Initialize cost list with negative values.
  for (int i=0; i<N; i++) {
    cost[i] = -(N-i);
    viewIndices[i] = -1;
  }

  if (planner.goalViews.size() == 0) return;

  ROS_INFO("Finding the closest viewpoint to see frontiers.");
  // Store costs and their indices in a vector
  std::vector<double> costs;
  std::vector<int> indices;
  for (int i=0; i<planner.goalViews.size(); i++){
    point[0] = planner.goalViews[i].pose.position.x;
    point[1] = planner.goalViews[i].pose.position.y;
    point[2] = planner.goalViews[i].pose.position.z;
    idx = planner.xyz_index3(point);
    if (dist3(planner.position, point) <= dAgentMin) {
      continue;
    }
    if (planner.goalViews[i].cloud.points.size() < planner.minViewCloudSize) { // Filter out goal poses that don't see enough frontier cells
      continue;
    }
    if ((idx < 0) || (idx > planner.esdf.size[0]*planner.esdf.size[1]*planner.esdf.size[2])) {
      continue;
    }
    if (planner.reach[idx] > (double)0.0) {
      // Ignore points that are clearly not in the top
        // Put the new point in the costs and indices vector
      // if (planner.updatePath(point)) {
      // Get the angle between the current robot pose and the path start
      // int j = min((int)planner.pathmsg.poses.size(), 5);
      // float start_path_vec[3] = {(float)(planner.pathmsg.poses[j].pose.position.x - planner.pathmsg.poses[0].pose.position.x),
      //                            (float)(planner.pathmsg.poses[j].pose.position.y - planner.pathmsg.poses[0].pose.position.y),
      //                            (float)(planner.pathmsg.poses[j].pose.position.z - planner.pathmsg.poses[0].pose.position.z)};
      float start_path_vec[3] = {(float)(point[0] - planner.position[0]),
                                 (float)(point[1] - planner.position[1]),
                                 (float)(point[2] - planner.position[2])};
      float start_path_yaw = std::atan2(start_path_vec[1], start_path_vec[0]); // rad
      double cost;
      cost = planner.reach[idx]*(1.0 + std::abs(angle_diff((180.0/M_PI)*start_path_yaw, (180.0/M_PI)*vehicle_yaw)/180.0)*planner.turnPenalty);
      // ROS_INFO("Vehicle yaw = %0.2f deg, Path yaw = %0.2f deg, cost = %0.2f, cost_turn = %0.2f", (180.0/M_PI)*vehicle_yaw, (180.0/M_PI)*start_path_yaw, planner.reach[idx], cost - planner.reach[idx]);
      costs.push_back(cost);
      indices.push_back(i);
      // }
    }
  }

  // Sort costs and indices vectors
  std::vector<double> costs_sorted;
  std::vector<int> indices_sorted;
  // ROS_INFO("Goal View Costs:");
  // std::cout << "[";
  for (auto i:sort_indexes(costs)) {
    costs_sorted.push_back(costs[i]);
    indices_sorted.push_back(indices[i]);
    // std::cout << costs[i] << ", ";
  }
  // std::cout << "]" << std::endl;

  // Return the N closest goal views that are at least dMin distance from those above them.
  bool addView = true;
  int addCount = 0;
  int plannerCallCount = 0;
  ROS_INFO("Closest Goal Views:");
  for (int i=0; i<costs_sorted.size(); i++) {
    point[0] = planner.goalViews[indices_sorted[i]].pose.position.x;
    point[1] = planner.goalViews[indices_sorted[i]].pose.position.y;
    point[2] = planner.goalViews[indices_sorted[i]].pose.position.z;
    if (addCount == 0) {
      plannerCallCount++;
      if (!(planner.updatePath(point))) addView = false;
    }
    for (int j=(addCount-1); j>=0; j--) {
      // std::cout << j << std::endl;
      query[0] = planner.goalViews[viewIndices[j]].pose.position.x;
      query[1] = planner.goalViews[viewIndices[j]].pose.position.y;
      query[2] = planner.goalViews[viewIndices[j]].pose.position.z;
      if (dist3(point, query) < dMin) {
        addView = false; // Don't add goal views within dMin of each other
      } else {
        plannerCallCount++;
        if(!planner.updatePath(point)) {
          addView = false;
        }
      }
    }
    if (addView) {
      viewIndices[addCount] = indices_sorted[i];
      cost[addCount] = costs_sorted[i];
      ROS_INFO("Cost: %0.2f, @ (%0.2fm, %0.2fm, %0.2fm)", cost[addCount], point[0], point[1], point[2]);
      addCount++;
    }
    if (addCount == N) {
      ROS_INFO("%d Paths calculated in optimization.  Total optimization took %0.3f seconds.", plannerCallCount, (double)(clock() - tStart)/CLOCKS_PER_SEC);
      return; // Stop if the N closest view poses are found
    }
    addView = true;
  }
  ROS_INFO("%d Paths calculated in optimization.  Total optimization took %0.3f seconds.", plannerCallCount, (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void infoGoalView(Msfm3d& planner, int *viewIndices, double *utility, const int nAgents, const float dMin, const float dAgentMin)
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

  if (planner.goalViews.size() == 0) return;

  // Find the N views with the greatest utility (frontier voxels/second) and arrange them in ascending order in utility
  for (int i=0; i<planner.goalViews.size(); i++) {
    point[0] = planner.goalViews[i].pose.position.x;
    point[1] = planner.goalViews[i].pose.position.y;
    point[2] = planner.goalViews[i].pose.position.z;
    idx = planner.xyz_index3(point);

    // Check if the robot's position is within a radius of the goal view
    if (dist3(planner.position, point) <= dAgentMin) {
      continue;
    }

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
          int j = min((int)planner.pathmsg.poses.size(), 10);
          float start_path_vec[3] = {(float)(planner.pathmsg.poses[j].pose.position.x - planner.pathmsg.poses[0].pose.position.x),
                                     (float)(planner.pathmsg.poses[j].pose.position.y - planner.pathmsg.poses[0].pose.position.y),
                                     (float)(planner.pathmsg.poses[j].pose.position.z - planner.pathmsg.poses[0].pose.position.z)};
          float start_path_yaw = std::atan2(start_path_vec[1], start_path_vec[0]); // degrees
          float vehicle_yaw = std::atan2(2.0*(planner.q.w*planner.q.z + planner.q.x*planner.q.y), 1.0 - 2.0*(planner.q.y*planner.q.y + planner.q.z*planner.q.z)); // degrees

          // Get the angle between the path end and the goal pose
          int end_path_idx = (int)planner.pathmsg.poses.size()-1;
          float end_path_vec[3] = {(float)(planner.pathmsg.poses[end_path_idx].pose.position.x - planner.pathmsg.poses[end_path_idx-j+1].pose.position.x),
                                   (float)(planner.pathmsg.poses[end_path_idx].pose.position.y - planner.pathmsg.poses[end_path_idx-j+1].pose.position.y),
                                   (float)(planner.pathmsg.poses[end_path_idx].pose.position.z - planner.pathmsg.poses[end_path_idx-j+1].pose.position.z)};;
          float end_path_yaw = std::atan2(end_path_vec[1], end_path_vec[0]); // degrees
          Quaternion goal_q = planner.goalViews[i].pose.q;
          float goal_yaw = std::atan2(2.0*(goal_q.w*goal_q.z + goal_q.x*goal_q.y), 1.0 - 2.0*(goal_q.y*goal_q.y + goal_q.z*goal_q.z)); // degrees

          // Add these angle differences to the cost_time
          // cost_time = cost_time + (std::abs(angle_diff((180.0/M_PI)*end_path_yaw, (180.0/M_PI)*goal_yaw)) + std::abs(angle_diff((180.0/M_PI)*start_path_yaw, (180.0/M_PI)*vehicle_yaw)))/planner.turnRate;
          cost_time = cost_time*(1.0 + std::abs(angle_diff((180.0/M_PI)*start_path_yaw, (180.0/M_PI)*vehicle_yaw)/180.0)*planner.turnPenalty);

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
  return;
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
    if (planner.esdf.seen[i] && (planner.reach[i]<cost[0]) && (planner.esdf.data[i]>0.0 && planner.reach[i] > (double)0.0)){
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

void reach( Msfm3d& planner, const bool usesecond, const bool usecross, const int nFront, bool reachOrigin, const double timeOut = 1.50) {
    double tStart = clock();
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
    int reachOriginIteration = npixels;
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

        if (!reachOrigin) {
          if (planner.frontier[XYZ_index]) frontCount++;
        }

        // if (XYZ_index == planner.xyz_index3(planner.origin)) break;
        if (XYZ_index == planner.xyz_index3(planner.origin) && reachOrigin) reachOriginIteration = itt;
        if ((itt - reachOriginIteration) > 200) break;

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
        // TO-DO Change criteria to stop when the best num_Neighbor poses have been reached.
        if (frontCount >= nFront && !reachOrigin) break;
        if (((double)(clock() - tStart)/CLOCKS_PER_SEC) >= timeOut) {
          ROS_INFO("Reachbility calculation timed out after %0.2f seconds.", (double)(clock() - tStart)/CLOCKS_PER_SEC);
          break;
        }
    }
    /* Free memory */
    /* Destroy parameter list */
    destroy_list(listval, listprop);
    free(neg_listx);
    free(neg_listy);
    free(neg_listz);
    delete[] Frozen;
}

sensor_msgs::PointCloud2 plotReach(Msfm3d& planner)
{
  int N = planner.esdf.size[0]*planner.esdf.size[1]*planner.esdf.size[2];
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  for (int i=0; i<N; i++) {
    if (planner.reach[i] > 0.0) {
      float query[3] = {0.0, 0.0, 0.0};
      planner.index3_xyz(i, query);
      pcl::PointXYZI point;
      point.x = query[0]; point.y = query[1]; point.z = query[2];
      point.intensity = planner.reach[i];
      cloud->points.push_back(point);
    }
  }
  // Declare a new cloud to store the converted message
  sensor_msgs::PointCloud2 newPointCloud2;

  // Convert from pcl::PointCloud to sensor_msgs::PointCloud2
  pcl::toROSMsg(*cloud, newPointCloud2);
  newPointCloud2.header.seq = 1;
  newPointCloud2.header.stamp = ros::Time();
  newPointCloud2.header.frame_id = planner.frame;
  return newPointCloud2;
}

visualization_msgs::Marker plotGoals(Msfm3d& planner)
{
  // Preliminaries
  visualization_msgs::Marker msg;
  msg.header.frame_id = planner.frame;
  msg.header.stamp = ros::Time();
  msg.action = visualization_msgs::Marker::ADD;
  msg.pose.orientation.w = 1.0;
  msg.id = 80;
  // msg.type = visualization_msgs::Marker::POINTS;
  msg.type = visualization_msgs::Marker::SPHERE_LIST;
  msg.scale.x = planner.voxel_size;
  msg.scale.y = planner.voxel_size;
  msg.scale.z = planner.voxel_size;
  msg.color.g = 1.0;
  msg.color.a = 1.0;

  // Add all of the goal points to the msg
  float point[3];
  for (int i=0; i<planner.goalViews.size(); i++) {
    geometry_msgs::Point p;
    point[0] = planner.goalViews[i].pose.position.x;
    point[1] = planner.goalViews[i].pose.position.y;
    point[2] = planner.goalViews[i].pose.position.z;
    // int idx = planner.xyz_index3(point);
    p.x = point[0];
    p.y = point[1];
    p.z = point[2];
    msg.points.push_back(p);
  }
  return msg;
}

visualization_msgs::MarkerArray plotGoalViews(Msfm3d& planner)
{
  // Preliminaries
  visualization_msgs::MarkerArray output_msg;
  visualization_msgs::Marker msg;
  msg.header.frame_id = planner.frame;
  msg.header.stamp = ros::Time();
  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::ARROW;
  msg.scale.x = 0.6;
  msg.scale.y = 0.05;
  msg.scale.z = 0.05;
  msg.color.g = 1.0;
  msg.color.a = 1.0;

  // Add all of the goal points to the msg
  float point[3];
  for (int i=0; i<planner.goalViews.size(); i++) {
    msg.id = i;
    msg.pose.position.x = planner.goalViews[i].pose.position.x;
    msg.pose.position.y = planner.goalViews[i].pose.position.y;
    msg.pose.position.z = planner.goalViews[i].pose.position.z;
    msg.pose.orientation.x = planner.goalViews[i].pose.q.x;
    msg.pose.orientation.y = planner.goalViews[i].pose.q.y;
    msg.pose.orientation.z = planner.goalViews[i].pose.q.z;
    msg.pose.orientation.w = planner.goalViews[i].pose.q.w;
    output_msg.markers.push_back(msg);
  }
  return output_msg;
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

  // Vehicle speed and turnPenalty
  float speed, turnPenalty;
  n.param("global_planning/speed", speed, (float)1.0); // m/s
  n.param("global_planning/turnPenalty", turnPenalty, (float)5.0); // deg/s
  planner.speed = speed;
  planner.turnPenalty = turnPenalty;
  ROS_INFO("Turn penalty set to %0.1f percent", planner.turnPenalty*100.0);

  // Replanning ticks
  int replan_tick_limit;
  n.param("global_planning/replan_ticks", replan_tick_limit, 15);
  int goal_tick_limit;
  n.param("global_planning/goal_ticks", goal_tick_limit, 100);
  std::string replanTrigger;
  n.param<std::string>("global_planning/replanTrigger", replanTrigger, "frontier");

  // Minimum distance for a goal pose away from current position
  float minGoalDist;
  n.param("global_planning/minGoalDist", minGoalDist, (float)0.0);
  float goalArrivalDist;
  n.param("global_planning/goalArrivalDist", goalArrivalDist, (float)0.8);

  // Clustering Parameters
  float cluster_radius, min_cluster_size;
  n.param("global_planning/cluster_radius", cluster_radius, (float)(1.5*voxel_size)); // voxels
  n.param("global_planning/min_cluster_size", min_cluster_size, (float)(5.0/voxel_size)); // voxels
  n.param("global_planning/normalThresholdZ", planner.normalThresholdZ, (float)1.1);
  planner.cluster_radius = cluster_radius;
  planner.min_cluster_size = min_cluster_size;

  // Goal Height fixing for air vehicle (For a quad with constrained AGL)
  bool fixGoalHeightAGL;
  float goalHeightAGL;
  int dzFrontierVoxelWidth;
  n.param("global_planning/fixGoalHeightAGL", fixGoalHeightAGL, false);
  n.param("global_planning/goalHeightAGL", goalHeightAGL, (float)0.64);
  n.param("global_planning/dzFrontierVoxelWidth", dzFrontierVoxelWidth, -1);
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

  // robot2camera 3-2-1 rotation angles
  float roll, pitch, yaw;
  Eigen::Matrix3f R_roll, R_pitch, R_yaw;
  n.param("global_planning/robot2camera_roll", roll, (float)0.0);
  n.param("global_planning/robot2camera_pitch", pitch, (float)0.0);
  n.param("global_planning/robot2camera_yaw", yaw, (float)0.0);
  R_roll.setZero();
  R_roll(0,0) = 1.0;
  R_roll(1,1) = std::cos((M_PI/180.0)*roll);
  R_roll(1,2) = -std::sin((M_PI/180.0)*roll);
  R_roll(2,1) = std::sin((M_PI/180.0)*roll);
  R_roll(2,2) = std::cos((M_PI/180.0)*roll);
  R_pitch.setZero();
  R_pitch(0,0) = std::cos((M_PI/180.0)*pitch);
  R_pitch(0,2) = std::sin((M_PI/180.0)*pitch);
  R_pitch(2,0) = -std::sin((M_PI/180.0)*pitch);
  R_pitch(2,2) = std::cos((M_PI/180.0)*pitch);
  R_pitch(1,1) = 1.0;
  R_yaw.setZero();
  R_yaw(0,0) = std::cos((M_PI/180.0)*yaw);
  R_yaw(0,1) = -std::sin((M_PI/180.0)*yaw);
  R_yaw(1,0) = std::sin((M_PI/180.0)*yaw);
  R_yaw(1,1) = std::cos((M_PI/180.0)*yaw);
  R_yaw(2,2) = 1.0;
  planner.robot2camera.R = R_roll*R_pitch*R_yaw;
  ROS_INFO("Camera rotation matrix: [%0.2f, %0.2f, %0.2f; %0.2f, %0.2f, %0.2f; %0.2f, %0.2f, %0.2f]", planner.robot2camera.R(0,0),
    planner.robot2camera.R(0,1), planner.robot2camera.R(0,2), planner.robot2camera.R(1,0), planner.robot2camera.R(1,1), planner.robot2camera.R(1,2),
    planner.robot2camera.R(2,0), planner.robot2camera.R(2,1), planner.robot2camera.R(2,2));

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

  // Vehicle volume free boundaries
  bool vehicleVolumeOn;
  float vehicleVolumeXmin, vehicleVolumeXmax, vehicleVolumeYmin, vehicleVolumeYmax, vehicleVolumeZmin, vehicleVolumeZmax;
  n.param("global_planning/vehicleVolumeOn", vehicleVolumeOn, false);
  n.param("global_planning/vehicleVolumeXmin", vehicleVolumeXmin, (float)-0.50);
  n.param("global_planning/vehicleVolumeXmax", vehicleVolumeXmax, (float)0.50);
  n.param("global_planning/vehicleVolumeYmin", vehicleVolumeYmin, (float)-0.50);
  n.param("global_planning/vehicleVolumeYmax", vehicleVolumeYmax, (float)0.50);
  n.param("global_planning/vehicleVolumeZmin", vehicleVolumeZmin, (float)-0.50);
  n.param("global_planning/vehicleVolumeZmax", vehicleVolumeZmax, (float)0.50);
  planner.vehicleVolume.set = vehicleVolumeOn;
  planner.vehicleVolume.xmin = vehicleVolumeXmin;
  planner.vehicleVolume.xmax = vehicleVolumeXmax;
  planner.vehicleVolume.ymin = vehicleVolumeYmin;
  planner.vehicleVolume.ymax = vehicleVolumeYmax;
  planner.vehicleVolume.zmin = vehicleVolumeZmin;
  planner.vehicleVolume.zmax = vehicleVolumeZmax;

  // Get planner operating rate in Hz
  float updateRate;
  n.param("global_planning/updateRate", updateRate, (float)1.0); // Hz

  // Width to inflate obstacles for path planning
  float inflateWidth;
  n.param("global_planning/inflateWidth", inflateWidth, (float)0.0); // meters
  planner.inflateWidth = inflateWidth;

  // Closest goal or max utility
  std::string goalFunction;
  n.param<std::string>("global_planning/goalFunction", goalFunction, "information");

  // Multi-agent parameters
  bool multiAgentOn;
  int agentCount;
  float goalViewSeparation;
  n.param("global_planning/multiAgent", multiAgentOn, false);
  n.param("global_planning/agentCount", agentCount, 5);
  n.param("global_planning/goalViewSeparation", goalViewSeparation, (float)5.0);

  // minViewCloudSize
  n.param("global_planning/minViewCloudSize", planner.minViewCloudSize, 1);

  // View pose minimum obstacle distance
  n.param("global_planning/viewPoseObstacleDistance", planner.viewPoseObstacleDistance, (float)0.01);

  ROS_INFO("Subscribing to Occupancy Grid...");
  ros::Subscriber sub0 = n.subscribe("octomap_binary", 1, &Msfm3d::callback_Octomap, &planner);

  ROS_INFO("Subscribing to ESDF or TSDF PointCloud2...");
  ros::Subscriber sub1 = n.subscribe("voxblox_node/tsdf_pointcloud", 1, &Msfm3d::callback, &planner);

  ROS_INFO("Subscribing to robot state...");
  ros::Subscriber sub2 = n.subscribe("odometry", 1, &Msfm3d::callback_position, &planner);

  ROS_INFO("Subscribing to artifact message...");
  ros::Subscriber sub3 = n.subscribe("artifact_list", 1, &Msfm3d::callback_artifactDetected, &planner);

  ROS_INFO("Subcribing to neighbor count...");
  ros::Subscriber sub4 = n.subscribe("num_neighbors", 1, &Msfm3d::callback_numNeighbors, &planner);

  ROS_INFO("Subscribing to task...");
  ros::Subscriber sub5 = n.subscribe("task", 1, &Msfm3d::callback_task, &planner);

  ROS_INFO("Subscribing to custom goal point...");
  ros::Subscriber sub6 = n.subscribe("custom_goal_point", 1, &Msfm3d::callback_customGoal, &planner);

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
  cameraFrustum.header.frame_id = planner.frame;
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

  ros::Publisher pub6 = n.advertise<sensor_msgs::PointCloud2>("inflated_octomap", 5);
  sensor_msgs::PointCloud2 inflatedOccupiedMsg;
  ros::Publisher pub7 = n.advertise<msfm3d::GoalArray>("goal_array", 5);
  msfm3d::GoalArray goalArrayMsg;
  ros::Publisher pub8 = n.advertise<std_msgs::String>("task", 5);
  std_msgs::String noPathMsg;
  noPathMsg.data = "Unable to plan";
  std_msgs::String noPathHomeMsg;
  noPathHomeMsg.data = "Unable to plan home";
  std_msgs::String gotPathHomeMsg;
  gotPathHomeMsg.data = "Able to plan home";
  ros::Publisher pub9 = n.advertise<sensor_msgs::PointCloud2>("reach_grid", 5);
  ros::Publisher pub10 = n.advertise<visualization_msgs::Marker>("goal_points", 5);
  visualization_msgs::Marker goalMsg;
  ros::Publisher pub11 = n.advertise<sensor_msgs::PointCloud2>("frontier_steps", 5);
  ros::Publisher pub12 = n.advertise<visualization_msgs::MarkerArray>("frontier_clusters", 5);
  ros::Publisher pub13 = n.advertise<visualization_msgs::MarkerArray>("frontier_groups", 5);
  visualization_msgs::MarkerArray cluster_marker_msg;
  visualization_msgs::MarkerArray group_marker_msg;
  ros::Publisher pub14 = n.advertise<visualization_msgs::MarkerArray>("goal_views", 5);
  visualization_msgs::MarkerArray goal_views_marker_msg;

  int i = 0;
  bool goalFound = 0;
  ros::Rate r(updateRate); // Hz
  clock_t tStart;
  int npixels;
  int spins = 0;
  int goalViewList[agentCount];
  double goalViewCost[agentCount];
  int oldFrontierClusterCount = 0;
  for (int i=0; i<agentCount; i++) {
    goalViewList[i] = 0;
    goalViewCost[i] = -1.0;
  }
  float goal[3] = {0.0, 0.0, 0.0};
  int replan_ticks = 0;
  double costHome = -1.0;
  int goalIndex = 0;
  nav_msgs::Path newPath;

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
    if (planner.receivedPosition) {
      ROS_INFO("Position: [x: %f, y: %f, z: %f]", planner.position[0], planner.position[1], planner.position[2]);
      i = planner.xyz_index3(planner.position);
      ROS_INFO("Index at Position: %d", i);
      if (planner.receivedMap) {
        ROS_INFO("ESDF or Occupancy at Position: %f", planner.esdf.data[i]);
        // Find frontier cells and add them to planner.frontier for output to file.
        // Publish frontiers as MarkerArray
        if (updateFrontier(planner, pub11)) {
          planner.updateFrontierMsg();
          pub3.publish(planner.frontiermsg);
          ROS_INFO("Frontier published!");

          // for (int i=0; i<cluster_marker_msg.markers.size(); i++) cluster_marker_msg.markers[i].action = visualization_msgs::Marker::DELETE;
          // pub12.publish(cluster_marker_msg);
          // for (int i=0; i<group_marker_msg.markers.size(); i++) group_marker_msg.markers[i].action = visualization_msgs::Marker::DELETE;
          // pub13.publish(group_marker_msg);

          // updateFrontierClusterGroupMsgs(planner, cluster_marker_msg, group_marker_msg);
          // pub12.publish(cluster_marker_msg);
          // pub13.publish(group_marker_msg);

          // Replan if you're within a radius of the goal point

          // Check to make sure that at least 50% of the viewed frontier voxels are still frontiers, if not, resample goal poses, if in frontier_replan mode.
          int stillFrontierCount = 0;
          bool replan = 0;
          if ((int)planner.goalViews.size() == 0 || goalViewCost[0] < 0.0) {
            replan = 1;
          } else if (replanTrigger == "frontier") {
            int viewableFrontierCount = (int)planner.goalViews[goalViewList[0]].cloud.size();
            for (int i = 0; i < viewableFrontierCount; i++) {
              pcl::PointXYZ _query = planner.goalViews[goalViewList[0]].cloud.points[i];
              float query[3] = {_query.x, _query.y, _query.z};
              int idx = planner.xyz_index3(query);
              if (planner.frontier[idx]) {
                stillFrontierCount++;
              }
            }
            if (((float)stillFrontierCount)/((float)viewableFrontierCount)<= 0.5) {
              replan = 1;
            }
          } else if (replanTrigger == "distance") {
            float _query[3] = {planner.goalViews[goalViewList[0]].pose.position.x, planner.goalViews[goalViewList[0]].pose.position.y, planner.goalViews[goalViewList[0]].pose.position.z};
            if (dist3(planner.position, _query) < goalArrivalDist) {
              replan = 1;
            }
          } else if (oldFrontierClusterCount != planner.frontierClusterIndices.size()) {
            oldFrontierClusterCount = planner.frontierClusterIndices.size();
          }

          // Inflate the obstacle map to avoid collisions
          if (planner.updatedMap) {
            planner.inflateObstacles(inflateWidth, inflatedOccupiedMsg);
            planner.updatedMap = 0;
            // Check if the goal pose is now occupied or too close to a an obstacle
            if (!replan) {
              float _query[3] = {planner.goalViews[goalViewList[0]].pose.position.x, planner.goalViews[goalViewList[0]].pose.position.y, planner.goalViews[goalViewList[0]].pose.position.z};
              int idx = planner.xyz_index3(_query);
              if ((idx < 0) || (idx >= planner.esdf.size[0]*planner.esdf.size[1]*planner.esdf.size[2])) {
                if (planner.esdf.data[idx] < (planner.viewPoseObstacleDistance)) replan = 1;
              }
            }
          }

          if (planner.esdf_or_octomap) {
            inflatedOccupiedMsg.header.seq = 1;
            inflatedOccupiedMsg.header.frame_id = "world";
            inflatedOccupiedMsg.header.stamp = ros::Time();
            pub6.publish(inflatedOccupiedMsg);
            ROS_INFO("Inflated occupancy grid published!");
          }

          if (replan || ((replan_ticks % goal_tick_limit) == 0)) {
            // Find goal poses from which to view the frontier
            planner.updateGoalPoses();
            replan = 1;
            // Plot goal points
            goalMsg.action = visualization_msgs::Marker::DELETE;
            pub10.publish(goalMsg);
            goalMsg = plotGoals(planner);
            pub10.publish(goalMsg);
            // for (int i=0; i<goal_views_marker_msg.markers.size(); i++) goal_views_marker_msg.markers[i].action = visualization_msgs::Marker::DELETE;
            // pub14.publish(goal_views_marker_msg);
            // goal_views_marker_msg = plotGoalViews(planner);
            // pub14.publish(goal_views_marker_msg);
          }

          // The robot might have moved or the goal poses may have been updated, so you need to recalculate the reachability grid
          tStart = clock();

          if ((int)planner.frontierCloud->points.size() > (planner.numNeighbors+2)*(int)planner.frontierClusterIndices[0].indices.size()) {
            ROS_INFO("Reachability matrix calculating to %d closest frontier points...", 2*(int)planner.frontierClusterIndices[0].indices.size());
            reach(planner, 0, 0, (planner.numNeighbors+2)*(int)planner.frontierClusterIndices[0].indices.size(), false);
          } else {
            ROS_INFO("Reachability matrix calculating to %d closest frontier points...", (int)planner.frontierCloud->points.size());
            reach(planner, 0, 0, (int)planner.frontierCloud->points.size(), false);
          }

          // pub9.publish(plotReach(planner));
          ROS_INFO("Reachability Grid Calculated in: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);

          // Replan if the point is no longer reachable
          if (!replan) {
            float _query[3] = {planner.goalViews[goalViewList[0]].pose.position.x, planner.goalViews[goalViewList[0]].pose.position.y, planner.goalViews[goalViewList[0]].pose.position.z};
            if (!planner.updatePath(_query)) {
              ROS_INFO("Current goal point is unreachable, replanning...");
              replan = 1;
            }
          }

          // Find the best goal pose if the replan trigger has been toggled
          if (replan || ((replan_ticks % replan_tick_limit) == 0)) {
            // ROS_INFO("At least 50 percent of the frontiers at the goal pose are no longer frontiers or %d loops have occurred since last plan.  Replanning...", (int)(replan_ticks % replan_tick_limit));
            // Find goal views with an information-based optimization
            if (goalFunction == "information") {
              infoGoalView(planner, goalViewList, goalViewCost, agentCount, goalViewSeparation, minGoalDist);
            } else {
              closestGoalView(planner, goalViewList, goalViewCost, agentCount, goalViewSeparation, minGoalDist);
            }

            // If no reasonable goal views are found, expand search to all frontier cells to see if any goal view can be found
            if (goalViewCost[0] < 0.0) {
              tStart = clock();
              ROS_INFO("Reachability matrix calculating to %d closest frontier points...", (int)planner.frontierCloud->points.size());
              tStart = clock();
              reach(planner, 0, 0, (int)planner.frontierCloud->points.size(), false);
              ROS_INFO("Reachability Grid Calculated in: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);
              if (goalFunction == "information") {
                infoGoalView(planner, goalViewList, goalViewCost, agentCount, goalViewSeparation, minGoalDist);
              } else {
                closestGoalView(planner, goalViewList, goalViewCost, agentCount, goalViewSeparation, minGoalDist);
              }
            }

            // Output status for debugging
            if (planner.goalViews.size() > 0) { // Check to make sure there are at least enough goal poses for the number of robots
              if (goalFunction == "information") {
                for (int i=0; i<agentCount; i++) {
                  if (goalViewCost[i] >= 0.0) ROS_INFO("Frontier Pose Position: [x: %f, y: %f, z: %f, utility: %f]", planner.goalViews[goalViewList[i]].pose.position.x,
                    planner.goalViews[goalViewList[i]].pose.position.y, planner.goalViews[goalViewList[i]].pose.position.z, goalViewCost[i]);
                }
              } else {
                for (int i=0; i<agentCount; i++) {
                  if (goalViewCost[i] >= 0.0) ROS_INFO("Frontier Pose Position: [x: %f, y: %f, z: %f, cost: %f]", planner.goalViews[goalViewList[i]].pose.position.x,
                    planner.goalViews[goalViewList[i]].pose.position.y, planner.goalViews[goalViewList[i]].pose.position.z, goalViewCost[i]);
                }
              }


              // If using multi-agent, publish a goal decision matrix, goalArray, and the corresponding pathArray msg
              int rows = 0;
              msfm3d::GoalArray newGoalArrayMsg;
              for (int i=0; i<agentCount; i++) {
                if (goalViewCost[i]>=0.0) {
                  pcl::PointXYZ _query = planner.goalViews[goalViewList[i]].pose.position; // pcl::PointXYZ
                  float query[3] = {_query.x, _query.y, _query.z}; // float[3]
                  if (planner.updatePath(query)) {
                    msfm3d::Goal newGoalMsg;
                    // Write a new goal pose, path, and cost for publishing
                    goalPose.header.stamp = ros::Time::now();
                    goalPose.pose.position.x = query[0];
                    goalPose.pose.position.y = query[1];
                    goalPose.pose.position.z = query[2];
                    goalPose.pose.orientation.x = planner.goalViews[goalViewList[i]].pose.q.x;
                    goalPose.pose.orientation.y = planner.goalViews[goalViewList[i]].pose.q.y;
                    goalPose.pose.orientation.z = planner.goalViews[goalViewList[i]].pose.q.z;
                    goalPose.pose.orientation.w = planner.goalViews[goalViewList[i]].pose.q.w;
                    newGoalMsg.pose = goalPose;
                    newGoalMsg.path = planner.pathmsg;
                    newGoalMsg.cost.data = (float)goalViewCost[i];
                    newGoalArrayMsg.goals.push_back(newGoalMsg);
                    newGoalArrayMsg.costHome.data = (float)costHome;
                  }
                }
              }
              goalArrayMsg = newGoalArrayMsg;
              pub7.publish(goalArrayMsg);
            }
          }
          // Store goal location for publishing
          if ((goalViewCost[0] < 0.0) || planner.artifactDetected || (planner.task == "Home") || (planner.task == "Report")) {
            // The vehicle couldn't find any reasonable frontier views, head to the anchor node.
            ROS_INFO("No reachable frontier views after expanded search or artifact detected, heading to the anchor.");
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
            costHome = planner.reach[planner.xyz_index3(goal)];
            goalArrayMsg.costHome.data = (float)costHome;
            pub7.publish(goalArrayMsg);

            // Write a new goal pose for publishing
            goalPose.header.stamp = ros::Time::now();
            goalPose.pose.position.x = goal[0];
            goalPose.pose.position.y = goal[1];
            goalPose.pose.position.z = goal[2];
            goalPose.pose.orientation.x = 0.0;
            goalPose.pose.orientation.y = 0.0;
            goalPose.pose.orientation.z = 0.0;
            goalPose.pose.orientation.w = 1.0;

          } else if (planner.task == "guiCommand") {
            frontierGoal.point.x = planner.customGoal[0];
            frontierGoal.point.y = planner.customGoal[1];
            frontierGoal.point.z = planner.customGoal[2];
            goal[0] = planner.customGoal[0];
            goal[1] = planner.customGoal[1];
            goal[2] = planner.customGoal[2];

            // Write a new goal pose for publishing
            goalPose.header.stamp = ros::Time::now();
            goalPose.pose.position.x = goal[0];
            goalPose.pose.position.y = goal[1];
            goalPose.pose.position.z = goal[2];
            goalPose.pose.orientation.x = 0.0;
            goalPose.pose.orientation.y = 0.0;
            goalPose.pose.orientation.z = 0.0;
            goalPose.pose.orientation.w = 1.0;
          } else {
            // Write a new frontier goal location for publishing
            frontierGoal.point.x = planner.goalViews[goalViewList[0]].pose.position.x;
            frontierGoal.point.y = planner.goalViews[goalViewList[0]].pose.position.y;
            frontierGoal.point.z = planner.goalViews[goalViewList[0]].pose.position.z;
            goal[0] = planner.goalViews[goalViewList[0]].pose.position.x;
            goal[1] = planner.goalViews[goalViewList[0]].pose.position.y;
            goal[2] = planner.goalViews[goalViewList[0]].pose.position.z;

            // Write a new goal pose for publishing
            goalPose.header.stamp = ros::Time::now();
            goalPose.pose.position.x = goal[0];
            goalPose.pose.position.y = goal[1];
            goalPose.pose.position.z = goal[2];
            goalPose.pose.orientation.x = planner.goalViews[goalViewList[0]].pose.q.x;
            goalPose.pose.orientation.y = planner.goalViews[goalViewList[0]].pose.q.y;
            goalPose.pose.orientation.z = planner.goalViews[goalViewList[0]].pose.q.z;
            goalPose.pose.orientation.w = planner.goalViews[goalViewList[0]].pose.q.w;
          }

          // Publish goal point, pose, view frustum, path, etc.
          pub1.publish(frontierGoal);
          pub4.publish(goalPose);
          ROS_INFO("Goal point published!");

          // Calculate and publish path
          if (!(planner.updatePath(goal))) {
            if (planner.task == "guiCommand") pub8.publish(noPathMsg);
            if ((planner.task == "Home") || (planner.task == "Report")) pub8.publish(noPathHomeMsg);
            ROS_WARN("Couldn't find feasible path to goal.  Publishing previous path");
          } else {
            if ((planner.task == "Home") || (planner.task == "Report")) pub8.publish(gotPathHomeMsg);
          }

          if (planner.ground) {
            newPath = planner.pathmsg;
          } else if (replan) {
            newPath = planner.pathmsg;
          }
          pub2.publish(newPath);
          pub9.publish(plotReach(planner));
          ROS_INFO("Path to goal published!");
          goalFound = 0;

          // Publish view frustum
          cameraFrustum.action = 2; // DELETE action
          pub5.publish(cameraFrustum);
          cameraFrustum.action = 0; // ADD action
          if (goalViewCost[0] >= 0.0) {
            view2MarkerMsg(planner.goalViews[goalViewList[0]], planner.camera, cameraFrustum, planner.robot2camera);
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

          ROS_INFO("No frontiers after filtering or home command given, robot is heading home.");
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
          if (!goalFound && ((planner.task == "Home") || (planner.task == "Report"))) pub8.publish(noPathHomeMsg);
          if (goalFound && ((planner.task == "Home") || (planner.task == "Report"))) pub8.publish(gotPathHomeMsg);

           // Publish path, goal point, and goal point only path
          pub1.publish(frontierGoal);
          pub4.publish(goalPose);
          ROS_INFO("Goal point published!");

          // Output and publish path
          pub2.publish(planner.pathmsg);
          ROS_INFO("Path to goal published!");
          goalFound = 0;
          pub9.publish(plotReach(planner));

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
