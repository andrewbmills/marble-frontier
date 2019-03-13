// g++ msfm3d_node.cpp -g -o msfm3d_node.o -I /opt/ros/melodic/include -I /usr/include/c++/7.3.0 -I /home/andrew/catkin_ws/devel/include -I /home/andrew/catkin_ws/src/octomap_msgs/include -I /usr/include/pcl-1.8 -I /usr/include/eigen3 -L /usr/lib/x86_64-linux-gnu -L /home/andrew/catkin_ws/devel/lib -L /opt/ros/melodic/lib -Wl,-rpath,opt/ros/melodic/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization -loctomap -lboost_system -lpcl_common -lpcl_io -lpcl_filters -lpcl_features -lpcl_kdtree -lpcl_segmentation


#include <math.h>
#include <algorithm>
#include <random>
// ROS libraries
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
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
  float sum;
  for (int i=0; i < 2; i++) sum += (b[i] - a[i])*(b[i] - a[i]);
  return std::sqrt(sum);
}

float dist3(const float a[3], const float b[3]){
  float sum;
  for (int i=0; i < 3; i++) sum += (b[i] - a[i])*(b[i] - a[i]);
  return std::sqrt(sum);
}

int sign(float a){
  if (a>0.0) return 1;
  if (a<0.0) return -1;
  return 0;
}

// Some orientation and pose structures
struct Quaternion {
  float w, x, y, z;
};
struct Pose {
  pcl::PointXYZ position;
  Eigen::Matrix3f R;
};
struct Sensor {
  float verticalFoV = 0.0; // in degrees
  float horizontalFoV = 0.0; // in degrees
  float rMin = 0.0; // in meters
  float rMax = 1.0; // in meters
};

//Msfm3d class declaration
class Msfm3d
{
  public:
    // Constructor
    Msfm3d():
    frontierCloud(new pcl::PointCloud<pcl::PointXYZ>)
    {
      reach = NULL;
      esdf.data = NULL;
      esdf.seen = NULL;
      frontier = NULL;
      entrance = NULL;
      tree = NULL;
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
    bool ground = 0; // whether the vehicle is a ground vehicle
    float wheel_bottom_dist = 0.0;
    float position[3] = {69.0, 420.0, 1337.0}; // robot position
    float euler[3]; // robot orientation in euler angles
    float R[9]; // Rotation matrix
    boundary vehicleVolume; // xyz boundary of the vehicle bounding box in rectilinear coordinates for collision detection/avoidance

    // Environment/Sensor parameters
    std::string frame = "world";
    bool esdf_or_octomap = 0; // Boolean to use an esdf PointCloud2 or an Octomap as input
    bool receivedPosition = 0;
    bool receivedMap = 0;
    float voxel_size;
    float bubble_radius = 1.5; // map voxel size, and bubble radius
    float origin[3]; // location in xyz coordinates where the robot entered the environment
    
    double * reach; // reachability grid (output from reach())
    sensor_msgs::PointCloud2 PC2msg;
    nav_msgs::Path pathmsg;
    // visualization_msgs::MarkerArray frontiermsg;
    octomap::OcTree* tree; // OcTree object for holding Octomap

    // Frontier and frontier filter parameters
    bool * frontier;
    bool * entrance;
    int frontier_size = 0;
      // Clustering/Filtering
      pcl::PointCloud<pcl::PointXYZ>::Ptr frontierCloud; // Frontier PCL
      std::vector<pcl::PointIndices> frontierClusterIndices;
      // ROS Interfacing
      sensor_msgs::PointCloud2 frontiermsg;
      // Frontier Grouping
      std::vector<pcl::PointIndices> greedyGroups;
      pcl::PointCloud<pcl::PointXYZ> greedyCenters;

    // Vector of possible goal poses
    struct View {
      Pose pose;
      pcl::PointCloud<pcl::PointXYZ> cloud;
    };
    std::vector<View> goalViews;

    // Sensor parameters
    Sensor camera;
    camera.verticalFoV = 45.0;
    camera.horizontalFoV = 60.0;
    camera.rMin = 1.0;
    camera.rMax = 5.0;

    Quaternion q; // robot orientation in quaternions
    ESDF esdf; // ESDF struct object
    Boundary bounds; // xyz boundary of possible goal locations

    void callback(sensor_msgs::PointCloud2 msg); // Subscriber callback function for PC2 msg (ESDF)
    void callback_Octomap(const octomap_msgs::Octomap::ConstPtr msg); // Subscriber callback function for Octomap msg
    void callback_position(const nav_msgs::Odometry msg); // Subscriber callback for robot position
    void parsePointCloud(); // Function to parse pointCloud2 into an esdf format that msfm3d can use
    int xyz_index3(const float point[3]);
    void index3_xyz(const int index, float point[3]);
    void getEuler(); // Updates euler array given the current quaternion values
    void getRotationMatrix(); // Updates Rotation Matrix given the current quaternion values
    bool updatePath(const float goal[3]); // Updates the path vector from the goal frontier point to the robot location
    void updateFrontierMsg(); // Updates the frontiermsg MarkerArray with the frontier matrix for publishing
    void clusterFrontier(const bool print2File); // Clusters the frontier pointCloud with euclidean distance within a radius
    bool inBoundary(const float point[3]); // Checks if a point is inside the planner boundaries
    bool collisionCheck(const float point[3]); // Checks if the robot being at the current point (given vehicleVolume) intersects with the obstacle environment (esdf)
    void greedyGrouping(const float r, const bool print2File);
    bool raycast(const pcl::PointXYZ start, const pcl::PointXYZ end);
    Pose samplePose(const pcl::PointXYZ centroid, const Sensor camera, const int sampleLimit);
    void updateGoalPoses();
};

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

void Msfm3d::clusterFrontier(const bool print2File)
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
  tree->setInputCloud(frontierCloud);

  // Clear previous frontierClusterIndices
  frontierClusterIndices.clear();

  // Initialize euclidean cluster extraction object
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(1.5*voxel_size); // Clusters must be made of contiguous sections of frontier (within sqrt(2)*voxel_size of each other)
  ec.setMinClusterSize(roundf(12.0/voxel_size)); // Cluster must be at least 15 voxels in size
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

  // Get new indices of Frontier Clusters after filtering (extract filter does not preserve indices);
  frontierClusterIndices.clear();
  tree->setInputCloud(frontierCloud);
  ec.setSearchMethod(tree);
  ec.setInputCloud(frontierCloud);
  ec.extract(frontierClusterIndices);

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
      ROS_INFO("Initializing ungrouped_cloud for group %d containing %d points.", groupCount, (int)ungrouped_cloud->points.size());

      // Sample a random index from ungrouped
      std::uniform_int_distribution<int> dist(0, (int)ungrouped->indices.size()-1);
      int sample_id = ungrouped->indices[dist(mt)];
      pcl::PointXYZ sample_point = frontierCloud->points[sample_id];
      ROS_INFO("Sampled index %d which is located at (%f, %f, %f).", sample_id, sample_point.x, sample_point.y, sample_point.z);

      // Find all the voxels in ungrouped_cloud that are within r of sample_point
      pcl::PointIndices::Ptr group(new pcl::PointIndices);
      filterCloudRadius(radius, sample_point, ungrouped_cloud, group);
      ROS_INFO("Found %d points within %f of the sample point.", (int)group->indices.size(), r);

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
}

bool Msfm3d::collisionCheck(const float position[3]) 
{
  // Get indices corresponding to the voxels occupied by the vehicle
  int lower_corner[3], voxel_width[3], idx, npixels = esdf.size[0]*esdf.size[1]*esdf.size[2]; // xyz point of the lower left of the vehicle rectangle
  lower_corner[0] = position[0] + vehicleVolume.xmin;
  lower_corner[1] = position[1] + vehicleVolume.ymin;
  lower_corner[2] = position[2] + vehicleVolume.zmin;
  voxel_width[0] = roundf((vehicleVolume.xmax - vehicleVolume.xmin)/voxel_size);
  voxel_width[1] = roundf((vehicleVolume.xmax - vehicleVolume.xmin)/voxel_size);
  voxel_width[2] = roundf((vehicleVolume.xmax - vehicleVolume.xmin)/voxel_size);
  bool collision = 0;
  float query[3];

  // Loop through all of the voxels occupied by the vehicle and check for occupancy to detect a collision
  // Return when a collision is detected
  for (int i=0; i<voxel_width[0]; i++){
    query[0] = lower_corner[0] + i*voxel_size;
    for (int j=0; j<voxel_width[1]; j++){
      query[1] = lower_corner[1] + j*voxel_size;
      for (int k=0; k<voxel_width[2]; k++){
        query[2] = lower_corner[2] + k*voxel_size;
        idx = xyz_index3(query);
        // Check to see if idx is inside a valid index
        if (idx < 0 || idx >= npixels ) {
          if (ground && (query[2] >= (position[2] - wheel_bottom_dist + voxel_size/2.0)) && esdf.data[idx] < 0.0) {
          	return 1; // Check for collisions above the wheels on the ground robot
      	  }
          if (!ground && esdf.data[idx] < 0.0) {
          	return 1; // Check for air vehicle collision
          }
          if (ground && (query[2] <= (position[2] - wheel_bottom_dist - voxel_size/2.0)) && esdf.data[idx] > 0.0) { // Check for wheel contact with the ground for the ground vehicle
            ROS_INFO("Original path doesn't keep the ground vehicle on the ground, replanning to next closest frontier voxel...");
            return 1;
          }
        }
      }
    }
  }
}

bool Msfm3d::raycast(const pcl::PointXYZ start, const pcl::PointXYZ end) {
  float dx = end.x - start.x;
  float dy = end.y - start.y;
  float dz = end.z - start.z;
  float radius = std::sqrt(dx*dx + dy*dy + dz*dz);

  if (esdf_or_octomap) {
    // Perform a raycast in Octomap
    octomap::point3d origin;
    origin.x = start.x;
    origin.y = start.y;
    origin.z = start.z;

    octomap::point3d direction;
    direction.x = dx;
    direction.y = dy;
    direction.z = dz;

    octomap::point3d stop;

    if (tree.castRay(origin, direction, stop, false, (double)radius)) {
      return false;
    }

    // Calculate the distance between the end and centroid
    float diff_x = (end.x - stop.x);
    float diff_y = (end.y - stop.y);
    float diff_z = (end.z - stop.z);
    float distanced = std::sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);
    if (distance > voxel_size) {
      // castRay hit an unseen voxel before hitting the centroid, sample another point
      return false;
    }
    return true;
  } else {
    // I don't have this built out yet
    ROS_INFO("Occlusion detection is not defined for ESDF at the moment.  Use Octomap for pose sampling.");
    return false;
  }
}

Pose Msfm3d::samplePose(const pcl::PointXYZ centroid, const Sensor camera, const int sampleLimit)
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
  robotPose.position.x = NAN;
  robotPose.position.y = NAN;
  robotPose.position.z = NAN;
  robotPose.R.setZero();
  // Intialize random seed:
  std::random_device rd;
  std::mt19937 mt(rd());

  // Uniform continuous distributions over spherical coordinates
  std::uniform_real_distribution<float> radius_dist(camera.rMin, camera.rMax);
  std::uniform_real_distribution<float> azimuth_dist(0, 2*M_PI);
  std::uniform_real_distribution<float> elevation_dist((M_PI - camera.verticalFoV)/2.0, (M_PI + camera.verticalFoV)/2.0);

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

    // Find the index of the sample
    int sample_idx = xyz_index3({sample.x, sample.y, sample.z});

    // See if the sample is within the current map range
    if ((sample_idx < 0) || (sample_idx >= esdf.size[0]*esdf.size[1]*esdf.size[2])) {
      continue;
    }

    // See if sample is at an unreachable point (occupied or unseen)
    if (esdf.data[idx] <= 0.0) {
      continue;
    }

    // Check for occlusion
    if (!raycast(sample, centroid)) {
      continue;
    }
  
    // This pose is valid! Save it and exit the loop.
    float sample_yaw = M_PI + azimuth_sample;
    robotPose.position = sample;
    robotPose.R(0,0) = std::cos(sample_yaw);
    robotPose.R(0,1) = -std::sin(sample_yaw);
    robotPose.R(1,0) = std::sin(sample_yaw);
    robotPose.R(1,1) = std::cos(sample_yaw);
    robotPose.R(2,2) = 1.0;
    break;
  }

  return robotPose;
}

void Msfm3d::updateGoalPoses()
{
  // Conversion matrix from ENU to EUS
  Eigen::Matrix4f ENU_to_PCL;
  ENU_to_PCL << 1, 0, 0, 0
                0, 0, 1, 0
                0, -1, 0, 0
                0, 0, 0, 1;

  // Clear previous goalViews vector
  goalViews.clear();

  // Loop through the frontier group centroids
  for (pcl::PointCloud<pcl::PointXYZ>::iterator it = greedyCenters->begin(); it != greedyCenters->end(); ++it) {
    // Sample an admissable pose that sees the centroid
    Pose goalPose = samplePose(*it, camera, 50);
    if std::isnan(goalPose.position.x) {
      continue;
    }

    // Convert the goalPose into a 4x4 view matrix in EUS (East Up South) or (Forward-Up-Right)
    Eigen::Matrix4f camera_pose;
    camera_pose.setZero();
    camera_pose.topLeftCorner(3,3) = goalPose.R;
    camera_pose(0,3) = goalPose.position.x;
    camera_pose(1,3) = goalPose.position.y;
    camera_pose(2,3) = goalPose.position.z;
    camera_pose(3,3) = 1.0;
    camera_pose = ENU_to_PCL*camera_pose; // Rotate camera_pose into weird PCL EUS coordinates

    // Cull the view frustum points with the pcl function
    pcl::FrustumCulling<pcl::PointXYZ> fc;
    fc.setInputCloud(source);
    fc.setVerticalFOV(camera.verticalFoV);
    fc.setHorizontalFOV(camera.horizontalFoV);
    fc.setNearPlaneDistance(camera.rMin);
    fc.setFarPlaneDistance(camera.rMax);
    fc.setCameraPose(camera_pose);
    pcl::PointCloud<pcl::PointXYZ> viewed_cloud;
    fc.filter(viewed_cloud);

    // Perform raycasting and check which culled cloud points are visible from goalPose.position
    pcl::PointIndices::Ptr seen(new pcl::PointIndices());
    for (int i = 0; i < viewed_cloud.points.size(); i++) {
      if (raycast(goalPose.position, viewed_cloud.points[i])) {
        seen->indices.push_back(i);
      }
    }

    // Extract seen cloud members from viewed_cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(viewed_cloud);
    extract.setIndices(seen);
    extract.setNegative(false);
    extract.filter(viewed_cloud);

    // Add the pose and the viewed frontier indices to the goalViews vector.
    View sampleView;
    sampleView.pose = goalPose;
    sampleView.cloud = viewed_cloud;
    goalViews.push_back(sampleView);
  }
}

void Msfm3d::getRotationMatrix()
{
  if (receivedPosition) {
  	// Compute the 3-2-1 rotation matrix for the vehicle from its current orientation in quaternions (q)

    // First Row
    R[0] = q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z;
    R[1] = 2.0*(q.x*q.y - q.w*q.z);
    R[2] = 2.0*(q.w*q.y + q.x*q.z);
    // Second Row
    R[3] = 2.0*(q.x*q.y + q.w*q.z);
    R[4] = q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z;
    R[5] = 2.0*(q.y*q.z - q.w*q.x);
    // Third Row
    R[6] = 2.0*(q.x*q.z - q.w*q.y);
    R[7] = 2.0*(q.w*q.x + q.y*q.z);
    R[8] = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
  }
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

  // Free/Allocate the tree memory
  // ROS_INFO("Converting Octomap msg to AbstractOcTree...");
  // octomap::AbstractOcTree* abstree = new octomap::AbstractOcTree(msg->resolution);
  // abstree = octomap_msgs::binaryMsgToMap(*msg); // OcTree object for storing Octomap data.
  // ROS_INFO("Octomap converted to AbstractOcTree.");
  octomap::OcTree* mytree = new octomap::OcTree(msg->resolution);
  mytree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
  ROS_INFO("AbstractOcTree cast into OcTree.");

  ROS_INFO("Parsing Octomap...");
  // Make sure the tree is at the same resolution as the esdf we're creating.  If it's not, change the resolution.
  ROS_INFO("Tree resolution is %f meters.", mytree->getResolution());
  if (mytree->getResolution() != (double)voxel_size) {
    mytree->setResolution((double)voxel_size);
    mytree->prune();
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
    if (it->getValue() > 0){
      value = 0.0;
    } else{
      value = 1.0;
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

  // Free memory for AbstractOcTree object pointer
  ROS_INFO("Freeing OcTree and AbstractOcTree memory.");
  delete mytree;
  ROS_INFO("OctoMap message received!");
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
  float xyzis[5*(PC2msg.width)];
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
  int ijk000[3]; // i,j,k indices of corner[0]
  float xyz000[3]; // x,y,z, coordinates of corner[0];
  int corner[8]; // indices of the corners of the 8 closest voxels to the current point
  int neighbor[6]; // neighbor voxels to the current voxel
  float point[3]; // current point x,y,z coordinates
  float query[3]; // intermediate x,y,z coordinates for another operation
  float grad[3]; // gradient at the current point
  float gradcorner[24]; // corner voxel gradients
  float grad_norm = 1.0; // norm of the gradient vector
  float dist_robot2path = 10*voxel_size; // distance of the robot to the path
  float position2D[2];
  float point2D[2];
  std::vector<float> path;

  // 3D Interpolation intermediate values from https://en.wikipedia.org/wiki/Trilinear_interpolation
  float xd, yd, zd, c00, c01, c10, c11, c0, c1; // 3D linear interpolation weights.
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
    else {dist_robot2path = dist3(position, point);}

    // Check for a collision with the environment to make sure this goal point is feasible.
    if (vehicleVolume.set) {
      if (collisionCheck(point)){
        // Take the goal point out of the frontier and entrance arrays
        frontier[xyz_index3(goal)] = 0;
        entrance[xyz_index3(goal)] = 0;
        return 0;
      }
    }
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
  return 1;
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

void updateFrontier(Msfm3d& planner){
  ROS_INFO("Beginning Frontier update step...");
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

  // Extra variables for ground vehicle case so that only frontier close to vehicle plane are chosen.
  for (int i=0; i<npixels; i++){
    // Get the 3D point location
    planner.index3_xyz(i, point);
    _point.x = point[0];
    _point.y = point[1];
    _point.z = point[2];

    // Check if the voxel has been seen and is unoccupied
    if (planner.esdf.seen[i] && (planner.esdf.data[i]>0.0) && planner.inBoundary(point) && (dist3(point, planner.position) >= planner.bubble_radius)){

      // Check if the voxel is a frontier by querying adjacent voxels
      for (int j=0; j<3; j++) query[j] = point[j];

      // Create an array of neighbor indices
      for (int j=0; j<3; j++){
        if (point[j] < (planner.esdf.max[j] - planner.voxel_size)) query[j] = point[j] + planner.voxel_size;
        neighbor[2*j] = planner.xyz_index3(query);
        if (point[j] > (planner.esdf.min[j] + planner.voxel_size)) query[j] = point[j] - planner.voxel_size;
        neighbor[2*j+1] = planner.xyz_index3(query);
        query[j] = point[j];
      }
      // ROS_INFO("There are %d voxels in the current ESDF.", npixels);
      // ROS_INFO("Neighbor indices for cell %d are %d, %d, %d, %d, %d, and %d.", i, neighbor[0], neighbor[1], neighbor[2], neighbor[3], neighbor[4], neighbor[5]);
      // Check if the neighbor indices are unseen voxels
      if (planner.ground) {
        for (int j=0; j<4; j++) {
          if (!planner.esdf.seen[neighbor[j]] && !(i == neighbor[j])) frontier = 1;
        }
      }
      else {
        // For the time being, exclude the top/bottom neighbor (last two neighbors)
        // for (int j=0; j<6; j++) {
        for (int j=0; j<4; j++) {
          if (!planner.esdf.seen[neighbor[j]]  && !(i == neighbor[j])) frontier = 1;
        }
        // if (!planner.esdf.seen[neighbor[5]]  && !(i == neighbor[5])) frontier = 1;
      }
      // Check if the point is on the ground if it is a ground robot
      if (frontier && planner.ground) {
        // Only consider frontiers on the floor
        // if (planner.esdf.data[neighbor[5]] > (0.01)) frontier = 0;
        // if (!planner.esdf.seen[neighbor[5]]) frontier = 0;

        // Only consider frontiers close in z-coordinate (temporary hack)
        // if (abs(planner.position[2] - point[2]) >= 2*planner.voxel_size) frontier = 0;

        // Eliminate frontiers that are adjacent to occupied cells (unless it's the bottom neighbor for the ground case)
        for (int j=0; j<6; j++) {
          if (planner.esdf.data[neighbor[j]] < (0.01) && planner.esdf.seen[neighbor[j]]) frontier = 0;
        }
      }
      else if (frontier && !planner.ground) {
        // Eliminate frontiers that are adjacent to occupied cells
        for (int j=0; j<6; j++) {
          if (planner.esdf.data[neighbor[j]] < (0.01) && planner.esdf.seen[neighbor[j]]) frontier = 0;
        }
      }

      // Check if the voxel is at the entrance
      if (frontier && (dist3(point, planner.origin) <= 25.0)) {
        planner.entrance[i] = 1;
        frontier = 0;
      }

      // If the current voxel is a frontier, add the  current voxel location to the planner.frontier array
      if (frontier) {
        frontier = 0; // reset for next loop
        planner.frontier[i] = 1;
        planner.frontierCloud->push_back(_point);
      }
    }
  }
  ROS_INFO("Frontier updated.");

  // Filter the updated frontier with pcl::radius_outlier_filter
  if (planner.frontierFilterOn){
    planner.filterFrontier();
  }

  // Cluster the frontier into euclidean distance groups
  planner.clusterFrontier(true);

  // Group frontier within each cluster with greedy algorithm
  planner.greedyGrouping(4*planner.voxel_size, true);

}

void findFrontier(Msfm3d& planner, float frontierList[15], double cost[5]){
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

void findEntrance(Msfm3d& planner, float entranceList[15], double cost[5]){
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

void reach( Msfm3d& planner, const bool usesecond, const bool usecross, const int nFront) {

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

    /* Frontier Count */
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
        if(IsInf(neg_listv[index])) { break; }
        
        /*index=minarray(neg_listv, neg_pos); */
        x=(int)neg_listx[index]; y=(int)neg_listy[index]; z=(int)neg_listz[index];
        XYZ_index=mindex3(x, y, z, dims[0], dims[1]);
        Frozen[XYZ_index]=1;
        T[XYZ_index]=neg_listv[index];
        if(Ed) { Y[XYZ_index]=neg_listo[index]; }

        /*Remove min value by replacing it with the last value in the array */
        list_remove_replace(listval, listprop, index) ;
        neg_listv=listval[listprop[1]-1];
        if(index<(neg_pos-1)) {
            neg_listx[index]=neg_listx[neg_pos-1];
            neg_listy[index]=neg_listy[neg_pos-1];
            neg_listz[index]=neg_listz[neg_pos-1];
            if(Ed){
                neg_listo[index]=neg_listo[neg_pos-1];
            }
            T[(int)mindex3((int)neg_listx[index], (int)neg_listy[index], (int)neg_listz[index], dims[0], dims[1])]=index;
        }
        neg_pos =neg_pos-1;

        // Check to see if the current point is a frontier.
        // If so, add to the frontier counter.
        if (planner.frontier[XYZ_index]) frontCount++;

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
        if (frontCount >= nFront) break;
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
  ros::init(argc, argv, "msfm3d_node");
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // Initialize planner object
  ROS_INFO("Initializing msfm3d planner...");
  Msfm3d planner;
  planner.ground = 1;
  planner.esdf_or_octomap = 1; // Use a TSDF message (0) or Use an octomap message (1)
  planner.origin[0] = 0.0;
  planner.origin[1] = 0.0;
  planner.origin[2] = 0.0;
  // planner.bubble_radius = 3.0;
  // Set planner bounds so that the robot doesn't exit a defined volume
  // planner.bounds.set = 1;
  // planner.bounds.xmin = -5.0;
  // planner.bounds.xmax = 50.0;
  // planner.bounds.ymin = 40.0;
  // planner.bounds.ymax = -40.0;
  // planner.bounds.zmin = -0.5;
  // planner.bounds.zmax = 2.0;

  // planner.name = "X1";

  // Get voxblox voxel size parameter
  // n.getParam("/X4/voxblox_node/tsdf_voxel_size", planner.voxel_size);

  // Voxel size for Octomap or Voxblox
  planner.voxel_size = 0.2;

  // Frontier filter parameters
  planner.frontierFilterOn = 0;
  // planner.filter_radius = 5*planner.voxel_size;
  // planner.filter_neighbors = 10;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  // if (planner.esdf_or_octomap) {
  ROS_INFO("Subscribing to Occupancy Grid...");
  ros::Subscriber sub1 = n.subscribe("/octomap_binary", 1, &Msfm3d::callback_Octomap, &planner);
  // }
  // else {
  // ROS_INFO("Subscribing to ESDF or TSDF PointCloud2...");
  // ros::Subscriber sub1 = n.subscribe("/X1/voxblox_node/tsdf_pointcloud", 1, &Msfm3d::callback, &planner);
  // }
  ROS_INFO("Subscribing to robot state...");
  ros::Subscriber sub2 = n.subscribe("/X1/odom_truth", 1, &Msfm3d::callback_position, &planner);

  ros::Publisher pub1 = n.advertise<geometry_msgs::PointStamped>("/X1/nearest_frontier", 5);
  geometry_msgs::PointStamped frontierGoal;
  frontierGoal.header.frame_id = planner.frame;

  // Publish goal point to interface with btraj
  ros::Publisher pub4 = n.advertise<geometry_msgs::PoseStamped>("/X1/frontier_goal_pose", 5);
  geometry_msgs::PoseStamped goalPose;
  goalPose.header.frame_id = planner.frame;
  goalPose.pose.position.x = 0.0;
  goalPose.pose.position.y = 0.0;
  goalPose.pose.position.z = 0.0;
  goalPose.pose.orientation.w = 1.0;
  goalPose.header.seq = 1;

  ros::Publisher pub2 = n.advertise<nav_msgs::Path>("/X1/planned_path", 5);
  // ros::Publisher pub3 = n.advertise<visualization_msgs::MarkerArray>("/X1/frontier", 100);
  ros::Publisher pub3 = n.advertise<sensor_msgs::PointCloud2>("/X1/frontier", 5);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  int i = 0;
  bool goalFound = 0;
  ros::Rate r(1); // 1 hz
  clock_t tStart;
  int npixels;
  int spins = 0;
  float frontierList[15];
  double frontierCost[5];
  float goal[3] = {0.0, 0.0, 0.0};
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
      ROS_INFO("X1 Position: [x: %f, y: %f, z: %f]", planner.position[0], planner.position[1], planner.position[2]);
      i = planner.xyz_index3(planner.position);
      ROS_INFO("Index at Position: %d", i);
      if (planner.receivedMap){
        ROS_INFO("ESDF or Occupancy at Position: %f", planner.esdf.data[i]);

        // Find frontier cells and add them to planner.frontier for output to file.
        // Publish frontiers as MarkerArray
        updateFrontier(planner);
        planner.updateFrontierMsg();
        pub3.publish(planner.frontiermsg);
        ROS_INFO("Frontier published!");
        return 1;

        // Call msfm3d function
        tStart = clock();
        ROS_INFO("Reachability matrix calculating...");
        reach(planner, 1, 1, 50);
        ROS_INFO("Reachability Grid Calculated in: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);

        // Choose a new goal point if previous point is no longer a frontier
        if (!planner.frontier[planner.xyz_index3(goal)] || !planner.updatePath(goal)) {
          while (!goalFound){
            // Find frontiers
            findFrontier(planner, frontierList, frontierCost);

            // If there are no frontiers available, head to the entrance
            if (frontierCost[4] >= 1e5) findEntrance(planner, frontierList, frontierCost);

            // Write a new frontier goal location for publishing
            frontierGoal.point.x = frontierList[12];
            frontierGoal.point.y = frontierList[13];
            frontierGoal.point.z = frontierList[14];
            for (int i=0; i<3; i++) goal[i] = frontierList[12+i];

            // Write a new frontier goal path for publishing
            goalPose.header.stamp = ros::Time::now();
            goalPose.pose.position.x = frontierList[12];
            goalPose.pose.position.y = frontierList[13];
            goalPose.pose.position.z = frontierList[14];

            // Find a path to the goal point
            goalFound = planner.updatePath(goal);
          }
        }

        for (int i=0; i<5; i++) ROS_INFO("Frontier Position: [x: %f, y: %f, z: %f, cost: %f]", frontierList[3*i], frontierList[3*i+1], frontierList[3*i+2], frontierCost[i]);

        // Publish path, goal point, and goal point only path
        pub1.publish(frontierGoal);
        pub4.publish(goalPose);
        ROS_INFO("Goal point published!");

        // Output and publish path
        pub2.publish(planner.pathmsg);
        ROS_INFO("Path to goal published!");
        goalFound = 0;

        // Output reach matrix to .csv
        // if (spins < 2) {
        //   // Output the frontier
        //   FILE * myfile;
        //   myfile = fopen("frontier.csv", "w");
        //   npixels = planner.esdf.size[0]*planner.esdf.size[1]*planner.esdf.size[2];
        //   fprintf(myfile, "%d, %d, %d, %f\n", planner.esdf.size[0], planner.esdf.size[1], planner.esdf.size[2], planner.voxel_size);
        //   for (int i=0; i<npixels-1; i++) fprintf(myfile, "%d, ", planner.frontier[i]);
        //   fprintf(myfile, "%d", planner.frontier[npixels]);
        //   fclose(myfile);

        //   // Output the reachability grid
        //   myfile = fopen("reach.csv", "w");
        //   fprintf(myfile, "%d, %d, %d, %f\n", planner.esdf.size[0], planner.esdf.size[1], planner.esdf.size[2], planner.voxel_size);
        //   for (int i=0; i<npixels-1; i++) fprintf(myfile, "%f, ", planner.reach[i]);
        //   fprintf(myfile, "%f", planner.reach[npixels]);
        //   fclose(myfile);

        spins++;
        // }
      }
    }
  }

  return 0;
}