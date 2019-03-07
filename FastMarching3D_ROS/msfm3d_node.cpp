// g++ msfm3d_node.cpp -g -o msfm3d_node.o -I /opt/ros/melodic/include -I /usr/include/c++/7.3.0 -I /home/andrew/catkin_ws/devel/include -I /home/andrew/catkin_ws/src/octomap_msgs/include -I /usr/include/pcl-1.8 -I /usr/include/eigen3 -L /usr/lib/x86_64-linux-gnu -L /home/andrew/catkin_ws/devel/lib -L /opt/ros/melodic/lib -Wl,-rpath,opt/ros/melodic/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization -loctomap -lboost_system -lpcl_common -lpcl_io -lpcl_filters -lpcl_features -lpcl_kdtree -lpcl_sample_consensus -lpcl_segmentation


#include <math.h>
// ROS libraries
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
// Octomap libaries
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
// pcl_ros and pcl libraries
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
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
    std::string frame = "world";
    bool ground = 0;
    bool receivedPosition = 0;
    bool receivedMap = 0;
    bool newMap = 0;
    bool esdf_or_octomap = 0; // Boolean to use an esdf PointCloud2 or an Octomap as input
    float voxel_size, bubble_radius = 1.5; // map voxel size, and bubble radius
    float position[3] = {69.0, 420.0, 1337.0}; // robot position
    float euler[3]; // robot orientation in euler angles
    float R[9]; // Rotation matrix
    float origin[3]; // location in xyz coordinates where the robot entered the environment
    double * reach; // reachability grid (output from reach())
    bool * frontier;
    pcl::PointCloud<pcl::PointXYZ>::Ptr frontierCloud; // Frontier PCL
    bool * entrance;
    int frontier_size = 0;
    int link_id = -1;
    sensor_msgs::PointCloud2 PC2msg;
    nav_msgs::Path pathmsg;
    visualization_msgs::MarkerArray frontiermsg;
    octomap::OcTree* tree; // OcTree object for holding Octomap

    struct ESDF {
      double * data; // esdf matrix pointer
      bool * seen; // seen matrix pointer
      int size[3]; // number of elements in each dimension
      float max[4]; // max and min values in each dimension
      float min[4];
    };
    struct orientation {
      float w, x, y, z;
    };
    struct boundary {
      bool set = 0;
      float xmin, xmax, ymin, ymax, zmin, zmax;
    };

    orientation q; // robot orientation in quaternions
    ESDF esdf; // ESDF struct object
    boundary bounds; // xyz boundary of possible goal locations

    void callback(sensor_msgs::PointCloud2 msg); // Subscriber callback function for PC2 msg (ESDF)
    void callback_Octomap(const octomap_msgs::Octomap::ConstPtr msg); // Subscriber callback function for Octomap msg
    void callback_position(const nav_msgs::Odometry msg); // Subscriber callback for robot position
    void parsePointCloud(); // Function to parse pointCloud2 into an esdf format that msfm3d can use
    int xyz_index3(const float point[3]);
    void index3_xyz(const int index, float point[3]);
    void getEuler(); // Updates euler array given the current quaternion values
    void getRotationMatrix(); // Updates Rotation Matrix given the current quaternion values
    void updatePath(const float goal[3]); // Updates the path vector from the goal frontier point to the robot location
    void updateFrontierMsg(); // Updates the frontiermsg MarkerArray with the frontier matrix for publishing
    void clusterFrontier(const float radius); // Clusters the frontier pointCloud with euclidean distance within a radius
    bool inBoundary(const float point[3]); // Checks if a point is inside the planner boundaries
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

void Msfm3d::getRotationMatrix()
{
  if (receivedPosition) {
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
  // delete abstree;
  delete mytree;

  // newMap = 1;
  ROS_INFO("OctoMap message received!");
}

void Msfm3d::callback(sensor_msgs::PointCloud2 msg)
{
  ROS_INFO("Getting ESDF PointCloud2...");
  if (!receivedMap) receivedMap = 1;
  PC2msg = msg;
  newMap = 1;
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
    // esdf.data[index] = (double)(xyzis[i+3]);
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

void Msfm3d::clusterFrontier(const float radius) {
  ROS_INFO("frontierCloud before filtering has: %d data points.", (int)frontierCloud->points.size());

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  // pcl::VoxelGrid<pcl::PointXYZ> vg;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  // vg.setInputCloud (cloud);
  // vg.setLeafSize (0.01f, 0.01f, 0.01f);
  // vg.filter (*cloud_filtered);
  // std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.2);

  int i=0, nr_points = (int) frontierCloud->points.size ();
  while (frontierCloud->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (frontierCloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (frontierCloud);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *frontierCloud = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (frontierCloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.2); // 0.2m
  ec.setMinClusterSize (15);
  ec.setMaxClusterSize (100);
  ec.setSearchMethod (tree);
  ec.setInputCloud (frontierCloud);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (frontierCloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }
}

void Msfm3d::updatePath(const float goal[3]){
  int npixels = esdf.size[0]*esdf.size[1]*esdf.size[2];
  int ijk000[3]; // i,j,k indices of corner[0]
  float xyz000[3]; // x,y,z, coordinates of corner[0];
  int corner[8]; // indices of the corners of the 8 closest voxels to the current point
  int neighbor[6]; // neighbor voxels to the current voxel
  float point[3]; // current point x,y,z coordinates
  float query[3]; // intermediate x,y,z coordinates for another operation
  float grad[3]; // gradient at the current point
  float gradcorner[24]; // corner voxel gradients
  float grad_norm; // norm of the gradient vector
  float dist_robot2path = 10*voxel_size; // distance of the robot to the path
  float position2D[2];
  float point2D[2];
  std::vector<float> path;

  // 3D Interpolation intermediate values from https://en.wikipedia.org/wiki/Trilinear_interpolation
  float xd, yd, zd, c00, c01, c10, c11, c0, c1; // 3D linear interpolation weights.
  float step = voxel_size/4.0;

  // Path message for ROS
  nav_msgs::Path newpathmsg;
  geometry_msgs::PoseStamped pose;
  newpathmsg.header.frame_id = frame;

  // ROS_INFO("Variables declared.");
  // Clear the path vector to prep for adding new elements.
  for (int i=0; i<3; i++){
    point[i] = goal[i];
    path.push_back(point[i]);
  }

  // Run loop until the path is within a voxel of the robot.
  while ((dist_robot2path > 2.0*voxel_size) && (path.size() < 30000)) {
    // Find the corner indices to the current point
    // for (int i=0; i<3; i++) ijk000[i] = floor((point[i] - esdf.min[i])/voxel_size);
    // corner[0] = mindex3(ijk000[0], ijk000[1], ijk000[2], esdf.size[0], esdf.size[1]);
    // corner[1] = mindex3(ijk000[0] + 1, ijk000[1], ijk000[2], esdf.size[0], esdf.size[1]);
    // corner[2] = mindex3(ijk000[0], ijk000[1] + 1, ijk000[2], esdf.size[0], esdf.size[1]);
    // corner[3] = mindex3(ijk000[0] + 1, ijk000[1] + 1, ijk000[2], esdf.size[0], esdf.size[1]);
    // corner[4] = mindex3(ijk000[0], ijk000[1], ijk000[2] + 1, esdf.size[0], esdf.size[1]);
    // corner[5] = mindex3(ijk000[0] + 1, ijk000[1], ijk000[2] + 1, esdf.size[0], esdf.size[1]);
    // corner[6] = mindex3(ijk000[0], ijk000[1] + 1, ijk000[2] + 1, esdf.size[0], esdf.size[1]);
    // corner[7] = mindex3(ijk000[0] + 1, ijk000[1] + 1, ijk000[2] + 1, esdf.size[0], esdf.size[1]);

    // ROS_INFO("Corner indices found");
    // Compute the gradients at each corner point.
    // for (int i=0; i<8; i++) {
    //   neighbor[0] = corner[i] - 1; // i-1
    //   neighbor[1] = corner[i] + 1; // i+1
    //   neighbor[2] = corner[i] - esdf.size[0]; // j-1
    //   neighbor[3] = corner[i] + esdf.size[0]; // j+1
    //   neighbor[4] = corner[i] - esdf.size[0]*esdf.size[1]; // k-1
    //   neighbor[5] = corner[i] + esdf.size[0]*esdf.size[1]; // k+1
    //   // ROS_INFO("Neighbor indices found of corner point %d", i);
    //   for (int j=0; j<3; j++) gradcorner[3*i+j] = 0.5*(float)(reach[neighbor[2*j]] - reach[neighbor[2*j+1]]); // Central Difference Operator (Try Sobel if this is bad)
    // }
	
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
      grad[j] = 0.5*(float)(reach[neighbor[2*j]] - reach[neighbor[2*j+1]]); // Central Difference Operator (Try Sobel if this is bad)
    }
    // ROS_INFO("gradients found at each corner point.");
    // Linearly interpolate in 3D to find the gradient at the current point.
    // index3_xyz(corner[0], xyz000);
    // xd = (point[0] - xyz000[0])/voxel_size;
    // yd = (point[1] - xyz000[1])/voxel_size;
    // zd = (point[2] - xyz000[2])/voxel_size;
    // for (int i=0; i<3; i++){
    //   c00 = (1.0-xd)*gradcorner[i] + xd*gradcorner[3+i];
    //   c01 = (1.0-xd)*gradcorner[12+i] + xd*gradcorner[15+i];
    //   c10 = (1.0-xd)*gradcorner[6+i] + xd*gradcorner[9+i];
    //   c11 = (1.0-xd)*gradcorner[18+i] + xd*gradcorner[21+i];
    //   c0 = (1.0-yd)*c00 + yd*c10;
    //   c1 = (1.0-yd)*c01 + yd*c11;
    //   grad[i] = (1.0-zd)*c0 + zd*c1;
    // }

    // Normalize the size of the gradient vector if it is too large
    grad_norm = std::sqrt(grad[0]*grad[0] + grad[1]*grad[1] + grad[2]*grad[2]);
    if (grad_norm > 0.25){
      for (int i=0; i<3; i++) grad[i] = std::sqrt(0.25)*grad[i]/grad_norm;
    }
    if (grad_norm < 0.05){
      for (int i=0; i<3; i++) grad[i] = std::sqrt(0.05)*grad[i]/grad_norm;
    }

    // ROS_INFO("3D Interpolation performed.");
    // ROS_INFO("Gradients: [%f, %f, %f]", grad[0], grad[1], grad[2]);
    // Update point and add to path
    for (int i=0; i<3; i++) {
      point[i] = point[i] + step*grad[i];
      path.push_back(point[i]);
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
  }

  // Add path vector to path message for plotting in rviz
  ROS_INFO("Path finished of length %d", (int)path.size());
  for (int i=(path.size()-3); i>=0; i=i-3){
    pose.header.frame_id = frame;
    pose.pose.position.x = path[i];
    pose.pose.position.y = path[i+1];
    pose.pose.position.z = path[i+2];
    newpathmsg.poses.push_back(pose);
  }
  pathmsg = newpathmsg;
}

void Msfm3d::updateFrontierMsg() {
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray newMarkerArray;
  int npixels = esdf.size[0]*esdf.size[1]*esdf.size[2];
  int count = 1;
  float point[3];
  int diff;
  for (int i=0; i<npixels; i++){
    if (frontier[i]){
      index3_xyz(i, point);
      marker.header.frame_id = frame;
      marker.header.stamp = ros::Time();
      marker.id = count;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = point[0];
      marker.pose.position.y = point[1];
      marker.pose.position.z = point[2];
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = voxel_size;
      marker.scale.y = voxel_size;
      marker.scale.z = voxel_size;
      marker.color.a = 0.75; // Alpha value
      marker.color.r = 0.0; // red
      marker.color.g = 1.0; // green
      marker.color.b = 0.0; // blue
      newMarkerArray.markers.push_back(marker);
      count++;
    }
  }
  if (frontier_size < count) {
    diff = count - frontier_size;
    for (int i=0; i < diff; i++){
      marker.header.frame_id = frame;
      marker.header.stamp = ros::Time();
      marker.id = frontier_size + i + 1;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::DELETE;
      newMarkerArray.markers.push_back(marker);
    }
  }
  frontier_size = count;
  frontiermsg = newMarkerArray;
}


void updateFrontier(Msfm3d& planner){
  ROS_INFO("Beginning Frontier update step...");
  int npixels = planner.esdf.size[0]*planner.esdf.size[1]*planner.esdf.size[2];
  delete[] planner.frontier;
  ROS_INFO("Previous frontier deleted.  Allocating new frontier of size %d...", npixels);
  planner.frontier = NULL;
  planner.frontier = new bool [npixels] { }; // Initialize the frontier array as size npixels with all values false.
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
        for (int j=0; j<5; j++) {
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
        if (abs(planner.position[2] - point[2]) >= 2*planner.voxel_size) frontier = 0;

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
  planner.esdf_or_octomap = 0; // Use an octomap message
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
  planner.voxel_size = 0.2;

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
  // ROS_INFO("Subscribing to Occupancy Grid...");
  // ros::Subscriber sub1 = n.subscribe("/octomap_binary", 1, &Msfm3d::callback_Octomap, &planner);  
  // }
  // else {
  ROS_INFO("Subscribing to ESDF or TSDF PointCloud2...");
  ros::Subscriber sub1 = n.subscribe("/X1/voxblox_node/tsdf_pointcloud", 1, &Msfm3d::callback, &planner);
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
  ros::Publisher pub3 = n.advertise<visualization_msgs::MarkerArray>("/X1/frontier", 100);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  int i = 0;
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
    planner.parsePointCloud();
    // Heartbeat status update
    if (planner.receivedPosition){
      ROS_INFO("X1 Position: [x: %f, y: %f, z: %f]", planner.position[0], planner.position[1], planner.position[2]);
      i = planner.xyz_index3(planner.position);
      ROS_INFO("Index at Position: %d", i);
      if (planner.receivedMap){
        ROS_INFO("ESDF at Position: %f", planner.esdf.data[i]);

        // Find frontier cells and add them to planner.frontier for output to file.
        // Publish frontiers as MarkerArray
        updateFrontier(planner);
        planner.updateFrontierMsg();
        pub3.publish(planner.frontiermsg);
        ROS_INFO("Frontier MarkerArray published!");
        
        // Call msfm3d function
        tStart = clock();
        ROS_INFO("Reachability matrix calculating...");
        reach(planner, 1, 1, 50);
        ROS_INFO("Reachability Grid Calculated in: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);

        // Choose a new goal point if previous point is no longer a frontier
        if (!planner.frontier[planner.xyz_index3(goal)]){
          // Find frontiers
          findFrontier(planner, frontierList, frontierCost);
          for (int i=0; i<5; i++) ROS_INFO("Frontier Position: [x: %f, y: %f, z: %f, cost: %f]", frontierList[3*i], frontierList[3*i+1], frontierList[3*i+2], frontierCost[i]);

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
        }
        // Publish path, goal point, and goal point only path
        pub1.publish(frontierGoal);
        pub4.publish(goalPose);
        ROS_INFO("Goal point published!");

        // Output and publish path
        planner.updatePath(goal);
        pub2.publish(planner.pathmsg);
        ROS_INFO("Path to goal published!");

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