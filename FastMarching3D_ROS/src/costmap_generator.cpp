// This node fuses occupancy grid Octomaps, ESDF/TSDF PointCloud2's, and traversability Octomaps into one map for robust planning and control.
// C++ Standard Libraries
#include <math.h>
// ROS libraries
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// pcl
#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>
// Octomap libaries
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
// pcl ROS
#include <pcl_conversions/pcl_conversions.h>


#define PI 3.14159265

void index3_xyz(const int index, double point[3], double min[3], int size[3], double voxelSize)
{
  // x+y*sizx+z*sizx*sizy
  point[2] = min[2] + (index/(size[1]*size[0]))*voxelSize;
  point[1] = min[1] + ((index % (size[1]*size[0]))/size[0])*voxelSize;
  point[0] = min[0] + ((index % (size[1]*size[0])) % size[0])*voxelSize;
}

int xyz_index3(const double point[3], double min[3], int size[3], double voxelSize)
{
  int ind[3];
  for (int i=0; i<3; i++) ind[i] = round((point[i]-min[i])/voxelSize);
  return (ind[0] + ind[1]*size[0] + ind[2]*size[0]*size[1]);
}

struct Point {
  double x, y, z;
};

class Costmap_fuser
{
  public:
    // Constructor
    Costmap_fuser(double resolution):
    esdfCloud(new pcl::PointCloud<pcl::PointXYZI>)
    {
      occupancyTree = new octomap::OcTree(resolution);
      traverseTree = new octomap::OcTree(resolution);
      voxelSize = resolution;
    }

    // Property Definitions
    double voxelSize;

    // Storing pcl2 message for publishing
    sensor_msgs::PointCloud2 fusedMsg;

    // Holder arrays
    octomap::OcTree* occupancyTree; // OcTree object for holding occupancy Octomap
    octomap::OcTree* traverseTree; // OcTree object for holding traversability Octomap
    pcl::PointCloud<pcl::PointXYZI>::Ptr esdfCloud;
    Point esdfMin;
    Point esdfMax;

    // Subscribed Message update booleans
    bool updatedOccupancy = false;
    bool updatedTraverse = false;
    bool updatedEsdf = false;
    bool updatedOccupancyFirst = false;
    bool updatedTraverseFirst = false;
    bool updatedEsdfFirst = false;

    // World frame
    std::string fixedFrameId = "world";

    float untraversableDistance = 0.01;
    float octomapFreeDistance = 3.0*voxelSize;

    // Only publish map once it's been populated
    bool pubMap = false;

    // Method Definitions
    void callback_cloud(const sensor_msgs::PointCloud2ConstPtr& msg);
    void callback_octomap(const octomap_msgs::Octomap::ConstPtr msg);
    void callback_traverse(const octomap_msgs::Octomap::ConstPtr msg);
    void fuse_maps();
};

void Costmap_fuser::callback_octomap(const octomap_msgs::Octomap::ConstPtr msg)
{
  ROS_INFO("Occupancy octomap callback called");
  if (msg->data.size() == 0) return;
  delete occupancyTree;
  occupancyTree = new octomap::OcTree(msg->resolution);
  occupancyTree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
  updatedOccupancy = true;
  updatedOccupancyFirst = true;
  ROS_INFO("Occupancy octomap callback success!");
  return;
}

void Costmap_fuser::callback_traverse(const octomap_msgs::Octomap::ConstPtr msg)
{
  ROS_INFO("Traversability octomap callback called");
  if (msg->data.size() == 0) return;
  delete traverseTree;
  traverseTree = new octomap::OcTree(msg->resolution);
  traverseTree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
  updatedTraverse = true;
  updatedTraverseFirst = true;
  ROS_INFO("Traversability octomap callback success!");
  return;
}

void Costmap_fuser::callback_cloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  ROS_INFO("ESDF callback called");
  if (msg->data.size() == 0) return;
  // Convert from ROS PC2 msg to PCL object
  pcl::PointCloud<pcl::PointXYZI>::Ptr ESDFCloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *esdfCloud);

  // Get the xyz extents of the PCL by running one loop through the data
  for (int i=0; i<esdfCloud->points.size(); i++) {
    pcl::PointXYZI query = esdfCloud->points[i];
    if (i == 0) {
      esdfMin.x = (double)query.x; esdfMin.y = (double)query.y; esdfMin.z = (double)query.z;
      esdfMax.x = (double)query.x; esdfMax.y = (double)query.y; esdfMax.z = (double)query.z;
      continue;
    }
    if (query.x < esdfMin.x) esdfMin.x = (double)query.x;
    if (query.y < esdfMin.y) esdfMin.y = (double)query.y;
    if (query.z < esdfMin.z) esdfMin.z = (double)query.z;
    if (query.x > esdfMax.x) esdfMax.x = (double)query.x;
    if (query.y > esdfMax.y) esdfMax.y = (double)query.y;
    if (query.z > esdfMax.z) esdfMax.z = (double)query.z;
  }
  updatedEsdf = true;
  updatedEsdfFirst = true;
  ROS_INFO("ESDF callback success!");
  return;
}

void Costmap_fuser::fuse_maps()
{
  ROS_INFO("Map Fusion Started");
  // Set updates to false and exit if no messages have been updated
  if (updatedEsdf || updatedTraverse || updatedOccupancy) {
    updatedEsdf = false;
    updatedTraverse = false;
    updatedOccupancy = false;
  } else {
    return;
  }

  ROS_INFO("Getting largest map extents...");
  // Get the largest extent in any dimension of each map representation
  Point mapMin;
  Point mapMax;
  if (updatedEsdfFirst) {
    mapMin = esdfMin;
    mapMax = esdfMax;
    ROS_INFO("ESDF, Map mins: (%0.1f, %0.1f, %0.1f), Map maxes: (%0.1f, %0.1f, %0.1f)", mapMin.x, mapMin.y, mapMin.z, mapMax.x, mapMax.y, mapMax.z);
  } else if (updatedOccupancyFirst) {
    occupancyTree->getMetricMin(mapMin.x, mapMin.y, mapMin.z);
    occupancyTree->getMetricMax(mapMax.x, mapMax.y, mapMax.z);
    ROS_INFO("Occupancy, Map mins: (%0.1f, %0.1f, %0.1f), Map maxes: (%0.1f, %0.1f, %0.1f)", mapMin.x, mapMin.y, mapMin.z, mapMax.x, mapMax.y, mapMax.z);
  } else if (updatedTraverseFirst) {
    traverseTree->getMetricMin(mapMin.x, mapMin.y, mapMin.z);
    traverseTree->getMetricMax(mapMax.x, mapMax.y, mapMax.z);
    ROS_INFO("Traverse, Map mins: (%0.1f, %0.1f, %0.1f), Map maxes: (%0.1f, %0.1f, %0.1f)", mapMin.x, mapMin.y, mapMin.z, mapMax.x, mapMax.y, mapMax.z);
  }
  if (updatedEsdfFirst) {
    if (esdfMin.x < mapMin.x) mapMin.x = esdfMin.x;
    if (esdfMin.y < mapMin.y) mapMin.y = esdfMin.y;
    if (esdfMin.z < mapMin.z) mapMin.z = esdfMin.z;
    if (esdfMax.x > mapMax.x) mapMax.x = esdfMax.x;
    if (esdfMax.y > mapMax.y) mapMax.y = esdfMax.y;
    if (esdfMax.z > mapMax.z) mapMax.z = esdfMax.z;
    ROS_INFO("ESDF, Map mins: (%0.1f, %0.1f, %0.1f), Map maxes: (%0.1f, %0.1f, %0.1f)", mapMin.x, mapMin.y, mapMin.z, mapMax.x, mapMax.y, mapMax.z);
  }
  if (updatedOccupancyFirst) {
    Point occupancyMin;
    Point occupancyMax;
    occupancyTree->getMetricMin(occupancyMin.x, occupancyMin.y, occupancyMin.z);
    occupancyTree->getMetricMax(occupancyMax.x, occupancyMax.y, occupancyMax.z);
    occupancyMin.x = occupancyMin.x - 0.5*voxelSize;
    occupancyMin.y = occupancyMin.y - 0.5*voxelSize;
    occupancyMin.z = occupancyMin.z - 0.5*voxelSize;
    occupancyMax.x = occupancyMax.x + 0.5*voxelSize;
    occupancyMax.y = occupancyMax.y + 0.5*voxelSize;
    occupancyMax.z = occupancyMax.z + 0.5*voxelSize;
    if (occupancyMin.x < mapMin.x) mapMin.x = occupancyMin.x;
    if (occupancyMin.y < mapMin.y) mapMin.y = occupancyMin.y;
    if (occupancyMin.z < mapMin.z) mapMin.z = occupancyMin.z;
    if (occupancyMax.x > mapMax.x) mapMax.x = occupancyMax.x;
    if (occupancyMax.y > mapMax.y) mapMax.y = occupancyMax.y;
    if (occupancyMax.z > mapMax.z) mapMax.z = occupancyMax.z;
    ROS_INFO("Occupancy, Map mins: (%0.1f, %0.1f, %0.1f), Map maxes: (%0.1f, %0.1f, %0.1f)", mapMin.x, mapMin.y, mapMin.z, mapMax.x, mapMax.y, mapMax.z);
  }
  if (updatedTraverseFirst) {
    Point traverseMin;
    Point traverseMax;
    traverseTree->getMetricMin(traverseMin.x, traverseMin.y, traverseMin.z);
    traverseTree->getMetricMax(traverseMax.x, traverseMax.y, traverseMax.z);
    traverseMin.x = traverseMin.x - 0.5*voxelSize;
    traverseMin.y = traverseMin.y - 0.5*voxelSize;
    traverseMin.z = traverseMin.z - 0.5*voxelSize;
    traverseMax.x = traverseMax.x + 0.5*voxelSize;
    traverseMax.y = traverseMax.y + 0.5*voxelSize;
    traverseMax.z = traverseMax.z + 0.5*voxelSize;
    if (traverseMin.x < mapMin.x) mapMin.x = traverseMin.x;
    if (traverseMin.y < mapMin.y) mapMin.y = traverseMin.y;
    if (traverseMin.z < mapMin.z) mapMin.z = traverseMin.z;
    if (traverseMax.x > mapMax.x) mapMax.x = traverseMax.x;
    if (traverseMax.y > mapMax.y) mapMax.y = traverseMax.y;
    if (traverseMax.z > mapMax.z) mapMax.z = traverseMax.z;
    ROS_INFO("Traverse, Map mins: (%0.1f, %0.1f, %0.1f), Map maxes: (%0.1f, %0.1f, %0.1f)", mapMin.x, mapMin.y, mapMin.z, mapMax.x, mapMax.y, mapMax.z);
  }

  ROS_INFO("Allocating fused map holder vector memory...");
  // Allocate a vector for holding all of the spacial data
  // int mapSize[3] = {(int)(round((mapMax.x - mapMin.x)/voxelSize) + 1.0), (int)(round((mapMax.y - mapMin.y)/voxelSize) + 1.0), (int)(round((mapMax.z - mapMin.z)/voxelSize) + 1.0)};
  int mapSize[3];
  mapSize[0] = round((mapMax.x - mapMin.x)/voxelSize) + 1;
  mapSize[1] = round((mapMax.y - mapMin.y)/voxelSize) + 1;
  mapSize[2] = round((mapMax.z - mapMin.z)/voxelSize) + 1;
  ROS_INFO("Allocating fused map holder vector memory for arrays of size (%d, %d, %d)", mapSize[0], mapSize[1], mapSize[2]);
  std::vector<float> fusedMap(mapSize[0]*mapSize[1]*mapSize[2], (float)0.0);
  std::vector<bool> fusedSeenMap(mapSize[0]*mapSize[1]*mapSize[2], false);
  std::vector<pcl::PointXYZ> fusedPointMap(mapSize[0]*mapSize[1]*mapSize[2]);
  double mapMinArray[3] = {mapMin.x, mapMin.y, mapMin.z};

  ROS_INFO("Reading ESDF values into fused map...");
  // Add traversable ESDF values to the fusedMap and fusedSeenMap vectors
  if (updatedEsdfFirst) {
    if (updatedTraverseFirst) {
      for (int i=0; i<esdfCloud->points.size(); i++) {
        pcl::PointXYZI query = esdfCloud->points[i];
        double point[3] = {(double)query.x, (double)query.y, (double)query.z};
        int id = xyz_index3(point, mapMinArray, mapSize, voxelSize);
        // ROS_INFO("ESDF point at (%0.1f, %0.1f, %0.1f) with id: %d", query.x, query.y, query.z, id);
        // ROS_INFO("Querying Octree...");
        octomap::OcTreeNode* traverseNode = traverseTree->search(point[0], point[1], point[2]);
        // ROS_INFO("Octree queried");
        if (traverseNode!= NULL) {
          if (traverseNode->getOccupancy() < 0.2) {
            // ROS_INFO("Node is traversable. with occupancy %0.1f", traverseNode->getOccupancy());
            fusedMap[id] = query.intensity;
          } else {
            // ROS_INFO("Node is not traversable. with occupancy %0.1f", traverseNode->getOccupancy());
            fusedMap[id] = std::min(untraversableDistance, query.intensity);
          }
        } else {
          fusedMap[id] = std::min(untraversableDistance, query.intensity);
        }
        fusedSeenMap[id] = true;
        fusedPointMap[id].x = query.x; fusedPointMap[id].y = query.y; fusedPointMap[id].z = query.z;
      }
    } else { // Add every point's value as is since there is no traversability map
      for (int i=0; i<esdfCloud->points.size(); i++) {
        pcl::PointXYZI query = esdfCloud->points[i];
        double point[3] = {(double)query.x, (double)query.y, (double)query.z};
        int id = xyz_index3(point, mapMinArray, mapSize, voxelSize);
        fusedMap[id] = query.intensity;
        fusedSeenMap[id] = true;
        fusedPointMap[id].x = query.x; fusedPointMap[id].y = query.y; fusedPointMap[id].z = query.z;
      }
    }
  }

  ROS_INFO("Reading Octomap values into fused map...");
  // Iterate through occupancy Octomap and add values where the ESDF doesn't already exist
  if (updatedOccupancyFirst) {
    int lowest_depth = (int)occupancyTree->getTreeDepth();
    for(octomap::OcTree::leaf_iterator it = occupancyTree->begin_leafs(),
         end=occupancyTree->end_leafs(); it!=end; ++it)
    {
      // Get data from node
      int depth = (int)it.getDepth();
      double point[3] = {(double)it.getX(), (double)it.getY(), (double)it.getZ()};
      double size = it.getSize();
      // ROS_INFO("Reading data at node (%0.1f, %0.1f, %0.1f)", (double)it.getX(), (double)it.getY(), (double)it.getZ());
      float value;
      if (it->getOccupancy() >= 0.3) {
        value = 0.0;
        // ROS_INFO("Node occupied");
      } else {
        value = 1.0*octomapFreeDistance;
        // ROS_INFO("Node free");
      }

      int idx;
      float lower_corner[3];
      // Put data into fusedMap
      if (depth == lowest_depth) {
        idx = xyz_index3(point, mapMinArray, mapSize, voxelSize);
        // ROS_INFO("Node has id %d", idx);
        if (!fusedSeenMap[idx]) {
          if (updatedTraverseFirst) {
            // ROS_INFO("Getting traversability node...");
            octomap::OcTreeNode* traverseNode = traverseTree->search(point[0], point[1], point[2]);
            // ROS_INFO("Got traversability node...");
            if (traverseNode != NULL) {
              if (traverseNode->getOccupancy() < 0.2) {
                // ROS_INFO("Voxel is traversable.");
                fusedMap[idx] = value;
              } else {
                // ROS_INFO("Voxel is not traversable.");
                fusedMap[idx] = std::min(untraversableDistance, value);
              }
            } else {
              // ROS_INFO("Voxel is not in traversability map.");
              fusedMap[idx] = std::min(untraversableDistance, value);
            }
          } else {
            // ROS_INFO("Traversability map hasn't been subscribed to.  Voxel assumed traversable.");
            fusedMap[idx] = value;
          }
          fusedSeenMap[idx] = true;
          fusedPointMap[idx].x = point[0]; fusedPointMap[idx].y = point[1]; fusedPointMap[idx].z = point[2];
        }
      } else{ // Fill in all the voxels internal to the leaf
        int width = (int)std::pow(2.0, (double)(lowest_depth-depth));
        // ROS_INFO("Calculating lower corner position");
        for (int j=0; j<3; j++){
          lower_corner[j] = point[j] - size/2.0 + voxelSize/2.0;
        }
        // ROS_INFO("Point (%f, %f, %f) is not at the base depth.  It is %d voxels wide.", point[0], point[1], point[2], width);
        // ROS_INFO("Filling in leaf at depth %d with size %f.  The lower corner is at (%f, %f, %f)", depth, size, lower_corner[0], lower_corner[1], lower_corner[2]);
        for (int j=0; j<width; j++){
          point[0] = lower_corner[0] + j*voxelSize;
          for (int k=0; k<width; k++){
            point[1] = lower_corner[1] + k*voxelSize;
            for (int l=0; l<width; l++){
              point[2] = lower_corner[2] + l*voxelSize;
              idx = xyz_index3(point, mapMinArray, mapSize, voxelSize);
              // ROS_INFO("Node has id %d", idx);
              if (!fusedSeenMap[idx]) {
                // ROS_INFO("node unseen");
                if (updatedTraverseFirst) {
                  // ROS_INFO("Getting traversability node...");
                  octomap::OcTreeNode* traverseNode = traverseTree->search(point[0], point[1], point[2]);
                  // ROS_INFO("Got traversability node...");
                  if (traverseNode != NULL) {
                    if (traverseNode->getOccupancy() < 0.2) {
                      // ROS_INFO("Voxel is traversable.");
                      fusedMap[idx] = value;
                    } else {
                      // ROS_INFO("Voxel is not traversable.");
                      fusedMap[idx] = std::min(untraversableDistance, value);
                    }
                  } else {
                    // ROS_INFO("Voxel is not in traversability map.");
                    fusedMap[idx] = std::min(untraversableDistance, value);
                  }
                } else {
                  // ROS_INFO("Traversability map hasn't been subscribed to.  Voxel assumed traversable.");
                  fusedMap[idx] = value;
                }
                fusedSeenMap[idx] = true;
                fusedPointMap[idx].x = point[0]; fusedPointMap[idx].y = point[1]; fusedPointMap[idx].z = point[2];
              }
            }
          }
        }
      }
    }
  }

  ROS_INFO("Building pointcloud2 message...");
  // Iterate through the now fusedMap and store the (seen) values in a PointCloud object and then convert that object to a PC2 message
  pcl::PointCloud<pcl::PointXYZI>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZI>);
  for (int i=0; i<mapSize[0]*mapSize[1]*mapSize[2]; i++) {
    if (fusedSeenMap[i]) {
      pcl::PointXYZI query;
      query.x = fusedPointMap[i].x;
      query.y = fusedPointMap[i].y;
      query.z = fusedPointMap[i].z;
      query.intensity = fusedMap[i];
      outputCloud->points.push_back(query);
    }
  }
  pcl::toROSMsg(*outputCloud, fusedMsg);
  fusedMsg.header.stamp = ros::Time();
  fusedMsg.header.frame_id = fixedFrameId;
  pubMap = true;
  return;
}

int main(int argc, char **argv)
{
  // Initialize ROS node with name and object instantiation
  ros::init(argc, argv, "map_fusion_planning");
  ros::NodeHandle n;

  double voxelSize;
  n.param("costmap_generator/resolution", voxelSize, (double)0.2);
  Costmap_fuser mapFuser(voxelSize);

  // Declare subscribers and publishers
  ros::Subscriber sub1 = n.subscribe("esdf", 1, &Costmap_fuser::callback_cloud, &mapFuser);
  ros::Subscriber sub2 = n.subscribe("octomap_binary", 1, &Costmap_fuser::callback_octomap, &mapFuser);
  ros::Subscriber sub3 = n.subscribe("traversability_map", 1, &Costmap_fuser::callback_traverse, &mapFuser);
  ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("cost_map", 5);

  // Fixed frame id
  n.param<std::string>("costmap_generator/fixedFrame", mapFuser.fixedFrameId, "world");

  // ESDF value to use for untraversable voxels
  n.param("costmap_generator/untraversableDistance", mapFuser.untraversableDistance, (float)0.01);
  n.param("costmap_generator/octomapFreeDistance", mapFuser.octomapFreeDistance, (float)0.6);

  // Declare and read in the node update rate from the launch file parameters
  double updateRate;
  n.param("costmap_generator/rate", updateRate, (double)10.0); // Hz
  ros::Rate r(updateRate);

  // Run the node until ROS quits
  while (ros::ok())
  {
    r.sleep(); // Node sleeps to update at a rate as close as possible to the updateRate parameter
    ros::spinOnce(); // All subscriber callbacks are called here.
    mapFuser.fuse_maps();
    if (mapFuser.pubMap) pub1.publish(mapFuser.fusedMsg);
  }

  return 0;
}
