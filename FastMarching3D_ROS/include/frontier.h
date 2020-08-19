#include <vector>
#include "math.h"
#include <iostream>
#include <omp.h>
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
// pcl ROS
#include <pcl_conversions/pcl_conversions.h>
// Octomap libaries
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
// local packages
#include <mapGrid3D.h>

struct FrontierVoxel {
  pcl::PointXYZ position;
  pcl::Normal normal;
  int cluster = 0;
  int group = 0;
};

struct SeenOccupiedVoxel {
  bool seen = false;
  bool occupied = false;
};

struct Frontier {
  std::vector<FrontierVoxel> list;
  MapGrid3D<std::pair<bool,int>> map; // Stores whether or not a voxel is a frontier and the vector index of the frontier voxel at each x,y,z location
};

Frontier ConvertPointCloudToFrontier(pcl::PointCloud<pcl::PointXYZ>::Ptr frontierList, MapGrid3D<std::pair<bool,bool>>* seen)
{
  Frontier frontier;
  frontier.map = MapGrid3D<std::pair<bool,int>>(seen->voxelSize, seen->size, seen->minBounds);
  frontier.map.SetAll(std::make_pair(false,-1));
  for (int i=0; i<frontierList->points.size(); i++) {
    FrontierVoxel v;
    v.position = frontierList->points[i];
    frontier.list.push_back(v);
    std::pair<bool,int> vMap;
    vMap.first = true;
    vMap.second = frontier.list.size();
    frontier.map.Query(v.position.x, v.position.y, v.position.z) = vMap;
  }
  return frontier;
}

Frontier ConvertPointCloudToFrontier(pcl::PointCloud<pcl::PointNormal>::Ptr frontierList, MapGrid3D<std::pair<bool,bool>>* seen)
{
  Frontier frontier;
  frontier.map = MapGrid3D<std::pair<bool,int>>(seen->voxelSize, seen->size, seen->minBounds);
  frontier.map.SetAll(std::make_pair(false,-1));
  for (int i=0; i<frontierList->points.size(); i++) {
    pcl::PointNormal p = frontierList->points[i];
    FrontierVoxel v;
    v.position.x = p.x; v.position.y = p.y; v.position.z = p.z;
    v.normal.normal_x = p.normal_x; v.normal.normal_y = p.normal_y; v.normal.normal_z = p.normal_z;
    frontier.list.push_back(v);
    std::pair<bool,int> vMap;
    vMap.first = true;
    vMap.second = frontier.list.size();
    frontier.map.Query(v.position.x, v.position.y, v.position.z) = vMap;
  }
  return frontier;
}

Frontier ConvertPointCloudToFrontier(pcl::PointCloud<pcl::PointNormal>::Ptr frontierList, std::vector<pcl::PointIndices> frontierClusterIndices, MapGrid3D<std::pair<bool,bool>>* seen)
{
  Frontier frontier;
  frontier.map = MapGrid3D<std::pair<bool,int>>(seen->voxelSize, seen->size, seen->minBounds);
  frontier.map.SetAll(std::make_pair(false,-1));
  for (int cluster=0; cluster<frontierClusterIndices.size(); cluster++) {
    for (int i=0; i<frontierClusterIndices[cluster].indices.size(); i++) {
      int idx = frontierClusterIndices[cluster].indices[i];
      pcl::PointNormal p = frontierList->points[idx];
      FrontierVoxel v;
      v.position.x = p.x; v.position.y = p.y; v.position.z = p.z;
      v.normal.normal_x = p.normal_x; v.normal.normal_y = p.normal_y; v.normal.normal_z = p.normal_z;
      v.cluster = cluster;
      frontier.list.push_back(v);
      std::pair<bool,int> vMap;
      vMap.first = true;
      vMap.second = frontier.list.size();
      frontier.map.Query(v.position.x, v.position.y, v.position.z) = vMap;
    }
  }
  return frontier;
}

void FilterFrontierByNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr frontierList, pcl::PointCloud<pcl::PointNormal>::Ptr frontierListNormalFiltered, float normalZMax, float radius)
{
  // Create cloud pointer to store the removed points
  pcl::PointCloud<pcl::PointNormal>::Ptr normalCloud(new pcl::PointCloud<pcl::PointNormal>);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  kdtree->setInputCloud(frontierList);

  // Initialize euclidean cluster extraction object
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
  ne.setSearchMethod(kdtree);
  ne.setInputCloud(frontierList);
  ne.setRadiusSearch(radius);
  ne.compute(*normalCloud);

  // Keep all points with a normal z-component less than normalZMax
  frontierListNormalFiltered->points.clear();
  for (int i=0; i<normalCloud->points.size(); i++) {
    pcl::PointNormal query = normalCloud->points[i];
    if (std::abs(query.normal_z) <= normalZMax) {
      query.x = frontierList->points[i].x;
      query.y = frontierList->points[i].y;
      query.z = frontierList->points[i].z;
      frontierListNormalFiltered->points.push_back(query);
    }
  }
  return;
}

void FilterFrontierByCluster(pcl::PointCloud<pcl::PointNormal>::Ptr frontierList, pcl::PointCloud<pcl::PointNormal>::Ptr frontierListFiltered, 
std::vector<pcl::PointIndices> &frontierClusterIndices, float clusterDistance, int minClusterSize)
{
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree(new pcl::search::KdTree<pcl::PointNormal>);
  kdtree->setInputCloud(frontierList);

  // Extract clusters
  pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;
  ec.setClusterTolerance(clusterDistance); // Clusters must be made of contiguous sections of frontier (within sqrt(2)*voxel_size of each other)
  ec.setMinClusterSize(minClusterSize); // Cluster must be at least 15 voxels in size
  ec.setSearchMethod(kdtree);
  ec.setInputCloud(frontierList);
  ec.extract(frontierClusterIndices);

  // Add clusters to output frontier list
  frontierListFiltered->points.clear();
  for (int cluster=0; cluster<frontierClusterIndices.size(); cluster++) {
    for (int i=0; i<frontierClusterIndices[cluster].indices.size(); i++) {
      int idx = frontierClusterIndices[cluster].indices[i];
      frontierListFiltered->points.push_back(frontierList->points[idx]);
      frontierClusterIndices[cluster].indices[i] = frontierListFiltered->points.size();
    }
  }
  return;
}

void ConvertOctomapToSeenOccGrid(octomap::OcTree* map, MapGrid3D<std::pair<bool,bool>>* seenOccGrid)
{
  // Initialize MapGrid3D object of the appropriate size
  double xMin, yMin, zMin, xMax, yMax, zMax;
  seenOccGrid->voxelSize = map->getResolution();
  map->getMetricMin(xMin, yMin, zMin);
  map->getMetricMax(xMax, yMax, zMax);
  seenOccGrid->minBounds.x = (float)xMin - 1.5*map->getResolution();
  seenOccGrid->minBounds.y = (float)yMin - 1.5*map->getResolution();
  seenOccGrid->minBounds.z = (float)zMin - 1.5*map->getResolution();
  seenOccGrid->maxBounds.x = (float)xMax + 1.5*map->getResolution();
  seenOccGrid->maxBounds.y = (float)yMax + 1.5*map->getResolution();
  seenOccGrid->maxBounds.z = (float)zMax + 1.5*map->getResolution();
  seenOccGrid->size.x = roundf((seenOccGrid->maxBounds.x - seenOccGrid->minBounds.x)/map->getResolution()) + 1;
  seenOccGrid->size.y = roundf((seenOccGrid->maxBounds.y - seenOccGrid->minBounds.y)/map->getResolution()) + 1;
  seenOccGrid->size.z = roundf((seenOccGrid->maxBounds.z - seenOccGrid->minBounds.z)/map->getResolution()) + 1;
  seenOccGrid->_SetMaxBounds();
  seenOccGrid->SetAll(std::make_pair(false,false));

  clock_t tStart = clock();
  for(octomap::OcTree::leaf_iterator it = map->begin_leafs(),
  end=map->end_leafs(); it!=end; ++it) {
    float x = it.getX();
  }
  ROS_INFO("Looping through compressed Octomap took: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  map->expand();

  tStart = clock();
  for(octomap::OcTree::leaf_iterator it = map->begin_leafs(),
  end=map->end_leafs(); it!=end; ++it) {
    bool setSuccessful = seenOccGrid->SetVoxel(it.getX(), it.getY(), it.getZ(), std::make_pair(true, (it->getValue()>0.0)), false);
  }
  ROS_INFO("Copying data from Octomap took: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);

  tStart = clock();
  for(octomap::OcTree::leaf_iterator it = map->begin_leafs(),
  end=map->end_leafs(); it!=end; ++it) {
    float x = it.getX();
  }
  ROS_INFO("Looping through data from Octomap took: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  return;
}

bool CalculateFrontierRawPCL_fast(MapGrid3D<std::pair<bool, bool>>* seenOccGrid, pcl::PointCloud<pcl::PointXYZ>::Ptr frontier)
{
  frontier->points.clear();
  std::vector<int> neighbor_deltas;
  neighbor_deltas.push_back(1);
  neighbor_deltas.push_back(-1);
  neighbor_deltas.push_back(seenOccGrid->size.x);
  neighbor_deltas.push_back(-seenOccGrid->size.x);
  neighbor_deltas.push_back(seenOccGrid->size.x*seenOccGrid->size.y);
  neighbor_deltas.push_back(-seenOccGrid->size.x*seenOccGrid->size.y);
  int total_map_size = seenOccGrid->voxels.size();
  // Check all the map cells for frontier voxels
  for (int i=0; i<seenOccGrid->voxels.size(); i++){
    // Check if the voxel is seen and free (not occupied)
    if (seenOccGrid->voxels[i].first && !seenOccGrid->voxels[i].second) {
      // This section will segfault if any seen and free cells are on the border of the map.
      // Be sure to pad the seenOccGrid with at least one unseen voxel to prevent this.
      for (int neighbor=0; neighbor<6; neighbor++) {
        if (!seenOccGrid->voxels[i+neighbor_deltas[neighbor]].first) {
          Point query = seenOccGrid->_ConvertIndexToPosition(i);
          pcl::PointXYZ p{query.x, query.y, query.z};
          frontier->points.push_back(p);
          break;
        }
      }
    }
  }
}

Frontier CalculateFrontier(octomap::OcTree* map, bool normalFilterOn=true, bool clusteringOn=true, float minNormalZ=0.4, int minClusterSize=50)
{
  clock_t tStart = clock();
  MapGrid3D<std::pair<bool,bool>> seenOcc;
  ConvertOctomapToSeenOccGrid(map, &seenOcc);
  ROS_INFO("Octomap converted to seenOcc GridMap in: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);

  // Determines which voxels are frontier by rule:
  // Frontier Voxel - A seen and free voxel adjacent to an unseen voxel
  tStart = clock();
  pcl::PointCloud<pcl::PointXYZ>::Ptr frontierCloudRaw(new pcl::PointCloud<pcl::PointXYZ>);
  CalculateFrontierRawPCL_fast(&seenOcc, frontierCloudRaw);
  ROS_INFO("Raw frontier computed (fastmode) in: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);


  // Calculate frontier normal vectors and filter according to the z-component
  tStart = clock();
  pcl::PointCloud<pcl::PointNormal>::Ptr frontierCloudFilteredNormals(new pcl::PointCloud<pcl::PointNormal>);
  FilterFrontierByNormal(frontierCloudRaw, frontierCloudFilteredNormals, 2.1*seenOcc.voxelSize, minNormalZ);
  ROS_INFO("Normals filtered in: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);

  // Cluster frontier voxels based on contiguity
  tStart = clock();
  pcl::PointCloud<pcl::PointNormal>::Ptr frontierCloudFilteredClustered(new pcl::PointCloud<pcl::PointNormal>);
  std::vector<pcl::PointIndices> frontierClusterIndices;
  FilterFrontierByCluster(frontierCloudFilteredNormals, frontierCloudFilteredClustered, frontierClusterIndices, 1.5*seenOcc.voxelSize, minClusterSize);
  ROS_INFO("Clusters calculated in: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);

  // Convert PointCloud to Frontier struct
  tStart = clock();
  Frontier frontier = ConvertPointCloudToFrontier(frontierCloudFilteredClustered, frontierClusterIndices, &seenOcc);
  ROS_INFO("PointCloud converted to Frontier struct in: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);

  return frontier;
}

sensor_msgs::PointCloud2 ConvertFrontierToROSMsg(Frontier frontier)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr frontierList(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i=0; i<frontier.list.size(); i++){
    frontierList->points.push_back(frontier.list[i].position);
  }
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*frontierList, msg);
  return msg;
}