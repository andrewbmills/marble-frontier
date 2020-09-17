#include <vector>
#include "math.h"
#include <iostream>
#include <random.h>
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
#include <gain.h>

// Group relative pose sampling queue

// Try distributing points with Fibonacci spacing (https://stackoverflow.com/questions/9600801/evenly-distributing-n-points-on-a-sphere/44164075#44164075)

struct Group
{
  pcl::PointNormal centroid;
  pcl::PointIndices indices;
};

bool filterCloudRadius(const float radius, const pcl::PointXYZ point, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers, 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers)
{
  // filterRadius returns the indices (inside inliers->indices) in cloud that are within euclidean distance r of point (x,y,z).

  // Find all indices within r of point (x,y,z)
  float distance_squared;
  float delta_x;
  float delta_y;
  float delta_z;
  float radius_squared = radius*radius;
  for (int i = 0; i<(int)cloud->points.size(); i++) { // This line is vague, consider modifying with a more explicit range of values
    pcl::PointXYZ query = cloud->points[i];
    delta_x = point.x - query.x;
    delta_y = point.y - query.y;
    delta_z = point.z - query.z;
    distance_squared = (delta_x*delta_x) + (delta_y*delta_y) + (delta_z*delta_z);
    if (distance_squared < radius_squared) {
      inliers->indices.push_back(i);  // Add indices that are within the radius to inliers
      cloudInliers->points.push_back(query);
    }
  }

  // Return false if there are no points in cloud within r of point (x,y,z)
  if (inliers->indices.size() > 0)
    return 1;
  else
    return 0;
}

bool filterCloudRadius(const float radius, const pcl::PointNormal point, const pcl::PointCloud<pcl::PointNormal>::Ptr cloud, pcl::PointIndices::Ptr inliers, 
pcl::PointCloud<pcl::PointNormal>::Ptr cloudInliers)
{
  // filterRadius returns the indices (inside inliers->indices) in cloud that are within euclidean distance r of point (x,y,z).

  // Find all indices within r of point (x,y,z)
  float distance_squared;
  float delta_x;
  float delta_y;
  float delta_z;
  float radius_squared = radius*radius;
  for (int i = 0; i<(int)cloud->points.size(); i++) { // This line is vague, consider modifying with a more explicit range of values
    pcl::PointNormal query = cloud->points[i];
    delta_x = point.x - query.x;
    delta_y = point.y - query.y;
    delta_z = point.z - query.z;
    distance_squared = (delta_x*delta_x) + (delta_y*delta_y) + (delta_z*delta_z);
    if (distance_squared < radius_squared) {
      inliers->indices.push_back(i);  // Add indices that are within the radius to inliers
      cloudInliers->points.push_back(query);
    }
  }

  // Return false if there are no points in cloud within r of point (x,y,z)
  if (inliers->indices.size() > 0)
    return 1;
  else
    return 0;
}

void greedyGrouping(const float radius, const pcl::PointCloud<pcl::PointNormal>::Ptr cloud, std::vector<pcl::PointIndices> clusterIndices,
std::vector<Group> &groups, const bool print2File)
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
  //    3) clusterCount++, GoTo 1
  // Inputs:
  //    - clusterIndices (std::vector<pcl::PointIndices>)
  //    - cloud (pcl::PointCloud<pcl::PointNormal>::Ptr)
  //    - radius (float)
  // Outputs:
  //    - groups (std::vector<Group>)
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
  groups.clear();

  ROS_INFO("Beginning greedy grouping of %d clusters with sizes:", (int)clusterIndices.size());
  for (int i = 0; i < clusterIndices.size(); i++) {
  	std::cout << clusterIndices[i].indices.size() << " ";
  }
  std::cout << std::endl;

  // Loop through all of the frontier clusters
  for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {

    // Initialize a PointIndices pointer of all the ungrouped points in the current cluster
    pcl::PointIndices::Ptr ungrouped(new pcl::PointIndices);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
    	ungrouped->indices.push_back(*pit);
    }

    // Go until every member of ungrouped is in a greedy group
    while (ungrouped->indices.size() > 0) {
      // Reset/Update ungrouped PointCloud
      pcl::PointCloud<pcl::PointNormal>::Ptr ungroupedCloud(new pcl::PointCloud<pcl::PointNormal>);
      pcl::ExtractIndices<pcl::PointNormal> extract;
      extract.setInputCloud(cloud);
      extract.setIndices(ungrouped);
      extract.filter(*ungroupedCloud);
      // ROS_INFO("Initializing ungrouped_cloud for group %d containing %d points.", groupCount, (int)ungrouped_cloud->points.size());

      // Sample a random index from ungrouped
      std::uniform_int_distribution<int> dist(0, (int)ungrouped->indices.size()-1);
      int sampleId = ungrouped->indices[dist(mt)];
      pcl::PointNormal samplePoint = cloud->points[sampleId];
      // ROS_INFO("Sampled index %d which is located at (%f, %f, %f).", sample_id, sample_point.x, sample_point.y, sample_point.z);

      // Find all the voxels in ungrouped_cloud that are within r of sample_point
      pcl::PointIndices::Ptr group(new pcl::PointIndices);
      pcl::PointCloud<pcl::PointNormal>::Ptr groupCloud(new pcl::PointCloud<pcl::PointNormal>);
      filterCloudRadius(radius, samplePoint, ungroupedCloud, group, groupCloud);

      // Removed grouped indices from ungrouped  (This command removes all members of ungrouped->indices that have a value of -1)
      ungrouped->indices.erase(std::remove(ungrouped->indices.begin(), ungrouped->indices.end(), -1), ungrouped->indices.end());

      // Add point indices and center to groups
      Group newGroup;
      newGroup.centroid = samplePoint;
      for (std::vector<int>::const_iterator pit = group->indices.begin(); pit != group->indices.end(); ++pit) newGroup.indices.indices.push_back(*pit);
      groups.push_back(newGroup);
      groupCount++;
    }
    clusterCount++;
  }
  ROS_INFO("%d groups generated from the frontier clusters.", groupCount);
}

std::vector<View> SampleGoals()
{
  std::vector<View> views;
  return views;
}