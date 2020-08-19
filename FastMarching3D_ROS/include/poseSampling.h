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
#include <gain.h>

// Group relative pose sampling queue

// Try distributing points with Fibonacci spacing (https://stackoverflow.com/questions/9600801/evenly-distributing-n-points-on-a-sphere/44164075#44164075)






void greedyGrouping(const float radius, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const bool print2File)
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


std::vector<View> SampleGoals()
{
  std::vector<View> views;
  return views;
}