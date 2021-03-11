#ifndef POSE_SAMPLING_H
#define POSE_SAMPLING_H

#include <vector>
#include "math.h"
#include <iostream>
#include <random>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
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
#include <frontier.h>
#include <gain.h>

// Fibonacci sphere global lookup tables
extern Eigen::MatrixXf fibonacciSphere50(50,3);

// Group relative pose sampling queue

// Try distributing points with Fibonacci spacing (https://stackoverflow.com/questions/9600801/evenly-distributing-n-points-on-a-sphere/44164075#44164075)

struct Group
{
  pcl::PointXYZLNormal centroid;
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

bool filterCloudRadius(const float radius, const pcl::PointXYZLNormal point, const pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud, pcl::PointIndices::Ptr inliers, 
pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloudInliers)
{
  // filterRadius returns the indices (inside inliers->indices) in cloud that are within euclidean distance r of point (x,y,z).

  // Find all indices within r of point (x,y,z)
  float distance_squared;
  float delta_x;
  float delta_y;
  float delta_z;
  float radius_squared = radius*radius;
  for (int i = 0; i<(int)cloud->points.size(); i++) { // This line is vague, consider modifying with a more explicit range of values
    pcl::PointXYZLNormal query = cloud->points[i];
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

void greedyGrouping(const float radius, const pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud, std::vector<pcl::PointIndices> clusterIndices,
std::vector<Group> &groups)
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
  //    - cloud (pcl::PointCloud<pcl::PointXYZLNormal>::Ptr)
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
      pcl::PointCloud<pcl::PointXYZLNormal>::Ptr ungroupedCloud(new pcl::PointCloud<pcl::PointXYZLNormal>);
      pcl::ExtractIndices<pcl::PointXYZLNormal> extract;
      extract.setInputCloud(cloud);
      extract.setIndices(ungrouped);
      extract.filter(*ungroupedCloud);
      // ROS_INFO("Initializing ungrouped_cloud for group %d containing %d points.", groupCount, (int)ungroupedCloud->points.size());

      // Sample a random index from ungrouped
      std::uniform_int_distribution<int> dist(0, (int)ungrouped->indices.size()-1);
      int sampleId = ungrouped->indices[dist(mt)];
      pcl::PointXYZLNormal samplePoint = cloud->points[sampleId];
      // ROS_INFO("Sampled index %d which is located at (%f, %f, %f).", sampleId, samplePoint.x, samplePoint.y, samplePoint.z);

      // Find all the voxels in ungrouped_cloud that are within r of sample_point
      pcl::PointIndices::Ptr group(new pcl::PointIndices);
      pcl::PointCloud<pcl::PointXYZLNormal>::Ptr groupCloud(new pcl::PointCloud<pcl::PointXYZLNormal>);
      filterCloudRadius(radius, samplePoint, ungroupedCloud, group, groupCloud);

      // Add grouped indices to greedyGroups
      Group newGroup;
      newGroup.centroid = samplePoint;
      for (std::vector<int>::const_iterator git = group->indices.begin(); git != group->indices.end(); ++git) {
        newGroup.indices.indices.push_back(ungrouped->indices[*git]); // Add indices to group
        ungrouped->indices[*git] = -1;
      }
      groups.push_back(newGroup);
      groupCount++;
      // ROS_INFO("Added new group of size %d to groups list.", (int)newGroup.indices.indices.size());
      // Removed grouped indices from ungrouped  (This command removes all members of ungrouped->indices that have a value of -1)
      ungrouped->indices.erase(std::remove(ungrouped->indices.begin(), ungrouped->indices.end(), -1), ungrouped->indices.end());
    }
    clusterCount++;
  }
  ROS_INFO("%d groups generated from the frontier clusters.", groupCount);
  return;
}

bool CheckView(View v, pcl::PointXYZLNormal p, MapGrid3D<float> map, octomap::OcTree* octomap, float d, int sample_number) 
{
  // Check if the view is in the map
  Point v_position = {v.pose.position.x, v.pose.position.y, v.pose.position.z};
  if (!(map._CheckVoxelPositionInBounds(v_position))) {
    // if (sample_number < 3) ROS_INFO("Goal rejected for being outside of the map.");
    return false;
  }

  // Check if the point is within d of obstacle
  if (map.Query(v.pose.position.x, v.pose.position.y, v.pose.position.z) < d) {
    // if (sample_number < 3) ROS_INFO("Goal rejected for being too close to an obstacle.");
    return false;
  }

  octomap::point3d origin(v.pose.position.x, v.pose.position.y, v.pose.position.z);
  float dx = v.source.x - v.pose.position.x;
  float dy = v.source.y - v.pose.position.y;
  float dz = v.source.z - v.pose.position.z;
  // ROS_INFO("Checking if (%0.1f, %0.1f, %0.1f) is occluded", cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
  double radius = std::sqrt(dx*dx + dy*dy + dz*dz);
  // ROS_INFO("Casting ray from (%0.1f, %0.1f, %0.1f) in direction (%0.1f, %0.1f, %0.1f)", origin.x(), origin.y(), origin.z(), dx, dy, dz);
  octomap::point3d direction(dx, dy, dz);
  octomap::point3d stop;
  bool hit = octomap->castRay(origin, direction, stop, false, radius);
  double radius_stop = std::sqrt((stop.x() - origin.x())*(stop.x() - origin.x()) + 
                                (stop.y() - origin.y())*(stop.y() - origin.y()) +
                                (stop.z() - origin.z())*(stop.z() - origin.z()));
  if (hit || ((radius - radius_stop) > 1.5*(octomap->getResolution())) ) {
    // query point is the one that stops the raycast
    // ROS_INFO("Point (%0.1f, %0.1f, %0.1f) is visible!", cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
    return false;
  }


  // Check that line-of-sight from v to p is unoccluded
  // int v_ids[3];
  // v_ids[0] = roundf((v.pose.position.x - map.minBounds.x)/map.voxelSize);
  // v_ids[1] = roundf((v.pose.position.y - map.minBounds.y)/map.voxelSize);
  // v_ids[2] = roundf((v.pose.position.z - map.minBounds.z)/map.voxelSize);
  // int p_ids[3];
  // p_ids[0] = roundf((p.x - map.minBounds.x)/map.voxelSize);
  // p_ids[1] = roundf((p.y - map.minBounds.y)/map.voxelSize);
  // p_ids[2] = roundf((p.z - map.minBounds.z)/map.voxelSize);
  // std::vector<int> lineOfSightIds = Bresenham3D(v_ids[0], v_ids[1], v_ids[2], p_ids[0], p_ids[1], p_ids[2]);
  // for (int i=0; i<lineOfSightIds.size(); i=i+3) {
  //   int id = lineOfSightIds[i] + lineOfSightIds[i+1]*map.size.x + lineOfSightIds[i+2]*map.size.x*map.size.y;
  //   if (map.Query(id) < map.voxelSize*0.8) {
  //     ROS_INFO("Goal rejected because LoS to its source voxel is occluded.");
  //     return false;
  //   }
  // }
  return true;
}

Eigen::Matrix3f RotationMatrixBetweenVectors(Eigen::Vector3f a, Eigen::Vector3f b)
{
  // Derived from https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
  // R will be a rotation matrix such that b = R*a
  Eigen::Matrix3f R;
  a = a/a.norm();
  b = b/b.norm();
  Eigen::Vector3f v = a.cross(b);
  float s = v.norm();
  float c = a.dot(b);
  Eigen::MatrixXf v_x(3,3);
  v_x(0,0) = 0.0; v_x(0,1) = -v(2); v_x(0,2) = v(1);
  v_x(1,0) = v(2); v_x(1,1) = 0.0; v_x(1,2) = -v(0);
  v_x(2,0) = -v(1); v_x(2,1) = v(0); v_x(2,2) = 0.0;
  R = Eigen::Matrix3f::Identity() + v_x + ((v_x*v_x)*(1.0 - c)/(s*s));
  return R;
}

View SampleGoalRandomUniform(pcl::PointXYZLNormal source, SensorFoV sensor)
{
  // RNG
  std::random_device rd;
  std::mt19937 mt(rd());

  // Uniform continuous distributions over spherical coordinates
  std::uniform_real_distribution<float> radius_dist(sensor.rMin, sensor.rMax);
  std::uniform_real_distribution<float> azimuth_dist(0, 2*M_PI);
  std::uniform_real_distribution<float> elevation_dist((M_PI - (M_PI/180.0)*sensor.verticalFoV)/2.0, (M_PI + (M_PI/180.0)*sensor.verticalFoV)/2.0); // Assumes a sensor aligned with the vehicle's orientation

  // Sample in spherical coordinates
  float radius_sample = radius_dist(mt);
  float azimuth_sample = azimuth_dist(mt);
  float elevation_sample = elevation_dist(mt);

  // Generate view
  View v;
  v.pose.position.x = source.x + radius_sample*std::sin(elevation_sample)*std::cos(azimuth_sample);
  v.pose.position.y = source.y + radius_sample*std::sin(elevation_sample)*std::sin(azimuth_sample);
  v.pose.position.z = source.z + radius_sample*std::cos(elevation_sample);
  v.pose.q = euler2Quaternion(M_PI + azimuth_sample, 0.0, 0.0);
  v.pose.R = Quaternion2RotationMatrix(v.pose.q);
  v.source = source;
  return v;
}

std::pair<float, float> Vector2AzimuthElevation(Eigen::Vector3f v)
{
  float az, el;
  az = std::atan2(v(1), v(0));
  el = std::asin(v(1)/v.norm());
  return std::make_pair(az, el);
}

View SampleGoalRandomGaussian(pcl::PointXYZLNormal source, float radius)
{
  // RNG
  std::random_device rd;
  std::mt19937 mt(rd());

  // Uniform continuous distributions over spherical coordinates
  std::normal_distribution<float> azimuth_dist(0, M_PI/6); // 3sigma bounds are 90deg (99% of samples are less than +/-90deg)
  std::normal_distribution<float> elevation_dist(0, M_PI/6); // Assumes a sensor aligned with the vehicle's orientation

  // Get azimuth and elevation of current view from [1;0;0]
  Eigen::Vector3f normal;
  normal << source.normal_x, source.normal_y, source.normal_z;
  std::pair<float,float> azi_ele = Vector2AzimuthElevation(normal);

  // Sample in spherical coordinates
  float azimuth_sample = azimuth_dist(mt) + azi_ele.first;
  float elevation_sample = elevation_dist(mt) + azi_ele.second;

  // Generate view
  View v;
  v.pose.position.x = source.x + radius*std::sin(elevation_sample)*std::cos(azimuth_sample);
  v.pose.position.y = source.y + radius*std::sin(elevation_sample)*std::sin(azimuth_sample);
  v.pose.position.z = source.z + radius*std::cos(elevation_sample);
  v.pose.q = euler2Quaternion(M_PI + azimuth_sample, 0.0, 0.0);
  // v.pose.q = euler2Quaternion(M_PI + azi_ele.first, -azi_ele.second, 0.0);
  v.pose.R = Quaternion2RotationMatrix(v.pose.q);
  v.source = source;
  return v;
}

View SampleGoalFibonacciList(pcl::PointXYZLNormal source, int sample, float radius)
{
  Eigen::Vector3f normal;
  normal << source.normal_x, source.normal_y, source.normal_z;
  Eigen::Vector3f x; // Unit vector along the x-axis (origin of fibonacci spread)
  x << 1.0, 0.0, 0.0;
  Eigen::Matrix3f R = RotationMatrixBetweenVectors(x, normal); // normal = R*x

  // ROS_INFO("R between [1.0, 0.0, 0.0] and [%0.2f, %0.2f, %0.2f] is:", normal(0), normal(1), normal(2));
  // ROS_INFO("[%0.2f, %0.2f, %0.2f;", R(0,0), R(0,1), R(0,2));
  // ROS_INFO("%0.2f, %0.2f, %0.2f;", R(1,0), R(1,1), R(1,2));
  // ROS_INFO("%0.2f, %0.2f, %0.2f]", R(2,0), R(2,1), R(2,2));

  // Get the sample'th row of the fibonacci sphere 
  Eigen::Vector3f v_fib; // vector is a deviation from the x-axis
  v_fib << fibonacciSphere50(sample, 0), fibonacciSphere50(sample, 1), fibonacciSphere50(sample, 2);
  // ROS_INFO("Fibonacci sample #%d is [%0.2f, %0.2f, %0.2f]", sample, v_fib(0), v_fib(1), v_fib(2));
  // Rotate sample into the normal vector frame
  v_fib = radius*(R*v_fib);
  std::pair<float, float> azi_ele = Vector2AzimuthElevation(v_fib);
  // ROS_INFO("Fibonacci sample is [%0.2f, %0.2f, %0.2f] after rotation and scaling", v_fib(0), v_fib(1), v_fib(2));
  // ROS_INFO("Vector has %0.1f deg azimuth and %0.1f deg elevation", azi_ele.first*180.0/M_PI, azi_ele.second*180.0/M_PI);

  // Grabs the ith row of the view relative to source orientation
  View v;
  v.pose.position.x = source.x + v_fib(0);
  v.pose.position.y = source.y + v_fib(1);
  v.pose.position.z = source.z + v_fib(2);
  v.pose.q = euler2Quaternion(M_PI + azi_ele.first, 0.0, 0.0);
  // v.pose.q = euler2Quaternion(M_PI + azi_ele.first, -azi_ele.second, 0.0);
  v.pose.R = Quaternion2RotationMatrix(v.pose.q);
  v.source = source;
  return v;
}

std::vector<View> SampleGoals(std::vector<Group> groups, SensorFoV sensor, MapGrid3D<float> map, octomap::OcTree* octomap, int sampleLimit, float minObstacleProximity, std::string mode="uniform")
{
  std::vector<View> goals;
  float radius = (sensor.rMax - sensor.rMin)*0.5 + sensor.rMin;
  ROS_INFO("Sampling goal views for %d groups.", (int)groups.size());
  for (int i=0; i<groups.size(); i++) {
    View v;
    bool goalFound = false;
    // ROS_INFO("Group %d:", i);
    for (int j=0; j<sampleLimit; j++) {
      if (mode == "uniform") v = SampleGoalRandomUniform(groups[i].centroid, sensor);
      if (mode == "gaussian") v = SampleGoalRandomGaussian(groups[i].centroid, radius);
      if (mode == "fibonacci") v = SampleGoalFibonacciList(groups[i].centroid, j, radius);
      if (CheckView(v, groups[i].centroid, map, octomap, minObstacleProximity, j)) {
        goals.push_back(v);
        goalFound = true;
        // ROS_INFO("Goal found for group %d after %d iterations", i, j);
        break;
      }
    }
    // Try reversing the normal if no goals have been found yet
    if (!goalFound) {
      pcl::PointXYZLNormal p = groups[i].centroid;
      p.normal_x = -p.normal_x; p.normal_y = -p.normal_y; p.normal_z = -p.normal_z;
      for (int j=0; j<sampleLimit; j++) {
        if (mode == "uniform") v = SampleGoalRandomUniform(p, sensor);
        if (mode == "gaussian") v = SampleGoalRandomGaussian(p, radius);
        if (mode == "fibonacci") v = SampleGoalFibonacciList(p, j, radius);
        if (CheckView(v, p, map, octomap, minObstacleProximity, j)) {
          goals.push_back(v);
          goalFound = true;
          // ROS_INFO("Goal found for group %d after %d iterations", i, sampleLimit + j);
          break;
        }
      }
    }
  }
  ROS_INFO("%d goals sampled.", (int)goals.size());
  return goals;
}

sensor_msgs::PointCloud2 ConvertGoalsToPointCloud2(std::vector<View> goals)
{
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZINormal>);
  for (int i=0; i<goals.size(); i++) {
    pcl::PointXYZINormal p;
    p.x = goals[i].pose.position.x;
    p.y = goals[i].pose.position.y;
    p.z = goals[i].pose.position.z;
    p.normal_x = goals[i].pose.R(0,0);
    p.normal_y = goals[i].pose.R(1,0);
    p.normal_z = goals[i].pose.R(2,0);
    p.intensity = goals[i].gain;
    cloud->points.push_back(p);
  }

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  return msg;
}

geometry_msgs::PoseArray ConvertGoalsToPoseArray(std::vector<View> goals)
{
  geometry_msgs::PoseArray msg;
  for (int i=0; i<goals.size(); i++) {
    geometry_msgs::Pose p;
    p.position.x = goals[i].pose.position.x;
    p.position.y = goals[i].pose.position.y;
    p.position.z = goals[i].pose.position.z;
    p.orientation.x = goals[i].pose.q.x;
    p.orientation.y = goals[i].pose.q.y;
    p.orientation.z = goals[i].pose.q.z;
    p.orientation.w = goals[i].pose.q.w;
    msg.poses.push_back(p);
  }
  return msg;
}

std::vector<View> FilterGoalsFrontiersBBX(std::vector<View> goals, pcl::PointCloud<pcl::PointXYZLNormal>::Ptr frontier,
                                          pcl::PointCloud<pcl::PointXYZLNormal>::Ptr frontierFiltered, std::vector<Eigen::Vector4f> extremes) 
{
  // Setup CropBox PCL filter
  pcl::CropBox<pcl::PointXYZLNormal> boxFilter;
  boxFilter.setMin(extremes[0]);
  boxFilter.setMax(extremes[1]);
  boxFilter.setInputCloud(frontier);
  boxFilter.filter(*frontierFiltered);

  // Store goal source points for another CropBox
  pcl::PointCloud<pcl::PointXYZLNormal>::Ptr sources (new pcl::PointCloud<pcl::PointXYZLNormal>);
  pcl::PointCloud<pcl::PointXYZLNormal>::Ptr sources_filtered (new pcl::PointCloud<pcl::PointXYZLNormal>);
  pcl::PointIndices::Ptr goalsRemovedIds (new pcl::PointIndices());
  for (int i=0; i<goals.size(); i++) sources->points.push_back(goals[i].source);
  boxFilter.setInputCloud(sources);
  boxFilter.filter(*sources_filtered);
  boxFilter.getRemovedIndices(*goalsRemovedIds);

  // Iterate through removed indices and 
  std::sort(goalsRemovedIds->indices.begin(), goalsRemovedIds->indices.end());
  for (int i=(goalsRemovedIds->indices.size()-1); i<0; i--) {
    goals.erase(goals.begin() + goalsRemovedIds->indices[i]);
  }
  return goals;
}

MapGrid3D<int> ConvertFrontierPointCloudToMapGrid3D(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr frontier, float voxelSize)
{
  float min[3], max[3];
  GetPointCloudBounds(frontier, min, max);
  ROS_INFO("Frontier cloud has bounds (%0.1f, %0.1f, %0.1f) to (%0.1f, %0.1f, %0.1f).", min[0], min[1], min[2], max[0], max[1], max[2]);
  for (int i=0; i<3; i++) {
    min[i] = min[i] - 1.0*voxelSize;
    max[i] = max[i] + 1.0*voxelSize;
  }
  MapGrid3D<int> frontierMap;
  frontierMap.Reset(voxelSize, min, max, -1);
  // ROS_INFO("Copying over new frontier data into gridMap3D");
  for (int i=0; i<frontier->points.size(); i++) {
    Point p = {frontier->points[i].x, frontier->points[i].y, frontier->points[i].z};
    if (frontierMap._CheckVoxelPositionInBounds(p))
    frontierMap.SetVoxel(frontier->points[i].x, frontier->points[i].y, frontier->points[i].z, i);
    else {
      ROS_INFO("Frontier voxel out of bounds");
    }
  }
  return frontierMap;
}

void GetFrontierDiff(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr frontierOld, pcl::PointCloud<pcl::PointXYZLNormal>::Ptr frontierNew,
  float voxelSize, pcl::PointCloud<pcl::PointXYZLNormal>::Ptr frontierDiff, std::vector<pcl::PointIndices> &clustersDiff)
{
  ROS_INFO("Getting the difference between frontiers...");
  // Clear out frontierDiff points to store the function output
  frontierDiff->points.clear();
  clustersDiff.clear();

  // Convert frontierOld to a flat matrix of array indices
  MapGrid3D<int> frontierOldMap = ConvertFrontierPointCloudToMapGrid3D(frontierOld, voxelSize);
  ROS_INFO("Converted old frontier pointcloud to a MapGrid3D object.");

  // Check if the frontier voxels have changed between new and old
  int diffVoxelCount = 0;
  for (int i=0; i<frontierNew->points.size(); i++) {
    pcl::PointXYZLNormal frontierVoxel = frontierNew->points[i];
    Point p;
    bool addVoxel = false;
    p.x = frontierVoxel.x; p.y = frontierVoxel.y; p.z = frontierVoxel.z;
    if (frontierOldMap._CheckVoxelPositionInBounds(p)) {
      if (frontierOldMap.Query(p.x, p.y, p.z) == -1) {
        addVoxel = true;
      }
    } else {
      addVoxel = true;
    }
    if (addVoxel) {
      // Add indices to clustersDiff until it's the same size
      while (frontierVoxel.label >= clustersDiff.size()) {
        ROS_INFO("Adding cluster to cluster indices vector of length %d for frontier voxel with label %d...", (int)clustersDiff.size(), frontierVoxel.label);
        pcl::PointIndices indices;
        clustersDiff.push_back(indices);
      }
      frontierDiff->points.push_back(frontierVoxel);
      clustersDiff[frontierVoxel.label].indices.push_back(diffVoxelCount);
      addVoxel = false;
      diffVoxelCount++;
    }
  }

  ROS_INFO("%d clusters worth of indices initially copied with sizes:", (int)clustersDiff.size());
  for (int i=0; i<clustersDiff.size(); i++) std::cout << clustersDiff[i].indices.size() << ", ";
  std::cout << std::endl;

  // Erase clusterIndices entries with 0 members
  for (int i=(clustersDiff.size()-1); i>=0; i--) {
    if (clustersDiff[i].indices.size() == 0) {
      clustersDiff.erase(clustersDiff.begin() + i);
    }
  }

  ROS_INFO("Calculated difference cloud between two frontiers.");
}

std::vector<View> FilterGoalsNewFrontier(std::vector<View> goals, float voxelSize, pcl::PointCloud<pcl::PointXYZLNormal>::Ptr frontierNew)
{
  // Remove all goals whose frontier source voxel is no longer a frontier
  ROS_INFO("%d goals from last sampler run.  Erasing goal poses sourced by voxels that are no longer frontier...", (int)goals.size());
  int goalsErased = 0;
  MapGrid3D<int> frontierNewMap = ConvertFrontierPointCloudToMapGrid3D(frontierNew, voxelSize);
  ROS_INFO("New frontier grid map created with bounds (%0.1f, %0.1f, %0.1f) to (%0.1f, %0.1f, %0.1f) ...", frontierNewMap.minBounds.x, 
        frontierNewMap.minBounds.y, frontierNewMap.minBounds.z, frontierNewMap.maxBounds.x, frontierNewMap.maxBounds.y, frontierNewMap.maxBounds.z);
  if (goals.size() > 0) {
    for (int i=(goals.size()-1); i>=0; i--) {
      View goal = goals[i];
      int label = goal.source.label;
      Point p {goal.source.x, goal.source.y, goal.source.z};
      if (frontierNewMap._CheckVoxelPositionInBounds(p)) {
        if (frontierNewMap.Query(goal.source.x, goal.source.y, goal.source.z) <= 0) {
          goals.erase(goals.begin()+i);
          goalsErased++;
        }
      } else {
        goals.erase(goals.begin()+i);
        goalsErased++;
      }
    }
  }
  ROS_INFO("%d goal poses removed because their source voxels are from a changed frontier cluster", goalsErased);
  return goals;
}

std::vector<View> FindGoalsCloseToCloud(std::vector<View> &goals, pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud, float radius)
{
  // This function finds all goal views that are within a radius of any point in the input cloud.
  // These goals are stored in the output (goalsClose) and erased from the goals vector.

  std::vector<View> goalsClose;
  if (cloud->points.size() == 0) return goalsClose;
  int K = 1; // Find the closest neighbor, consider modifying
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  pcl::KdTreeFLANN<pcl::PointXYZLNormal> kdtree;
  std::vector<int> goalsCloseIdx;
  kdtree.setInputCloud(cloud);

  for (int i=0; i<goals.size(); i++) {
    pcl::PointXYZLNormal searchPoint;
    searchPoint.x = goals[i].pose.position.x;
    searchPoint.y = goals[i].pose.position.y;
    searchPoint.z = goals[i].pose.position.z;
    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
      // Remove from goals and add to goalsClose
      View goal = goals[i];
      goalsClose.push_back(goal);
      goalsCloseIdx.push_back(i);
    }
  }

  for (int i=(goalsCloseIdx.size()-1); i>=0; i--) {
    goals.erase(goals.begin() + goalsCloseIdx[i]);
  }
  return goalsClose;
}

void FilterCloudNearGoals(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn,  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut, std::vector<View> goals,
  SensorFoV sensor, float voxelSize, pcl::PointCloud<pcl::PointXYZLNormal>::Ptr frontier, std::string gainType)
{
  ROS_INFO("Generating a smaller EDT cloud for gain calculations...");
  if (goals.size() == 0) return;
  Eigen::Vector4f goalsMin;
  goalsMin << goals[0].pose.position.x, goals[0].pose.position.y, goals[0].pose.position.z, 0.0;
  Eigen::Vector4f goalsMax;
  goalsMax << goals[0].pose.position.x, goals[0].pose.position.y, goals[0].pose.position.z, 0.0;
  for (int i=0; i<goals.size(); i++) {
    pcl::PointXYZ p = goals[i].pose.position;
    goalsMin[0] = std::min(p.x, goalsMin[0]);
    goalsMin[1] = std::min(p.y, goalsMin[1]);
    goalsMin[2] = std::min(p.z, goalsMin[2]);
    goalsMax[0] = std::max(p.x, goalsMax[0]);
    goalsMax[1] = std::max(p.y, goalsMax[1]);
    goalsMax[2] = std::max(p.z, goalsMax[2]);
  }
  float boundsMin[3], boundsMax[3];
  float delta[3] = {sensor.rMax, sensor.rMax, std::sin(sensor.verticalFoV*M_PI/360.0)*sensor.rMax};
  int size[3];
  for (int i=0; i<3; i++)
  {
    boundsMin[i] = goalsMin[i] - ((float)1.1)*delta[i];
    goalsMin[i] = boundsMin[i];
    boundsMax[i] = goalsMax[i] + ((float)1.1)*delta[i];
    goalsMax[i] = boundsMax[i];
    size[i] = std::roundf((boundsMax[i] - boundsMin[i])/voxelSize) + 1;
  }
  // Filter cloud_in using a CropBox Filter
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInFiltered (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::CropBox<pcl::PointXYZI> boxFilter;
  boxFilter.setMin(goalsMin);
  boxFilter.setMax(goalsMax);
  boxFilter.setNegative(false);
  boxFilter.setInputCloud(cloudIn);
  boxFilter.filter(*cloudInFiltered);
  // Convert input cloud to padded MapGrid3D
  MapGrid3D<float> mapConvert(voxelSize, size, boundsMin);
  if (gainType == "frontier") mapConvert.SetAll(-2.0);
  else mapConvert.SetAll(-1.0);

  ROS_INFO("Copying EDT data into rectangular map grid with bounds (%0.1f, %0.1f, %0.1f) to (%0.1f, %0.1f, %0.1f) ...", mapConvert.minBounds.x, 
    mapConvert.minBounds.y, mapConvert.minBounds.z, mapConvert.maxBounds.x, mapConvert.maxBounds.y, mapConvert.maxBounds.z);
  for (int i=0; i<cloudInFiltered->points.size(); i++) {
    pcl::PointXYZI query = cloudInFiltered->points[i];
    if (query.intensity >= 0.0) mapConvert.SetVoxel(query.x, query.y, query.z, query.intensity);
  }

  ROS_INFO("Copying frontier data into rectangular map grid...");
  if (gainType == "frontier") {
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr frontierFiltered (new pcl::PointCloud<pcl::PointXYZLNormal>);
    pcl::CropBox<pcl::PointXYZLNormal> boxFilterFrontier;
    boxFilterFrontier.setMin(goalsMin);
    boxFilterFrontier.setMax(goalsMax);
    boxFilterFrontier.setNegative(false);
    boxFilterFrontier.setInputCloud(frontier);
    boxFilterFrontier.filter(*frontierFiltered);
    for (int i=0; i<frontierFiltered->points.size(); i++) {
      pcl::PointXYZLNormal frontierVoxel = frontierFiltered->points[i];
      mapConvert.SetVoxel(frontierVoxel.x, frontierVoxel.y, frontierVoxel.z, -1.0);
    }
  }

  // Convert back to pointcloud
  ROS_INFO("Converting map grid data into pointcloud...");
  CopyMapGrid3DToPointCloud(mapConvert, cloudOut);
  return;
}


#endif