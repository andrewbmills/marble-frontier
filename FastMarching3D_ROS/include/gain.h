#ifndef GAIN_H
#define GAIN_H

#include <math.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <pcl/point_types.h>
#include <pcl/filters/frustum_culling.h>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <mapGrid3D.h>

// Some orientation and pose structures
struct Quaternion {
  float w, x, y, z;
};

struct Quaternion HamiltonProduct(Quaternion q1, Quaternion q2)
{
  Quaternion q3;
  q3.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
  q3.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
  q3.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
  q3.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
  return q3;
}

struct Quaternion Conjugate(Quaternion q)
{
  q.x = -q.x; q.y = -q.y; q.z = -q.z;
  return q;
}

Eigen::Vector3f RotateVectorByQuaternion(Eigen::Vector3f v1, Quaternion q)
{
  Eigen::Vector3f v2;
  Quaternion q_v1;
  q_v1.w = 0.0; q_v1.x = v1(0); q_v1.y = v1(1); q_v1.z = v1(2);
  Quaternion q_v2 = HamiltonProduct(Conjugate(q), HamiltonProduct(q_v1, q));
  v2(0) = q_v2.x; v2(1) = q_v2.y; v2(2) = q_v2.z; 
  return v2;
}

Eigen::Matrix3f Quaternion2RotationMatrix(Quaternion q)
{
  Eigen::Matrix3f R;
  float s = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
  R(0,0) = 1 - 2.0*(q.y*q.y + q.z*q.z)/s;
  R(1,0) = 2.0*(q.x*q.y + q.z*q.w);
  R(2,0) = 2.0*(q.x*q.z - q.y*q.w);
  R(0,1) = 2.0*(q.x*q.y - q.z*q.w);
  R(1,1) = 1 - 2.0*(q.x*q.x + q.z*q.z)/s;
  R(2,1) = 2.0*(q.y*q.z + q.x*q.w);
  R(0,2) = 2.0*(q.x*q.z + q.y*q.w);;
  R(1,2) = 2.0*(q.y*q.z - q.x*q.w);
  R(2,2) = 1 - 2.0*(q.x*q.x + q.y*q.y)/s;
  return R;
}

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
  std::string type;
};
struct View {
  Pose pose;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::PointXYZLNormal source;
  int index = -1;
  float gain = 0.0;
};

Eigen::MatrixXf CalculateFieldOfViewNormals(Pose pose, SensorFoV sensor)
{
  float alpha = sensor.horizontalFoV*M_PI/360.0;
  float beta = sensor.verticalFoV*M_PI/360.0;
  Eigen::MatrixXf N(3,4);
  // Normals in camera frame
  N(0,0) = std::sin(alpha); N(1,0) = -std::cos(alpha); N(2,0) = 0.0;
  N(0,1) = N(0,0); N(1,1) = -N(1,0); N(2,1) = 0.0;
  N(0,2) = std::sin(beta); N(1,2) = 0.0; N(2,2) = -std::cos(beta);
  N(0,3) = N(0,2); N(1,3) = 0.0; N(2,3) = -N(2,2);
  // Multiply by R to put into inertial frame
  Eigen::MatrixXf N_inertial(3,4);
  N_inertial = pose.R*N;
  return N_inertial;
}

float FindConeCoefficient(SensorFoV sensor)
{
  float coeff = std::tan(M_PI/2.0 - sensor.verticalFoV*M_PI/360.0);
  return (coeff*coeff);
}

void GetPointsInFieldOfView(Pose pose, SensorFoV sensor, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out)
{
  // Lots of geometry checks in this function.

  // Get Field of View normals matrix
  Eigen::MatrixXf N = CalculateFieldOfViewNormals(pose, sensor);

  // Store robot position in vector
  Eigen::Vector3f p_robot;
  p_robot << pose.position.x, pose.position.y, pose.position.z;

  // Comparison values for field of view
  float range_min_squared = sensor.rMin*sensor.rMin;
  float range_max_squared = sensor.rMax*sensor.rMax;
  float vertical_cone_coefficient = FindConeCoefficient(sensor);
  cloud_out->points.clear();

  // Parse input Pointcloud into a matrix
  Eigen::MatrixXf P(cloud_in->points.size(), 3);
  for (int i=0; i<cloud_in->points.size(); i++) {
    // Store point in vector
    Eigen::Vector3f p;
    p << cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z;
    p = p - p_robot;

    // Check range condition
    float p_norm_squared = p.squaredNorm();
    if ((p_norm_squared > range_max_squared) || (p_norm_squared < range_min_squared)) continue;
    // Check horizontal FoV condition
    if (sensor.horizontalFoV <= 180.0) {
      if ((p.dot(N.col(0)) < 0.0) || (p.dot(N.col(1)) < 0.0)) continue;
    }
    else {
      if ((p.dot(N.col(0)) < 0.0) && (p.dot(N.col(1)) < 0.0)) continue;
    }
    // Check vertical FoV
    if (sensor.type == "camera") { // Plane condition
      if (sensor.verticalFoV <= 180.0) {
        if ((p.dot(N.col(2)) < 0.0) || (p.dot(N.col(3)) < 0.0)) continue;
      }
      else {
        if ((p.dot(N.col(2)) < 0.0) && (p.dot(N.col(3)) < 0.0)) continue;
      }
    }
    else if (sensor.type == "lidar") { // Cone condition
      if (sensor.verticalFoV <= 180.0) {
        if ((p(0)*p(0) + p(1)*p(1)) < vertical_cone_coefficient*p(2)*p(2)) continue;
      }
    }

    // All conditions passed, add point to output cloud
    cloud_out->points.push_back(cloud_in->points[i]);
  }
  return;
}

void CopyOctomapBbxToPointCloud(octomap::OcTree* map, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float bbx_min_array[3], float bbx_max_array[3]) {
  // ROS_INFO("Set bbx limits (%0.1f, %0.1f, %0.1f) to (%0.1f, %0.1f, %0.1f)", bbx_min_array[0], bbx_min_array[1], bbx_min_array[2],
  //                                                                           bbx_max_array[0], bbx_max_array[1], bbx_max_array[2]);
  octomap::point3d bbx_min_octomap(bbx_min_array[0], bbx_min_array[1], bbx_min_array[2]);
  octomap::point3d bbx_max_octomap(bbx_max_array[0], bbx_max_array[1], bbx_max_array[2]);
  // ROS_INFO("Clearing previous PointCloud data");
  cloud->points.clear();
  // ROS_INFO("Iterating through Octomap");
  int i = 1;
  for(octomap::OcTree::leaf_bbx_iterator
  it = map->begin_leafs_bbx(bbx_min_octomap, bbx_max_octomap);
  it != map->end_leafs_bbx(); ++it)
  {
    pcl::PointXYZI p;
    p.x = it.getX(); p.y = it.getY(); p.z = it.getZ();
    p.intensity = it->getOccupancy();
    cloud->points.push_back(p);
    i++;
  }
  return;
}

MapGrid3D<float> CopyOctomapBBxToMapGrid3D(octomap::OcTree* map, float bbx_min_array[3], float bbx_max_array[3]) {
  // ROS_INFO("Set bbx limits (%0.2f, %0.2f, %0.2f) to (%0.2f, %0.2f, %0.2f)", bbx_min_array[0], bbx_min_array[1], bbx_min_array[2],
                                                                            // bbx_max_array[0], bbx_max_array[1], bbx_max_array[2]);
  octomap::point3d bbx_min_octomap(bbx_min_array[0], bbx_min_array[1], bbx_min_array[2]);
  octomap::point3d bbx_max_octomap(bbx_max_array[0], bbx_max_array[1], bbx_max_array[2]);
  // ROS_INFO("Iterating through Octomap");
  float voxel_size = (float)map->getResolution();
  MapGrid3D<float> map_grid;
  bool first_voxel = true;
  for(octomap::OcTree::leaf_bbx_iterator
      it = map->begin_leafs_bbx(bbx_min_octomap, bbx_max_octomap);
      it != map->end_leafs_bbx(); ++it) {
    if (first_voxel) {
      Point min_bounds;
      min_bounds.x = it.getX() - voxel_size*(std::round((it.getX() - bbx_min_array[0])/voxel_size) + 2);
      min_bounds.y = it.getY() - voxel_size*(std::round((it.getY() - bbx_min_array[1])/voxel_size) + 2);
      min_bounds.z = it.getZ() - voxel_size*(std::round((it.getZ() - bbx_min_array[2])/voxel_size) + 2);
      Dimensions map_sizes;
      map_sizes.x = std::round((bbx_max_array[0] - min_bounds.x)/voxel_size) + 2;
      map_sizes.y = std::round((bbx_max_array[1] - min_bounds.y)/voxel_size) + 2;
      map_sizes.z = std::round((bbx_max_array[2] - min_bounds.z)/voxel_size) + 2;
      map_grid.minBounds = min_bounds;
      map_grid.voxelSize = voxel_size;
      map_grid.size = map_sizes;
      map_grid._SetMaxBounds();
      map_grid.voxels.resize(map_sizes.x*map_sizes.y*map_sizes.z);
      // ROS_INFO("Set gridmap limits (%0.2f, %0.2f, %0.2f) to (%0.2f, %0.2f, %0.2f)", map_grid.minBounds.x, map_grid.minBounds.y, map_grid.minBounds.z,
                                                                                    // map_grid.maxBounds.x, map_grid.maxBounds.y, map_grid.maxBounds.z);
      map_grid.SetAll(0.5);
      first_voxel = false;
    }
    map_grid.SetVoxel(it.getX(), it.getY(), it.getZ(), it->getOccupancy());
  }
  return map_grid;
}

void CopyMapGrid3DToPointCloud(MapGrid3D<float> map, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
  cloud->points.clear();
  for (int i=0; i < map.voxels.size(); i++) {
    pcl::PointXYZI p;
    Point p_map = map._ConvertIndexToPosition(i);
    p.x = p_map.x;
    p.y = p_map.y;
    p.z = p_map.z;
    p.intensity = map.voxels[i];
    cloud->points.push_back(p);
  }
  return;
}

void CheckLineOfSight(Pose pose, octomap::OcTree* map, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out, bool unseen = false)
{
  octomap::point3d origin(pose.position.x, pose.position.y, pose.position.z);
  for (int i=0; i<cloud_in->points.size(); i++) {
    float dx = cloud_in->points[i].x - origin.x();
    float dy = cloud_in->points[i].y - origin.y();
    float dz = cloud_in->points[i].z - origin.z();
    // ROS_INFO("Checking if (%0.1f, %0.1f, %0.1f) is occluded", cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
    double radius_squared = dx*dx + dy*dy + dz*dz;
    // ROS_INFO("Casting ray from (%0.1f, %0.1f, %0.1f) in direction (%0.1f, %0.1f, %0.1f)", origin.x(), origin.y(), origin.z(), dx, dy, dz);
    octomap::point3d direction(dx, dy, dz);
    octomap::point3d stop;
    bool hit = map->castRay(origin, direction, stop, unseen, (double)std::sqrt(radius_squared));
    double radius_stop_squared = (stop.x() - origin.x())*(stop.x() - origin.x()) + 
                                 (stop.y() - origin.y())*(stop.y() - origin.y()) +
                                 (stop.z() - origin.z())*(stop.z() - origin.z());
    if ((radius_squared - radius_stop_squared) <= 1.5*map->getResolution()) {
      // query point is the one that stops the raycast
      // ROS_INFO("Point (%0.1f, %0.1f, %0.1f) is visible!", cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
      cloud_out->points.push_back(cloud_in->points[i]);
    }
  }
  return;
}

float GainUnseen(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  float gain = 0.0;
  for (int i=0; i<cloud->points.size(); i++) {
    if ((cloud->points[i].intensity >= 0.4) && (cloud->points[i].intensity <= 0.6)) gain += 1.0;
  }
  return gain;
}

float Gain(Pose pose, SensorFoV sensor, octomap::OcTree* map, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_seen, std::string type="frontier", std::string mode="normal")
{
  // Calculates the gain value of a sensor at a pose within an occupancy grid map.
  // Also returns the voxels seen by the sensor at a pose in the cloud_seen PointCloud.

  if (type == "constant") return 1.0;

  // Copy map in PointCloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  float bbx_min[3] = {pose.position.x - ((float)1.1)*sensor.rMax, pose.position.y - ((float)1.1)*sensor.rMax, pose.position.z - ((float)1.1)*sensor.rMax};
  float bbx_max[3] = {pose.position.x + ((float)1.1)*sensor.rMax, pose.position.y + ((float)1.1)*sensor.rMax, pose.position.z + ((float)1.1)*sensor.rMax};
  if (mode == "debug") ROS_INFO("Copying octomap bounding box voxels into pointcloud.");
  // CopyOctomapBbxToPointCloud(map, cloud, bbx_min, bbx_max);
  MapGrid3D<float> map_grid = CopyOctomapBBxToMapGrid3D(map, bbx_min, bbx_max);
  if (mode == "debug") ROS_INFO("Map grid initialized with resolution %0.1f, sizes [%d, %d, %d], and min bounds [%0.2f, %0.2f, %0.2f]", map_grid.voxelSize,
            map_grid.size.x, map_grid.size.y, map_grid.size.z, map_grid.minBounds.x, map_grid.minBounds.y, map_grid.minBounds.z);
  CopyMapGrid3DToPointCloud(map_grid, cloud);
  if (mode == "debug") ROS_INFO("%d points copied from octomap to local pointcloud.", cloud->points.size());

  // Get cloud of voxels inside of sensor FoV
  if (mode == "debug") ROS_INFO("Getting points inside geometric sensor field of view.");
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_fov(new pcl::PointCloud<pcl::PointXYZI>);
  GetPointsInFieldOfView(pose, sensor, cloud, cloud_fov);
  if (mode == "debug") ROS_INFO("%d points possibly in camera field of view.", cloud_fov->points.size());

  // Check Line-of-sight on cloud_fov points
  bool include_unseen = false;
  if (mode == "debug") ROS_INFO("Checking for occlusions.");
  if (type == "frontier") {
    CheckLineOfSight(pose, map, cloud_fov, cloud_seen, false); // Consider the value of only the frontier
  }
  if (type == "unseen") {
    CheckLineOfSight(pose, map, cloud_fov, cloud_seen, true); // Consider the value of unseen voxels beyond the frontier
  }

  // Calculate gain for the seen voxels
  float gain = GainUnseen(cloud_seen);
  if (mode == "debug") ROS_INFO("Calculated gain of %0.1f unseen voxels.", gain);
  return gain;
}

#endif