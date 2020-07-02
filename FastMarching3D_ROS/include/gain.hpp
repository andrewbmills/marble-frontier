#include <math.h>
#include <string>
#include <vector>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <pcl/point_types.h>
#include <pcl/filters/frustum_culling.h>
#include <Eigen/Dense>

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
  std::string type;
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
  float coeff = std::tan(M_PI - sensor.verticalFoV*M_PI/360.0);
  return (coeff*coeff);
}

void GetPointsInFieldOfView(Pose pose, SensorFoV sensor, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out)
{
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
    p -= p_robot;

    // Check range condition
    float p_norm_squared = p.squaredNorm();
    if ((p_norm_squared > range_max_squared) || (p_norm_squared < range_min_squared)) continue;
    // Check horizontal FoV condition
    if ((p.dot(N.col(0)) < 0.0) || (p.dot(N.col(1)) < 0.0)) continue;
    // Check vertical FoV
    if (sensor.type == "camera") { // Plane condition
      if ((p.dot(N.col(2)) < 0.0) || (p.dot(N.col(3)) < 0.0)) continue;
    }
    else if (sensor.type == "lidar") { // Cone condition
      if ((p(0)*p(0) + p(1)*p(1)) < vertical_cone_coefficient*p(2)*p(2)) continue;
    }

    // All conditions passed, add point to output cloud
    cloud_out->points.push_back(cloud_in->points[i]);
  }
  return;
}

void CopyOctomapBbxToPointCloud(octomap::OcTree* map, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float bbx_min_array[3], float bbx_max_array[3]) {
  octomap::point3d bbx_min_octomap(bbx_min_array[0], bbx_min_array[1], bbx_min_array[2]);
  octomap::point3d bbx_max_octomap(bbx_max_array[0], bbx_max_array[1], bbx_max_array[2]);
  cloud->points.clear();
  for(octomap::OcTree::leaf_bbx_iterator
  it = map->begin_leafs_bbx(bbx_min_octomap, bbx_max_octomap);
  it != map->end_leafs_bbx(); ++it)
  {
    pcl::PointXYZI p;
    p.x = it.getX(); p.y = it.getY() ; p.z = it.getZ();
    p.intensity = it->getOccupancy();
    cloud->points.push_back(p);
  }
}

void CheckLineOfSight(Pose pose, octomap::OcTree* map, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out, bool unseen = false)
{
  octomap::point3d origin(pose.position.x, pose.position.y, pose.position.z);
  for (int i=0; i<cloud_in->points.size(); i++) {
    octomap::point3d origin(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
    float dx = cloud_in->points[i].x - origin.x();
    float dy = cloud_in->points[i].y - origin.y();
    float dz = cloud_in->points[i].z - origin.z();
    double radius_squared = dx*dx + dy*dy + dz*dz;
    octomap::point3d direction(dx, dy, dz);
    octomap::point3d stop;
    if (map->castRay(origin, direction, stop, false, std::sqrt(radius_squared))) {
      double radius_stop_squared = (stop.x() - origin.x())*(stop.x() - origin.x()) + 
                                   (stop.y() - origin.y())*(stop.y() - origin.y()) +
                                   (stop.z() - origin.z())*(stop.z() - origin.z());
      if (std::abs(radius_squared - radius_stop_squared) <= map->getResolution()) {
        // query point is occupied and is the one that stops the raycast
        cloud_out->points.push_back(cloud_in->points[i]);
      }
    }
    else {
      cloud_out->points.push_back(cloud_in->points[i]);
    }
  }
  return;
}

float Gain(View view, SensorFoV sensor, octomap::OcTree* map, std::string type, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_seen)
{
  // Copy map in PointCloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  float bbx_min[3] = {view.pose.position.x - 1.1*sensor.rMax, view.pose.position.y - 1.1*sensor.rMax, view.pose.position.z - 1.1*sensor.rMax};
  float bbx_max[3] = {view.pose.position.x + 1.1*sensor.rMax, view.pose.position.y + 1.1*sensor.rMax, view.pose.position.z + 1.1*sensor.rMax};
  CopyOctomapBbxToPointCloud(map, cloud, bbx_min, bbx_max);

  // Get cloud of voxels inside of sensor FoV
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_fov;
  GetPointsInFieldOfView(view.pose, sensor, cloud, cloud_fov);

  // Check Line-of-sight on cloud_fov points
  // bool include_unseen = false;I
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_seen;
  CheckLineOfSight(view.pose, map, cloud_fov, cloud_seen);

  // Calculate gain for the seen voxels
  float gain = GainUnseen(cloud_seen);
  return 0.0;
}

float GainUnseen(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  float gain = 0.0;
  for (int i=0; i<cloud->points.size(); i++) {
    if ((cloud->points[i].intensity > 0.4) || (cloud->points[i].intensity < 0.6)) gain += 1.0;
  }
  return gain;
}