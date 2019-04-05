// g++ depthImg2PCL2.cpp -g -o depthImg2PCL2.o -I /opt/ros/melodic/include -I /usr/include/c++/7.3.0 -I /usr/include/eigen3 -I /usr/include/pcl-1.8 -I /home/andrew/catkin_ws/devel/include  -I /usr/local/include -L /usr/lib/x86_64-linux-gnu -L /home/andrew/catkin_ws/devel/lib -L /opt/ros/melodic/lib -L /usr/lib -L /usr/local/lib -Wl,-rpath,opt/ros/melodic/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization -lboost_system -lpcl_common -lpcl_io -lopencv_core -lopencv_highgui -lcv_bridge

// C++ Standard Libraries
#include <math.h>
// ROS libraries
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
// OpenCV2
#include <opencv2/highgui/highgui.hpp>
// pcl
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// pcl ROS
#include <pcl_conversions/pcl_conversions.h>
// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

struct Sensor
{
  double rMin = 0.05; // meters
  double rMax = 6.0; // meters
  std::string frame;
  Eigen::Matrix3d R; // camera Rotation matrix
  Eigen::VectorXd D_plumb_bob; // camera Distortion vector
  Eigen::Matrix3d K; // camera Intrinsics matrix
};

class project_truncated_depths
{
  public:
    // Constructor

    // Property Definitions

      // Storing pcl2 message for publishing
      sensor_msgs::PointCloud2 depthCloudMsg;

      // Camera parameters
      Sensor camera;

      // Got Callback booleans
      bool gotCameraInfo = false;
      bool gotDepthImage = false;

    // Method Definitions
    void callback_camera_info(const sensor_msgs::CameraInfo msg);
    void callback_image(const sensor_msgs::ImageConstPtr& msg);
};

void project_truncated_depths::callback_camera_info(const sensor_msgs::CameraInfo msg)
{
  // Copy camera parameters

  camera.frame = msg.header.frame_id;

  // Rigid Rotation Matrix
  camera.R(0,0) = msg.R[0];
  camera.R(0,1) = msg.R[1];
  camera.R(0,2) = msg.R[2];
  camera.R(1,0) = msg.R[3];
  camera.R(1,1) = msg.R[4];
  camera.R(1,2) = msg.R[5];
  camera.R(2,0) = msg.R[6];
  camera.R(2,1) = msg.R[7];
  camera.R(2,2) = msg.R[8];

  // Distortion
  camera.D_plumb_bob.resize(5);
  camera.D_plumb_bob(0) = msg.D[0];
  camera.D_plumb_bob(1) = msg.D[1];
  camera.D_plumb_bob(2) = msg.D[2];
  camera.D_plumb_bob(3) = msg.D[3];
  camera.D_plumb_bob(4) = msg.D[4];

  // Intrinsics
  camera.K(0,0) = msg.K[0];
  camera.K(0,1) = msg.K[1];
  camera.K(0,2) = msg.K[2];
  camera.K(1,0) = msg.K[3];
  camera.K(1,1) = msg.K[4];
  camera.K(1,2) = msg.K[5];
  camera.K(2,0) = msg.K[6];
  camera.K(2,1) = msg.K[7];
  camera.K(2,2) = msg.K[8];

  // Set callback boolean to true
  gotCameraInfo = true;
}

void project_truncated_depths::callback_image(const sensor_msgs::ImageConstPtr& msg)
{
  // Return if no camera parameters have been received.
  if (!gotCameraInfo) {
    return;
  }

  // Convert to an opencv2 image pointer
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    // cv_ptr = cv_bridge::toCvCopy(msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Display the depth image
  // cv::imshow("depth image", cv_ptr->image);
  // cv::waitKey(3);

  // Allocate PC2 memory and loop through all the pixels to get depth info
  int width = cv_ptr->image.cols; // image pixel width
  int height = cv_ptr->image.rows; // image pixel height
  double cx = camera.K(0,2);
  double cy = camera.K(1,2);
  double fx = camera.K(0,0);
  double fy = camera.K(1,1);

  float z_min = 10.0;
  float z_max = 0.0;

  pcl::PointXYZ pt;

  pcl::PointCloud<pcl::PointXYZ>::Ptr depthCloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Loop through all the pixels in the image and assign them to points in the pointcloud2
  for (int u=0; u<width; u++) {
    for (int v=0; v<height; v++) {
      float z = cv_ptr->image.at<float>(v,u);
      if (z >= camera.rMin) {
        pt.x = z*(((float)u - cx)/fx);
        pt.y = z*(((float)v - cy)/fy);
        pt.z = z;
      } else if (std::isnan(z)) {
        // Look at the value of your neighbor pixels to see what you should be.  If you're the first pixel, scan down and right until you get a z value
        // ROS_INFO("At pixel [%d, %d].", u, v);
        float z_neighbor = std::numeric_limits<float>::quiet_NaN();
        if (u == 0 && v == 0) {
          int du = 0;
          int dv = 0;
          while (std::isnan(z_neighbor) && (u+du)<width) {
            while (std::isnan(z_neighbor) && (v+dv)<height) {
              z_neighbor = cv_ptr->image.at<float>(v+dv,u+du);
              dv++;
            }
            du++;
          }
          cv_ptr->image.at<float>(v,u) = z_neighbor;
        } else if (v == 0) {
          z_neighbor = cv_ptr->image.at<float>(v,u-1);
          cv_ptr->image.at<float>(v,u) = z_neighbor;
        } else {
          z_neighbor = cv_ptr->image.at<float>(v-1,u);
          cv_ptr->image.at<float>(v,u) = z_neighbor;
        }

        if (std::isnan(z_neighbor) || (z_neighbor < ((camera.rMax + camera.rMin)/2.0))) {
          pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
        } else {
          pt.x = z_neighbor*(((float)u - cx)/fx);
          pt.y = z_neighbor*(((float)v - cy)/fy);
          pt.z = z_neighbor;
        }
      }
      else {
        pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
      }
      depthCloud->points.push_back(pt);
    }
  }

  depthCloud->width = width;
  depthCloud->height = height;
  depthCloud->is_dense = true;

  // Convert to SensorMsgs::PointCloud2
  // Declare a new cloud to store the converted message
  sensor_msgs::PointCloud2 newPointCloud2;

  // Convert from pcl::PointCloud to sensor_msgs::PointCloud2
  pcl::toROSMsg(*depthCloud, newPointCloud2);
  newPointCloud2.header.seq = 1;
  newPointCloud2.header.stamp = ros::Time();
  newPointCloud2.header.frame_id = camera.frame;

  // Update the old message
  depthCloudMsg = newPointCloud2;
}

int main(int argc, char **argv)
{
  // Initialize ROS node with name and object instantiation
  ros::init(argc, argv, "project_truncated_depths");
  ros::NodeHandle n;

  project_truncated_depths trunc_depths;

  // Declare subscribers for the camera info and the raw depth image.  Use the image truncator callbacks
  ros::Subscriber sub1 = n.subscribe("X4/rgbd_camera/depth/camera_info", 1, &project_truncated_depths::callback_camera_info, &trunc_depths);
  ros::Subscriber sub2 = n.subscribe("X4/rgbd_camera/depth/image_raw", 1, &project_truncated_depths::callback_image, &trunc_depths);

  ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("X4/depth_cam_fixed", 5);

  // Get depth camera min and max range values
  double rMin, rMax;
  n.param("rgbd_camera/rMin", rMin, (double)0.05);
  n.param("rgbd_camera/rMax", rMax, (double)6.0);
  trunc_depths.camera.rMin = rMin;
  trunc_depths.camera.rMax = rMax;

  // Declare and read in the node update rate from the launch file parameters
  double updateRate;
  n.param("project_truncated_depths/updateRate", updateRate, (double)200.0); // Hz
  ros::Rate r(updateRate);

  // Run the node until ROS quits
  while (ros::ok())
  {
    r.sleep(); // Node sleeps to update at a rate as close as possible to the updateRate parameter
    ros::spinOnce(); // All subscriber callbacks are called here.
    pub1.publish(trunc_depths.depthCloudMsg);
  }

  return 0;
}