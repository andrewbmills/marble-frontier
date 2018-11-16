// g++ msfm3d_node.cpp -o msfm3d_node.o -I /opt/ros/melodic/include -I /usr/include/c++/7.3.0 -L /opt/ros/melodic/lib -Wl,-rpath,/opt/ros/melodic/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization


#include <ros/ros.h>
// #include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <gazebo_msgs/LinkStates.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
// #include <boost/foreach.hpp>

__inline int mindex3(int x, int y, int z, int sizx, int sizy) { return x+y*sizx+z*sizx*sizy; }

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

//msfm3d class declaration
class Msfm3d
{
  public:
    Msfm3d() {n.getParam("/X1/voxblox_node/tsdf_voxel_size", voxel_size); esdf.data = new float[1];} // Constructor
    float voxel_size; // voxblox voxel size
    ros::NodeHandle n; // Ros node handle
    float position[3]; // robot position
    struct ESDF {
      float * data; // esdf matrix pointer
      int size[3]; // number of elements in each dimension
      float max[4]; // max and min values in each dimension
      float min[4];
    };
    ESDF esdf;
    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg); // Subscriber callback function
    void callback_position(const gazebo_msgs::LinkStates msg); // Subscriber callback for robot position
    int xyz_index3(const float point[3]);
};

void Msfm3d::callback_position(const gazebo_msgs::LinkStates msg)
{
  position[0] = msg.pose[102].position.x;
  position[1] = msg.pose[102].position.y;
  position[2] = msg.pose[102].position.z;
}

void Msfm3d::callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // clock object for timing the parsing operation
  clock_t tStart = clock();
  // integer array to store uint8 byte values
  int bytes[4];
  // local pointcloud storage array
  float xyzi[4*(msg->width)];
  // pointcloud xyz limits
  float xyzi_max[4], xyzi_min[4];
  
  // parse pointcloud2 data
  for (int i=0; i<(msg->width); i++) {
    for (int j=0; j<4; j++) {
      for (int k=0; k<4; k++) {
        bytes[k] = msg->data[32*i+(msg->fields[j].offset)+k];
      }
      xyzi[4*i+j] = byte2Float(bytes);
      if (i < 4){
        xyzi_max[j] = xyzi[4*i+j];
        xyzi_min[j] = xyzi[4*i+j];
      } else {
        if (xyzi[4*i+j] > xyzi_max[j]) xyzi_max[j] = xyzi[4*i+j];
        if (xyzi[4*i+j] < xyzi_min[j]) xyzi_min[j] = xyzi[4*i+j];
      }
    }
  }

  // Replace max, min, and size esdf properties
  for (int i=0; i<4; i++) { esdf.max[i] = xyzi_max[i]; esdf.min[i] = xyzi_min[i]; }
  for (int i=0; i<3; i++) esdf.size[i] = roundf((esdf.max[i]-esdf.min[i])/voxel_size);

  // Empty current esdf matrix and create a new one.
  delete esdf.data;
  esdf.data = new float [esdf.size[0]*esdf.size[1]*esdf.size[2]] { }; // Initialize all values to zero.
  
  // Parse xyzi into esdf_mat
  float point[3];
  for (int i=0; i<(4*(msg->width)); i=i+4) {
    point[0] = xyzi[i]; point[1] = xyzi[i+1]; point[2] = xyzi[i+2];
    esdf.data[xyz_index3(point)] = xyzi[i+3];
  }

  ROS_INFO("ESDF Updated!");
  ROS_INFO("Parsing Time taken: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

int Msfm3d::xyz_index3(const float point[3])
{
  int ind[3];
  for (int i=0; i<3; i++) ind[i] = roundf((point[i]-esdf.min[i])/voxel_size);
  return mindex3(ind[0], ind[1], ind[2], esdf.size[0], esdf.size[1]);
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
  // ros::NodeHandle n;

  Msfm3d planner;

  // Get voxblox voxel size parameter

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
  ros::Subscriber sub1 = planner.n.subscribe("/X1/voxblox_node/tsdf_pointcloud", 1000, &Msfm3d::callback, &planner);
  ros::Subscriber sub2 = planner.n.subscribe("/gazebo/link_states", 1000, &Msfm3d::callback_position, &planner);


  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  int i = 0;
  ros::Rate r(5); // 5 hz
  while (ros::ok())
  {
    ROS_INFO("X1 Position: [x: %f, y: %f, z: %f]", planner.position[0], planner.position[1], planner.position[2]);
    i = planner.xyz_index3(planner.position);
    ROS_INFO("Index at Position: %d", i);
    ROS_INFO("ESDF at Position: %f", planner.esdf.data[i]);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}