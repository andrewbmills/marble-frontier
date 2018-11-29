// g++ msfm3d_node.cpp -g -o msfm3d_node.o -I /opt/ros/melodic/include -I /usr/include/c++/7.3.0 -L /opt/ros/melodic/lib -Wl,-rpath,/opt/ros/melodic/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization


#include <ros/ros.h>
// #include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <gazebo_msgs/LinkStates.h>
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

//Msfm3d class declaration
class Msfm3d
{
  public:
    // Constructor
    Msfm3d() {
    // n.getParam("/X1/voxblox_node/tsdf_voxel_size", voxel_size);
    reach = NULL;
    esdf.data = NULL;
    esdf.seen = NULL;
    }
    bool receivedPosition = 0;
    bool receivedPointCloud = 0;
    bool newCloud = 0;
    float voxel_size; // voxblox voxel size
    // ros::NodeHandle n; // Ros node handle
    float position[3]; // robot position
    double * reach; // reachability grid (output from reach())
    sensor_msgs::PointCloud2 PC2msg;
    // float * seen; // whether or not a grid cell has been seen by a sensor
    // std::vector<float> seen; // whether or not a grid cell has been seen by a sensor
    struct ESDF {
      double * data; // esdf matrix pointer
      bool * seen; // seen matrix pointer
      int size[3]; // number of elements in each dimension
      float max[4]; // max and min values in each dimension
      float min[4];
    };
    ESDF esdf;

    void callback(sensor_msgs::PointCloud2 msg); // Subscriber callback function
    void callback_position(const gazebo_msgs::LinkStates msg); // Subscriber callback for robot position
    void parsePointCloud(); // Function to parse pointCloud2 into a format that msfm3d can use
    int xyz_index3(const float point[3]);
    void index3_xyz(const int index, float point[3]);
};

void Msfm3d::callback_position(const gazebo_msgs::LinkStates msg)
{
  ROS_INFO("Getting vehicle position...");
  if (!receivedPosition) receivedPosition = 1;
  position[0] = msg.pose[102].position.x;
  position[1] = msg.pose[102].position.y;
  position[2] = msg.pose[102].position.z;
  ROS_INFO("Robot position updated!");
}

void Msfm3d::callback(sensor_msgs::PointCloud2 msg)
{
  if (!receivedPointCloud) receivedPointCloud = 1;
  PC2msg = msg;
  newCloud = 1;
}

void Msfm3d::parsePointCloud()
{
  if (newCloud) {
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
          } else {
            if (xyzis[5*i+j] > xyzi_max[j]) xyzi_max[j] = xyzis[5*i+j];
            if (xyzis[5*i+j] < xyzi_min[j]) xyzi_min[j] = xyzis[5*i+j];
          }
        }
      }
    }

    // Replace max, min, and size esdf properties
    for (int i=0; i<4; i++) { esdf.max[i] = xyzi_max[i]; esdf.min[i] = xyzi_min[i]; }
    for (int i=0; i<3; i++) esdf.size[i] = roundf((esdf.max[i]-esdf.min[i])/voxel_size);

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
      esdf.data[index] = (double)(xyzis[i+3]);
      esdf.seen[index] = (xyzis[i+4]>0.0);
    }

    ROS_INFO("ESDF Updated!");
    newCloud = 0; // Parsed the latest pointCloud2 object
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

void findFrontier(Msfm3d& planner, float frontierList[15], double cost[5]){
  // findFrontier scans through the environment arrays and returns the 5 closest frontier locations in euclidian distance.
  int npixels = planner.esdf.size[0]*planner.esdf.size[1]*planner.esdf.size[2];
  float point[3];
  float query[3];
  int neighbor[6];
  bool frontier = 0;
  int slot = 0;
  for (int i=0; i<5; i++) cost[i] = 1e5;
  for (int i=0; i<npixels; i++){
    // Check if the voxel has been seen, is unoccupied and it costs less to reach it than the 5 cheapest voxels
    if (planner.esdf.seen[i] && (planner.reach[i]<cost[0]) && (planner.esdf.data[i]>0)){
      // Check if the voxel is a frontier by querying adjacent voxels
      planner.index3_xyz(i, point);
      for (int j=0; j<3; j++) query[j] = point[j];
      // Create an array of neighbor indices
      for (int j=0; j<3; j++){
        if (point[j] < (planner.esdf.max[j] - planner.voxel_size)) query[j] = point[j] + planner.voxel_size;
        neighbor[2*j] = planner.xyz_index3(query);
        if (point[j] > (planner.esdf.min[j] + planner.voxel_size)) query[j] = point[j] - planner.voxel_size;
        neighbor[2*j+1] = planner.xyz_index3(query);
        query[j] = point[j];
      }
      // Check if the neighbor indices are unseen voxels
      for (int j=0; j<6; j++){
        if (!planner.esdf.seen[neighbor[j]]) frontier = 1;
      }
      // If the current voxel is a frontier, add the current voxel location to the cost and frontierList
      if (frontier) {
        frontier = 0; // reset for next loop
        // Put the cost into the correct slot.
        for (int j=0; j<5; j++) {
          if (planner.reach[i]<cost[j]) slot = j;
        }
        for (int j=0; j<slot; j++) {
          cost[j] = cost[j+1];
          for (int k=0; k<3; k++) frontierList[3*j+k] = frontierList[3*(j+1)+k];
        }
        cost[slot] = planner.reach[i];
        for (int j=0; j<3; j++) frontierList[j] = point[j];
      }
    }
  }
}

void reach( Msfm3d& planner, const bool usesecond, const bool usecross) {

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
    planner.reach = new double [npixels];
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
    }
    /* Free memory */
    /* Destroy parameter list */
    destroy_list(listval, listprop);
    free(neg_listx);
    free(neg_listy);
    free(neg_listz);
    free(Frozen);
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
  Msfm3d planner;

  // Get voxblox voxel size parameter
  n.getParam("/X1/voxblox_node/tsdf_voxel_size", planner.voxel_size);

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
  ros::Subscriber sub1 = n.subscribe("/X1/voxblox_node/tsdf_pointcloud", 1, &Msfm3d::callback, &planner);
  ros::Subscriber sub2 = n.subscribe("/gazebo/link_states", 1, &Msfm3d::callback_position, &planner);

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
  r.sleep();
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
    ROS_INFO("Planner Okay.");
    planner.parsePointCloud();
    // Heartbeat status update
    if (planner.receivedPosition){
      ROS_INFO("X1 Position: [x: %f, y: %f, z: %f]", planner.position[0], planner.position[1], planner.position[2]);
      i = planner.xyz_index3(planner.position);
      ROS_INFO("Index at Position: %d", i);
      if (!planner.newCloud){
        ROS_INFO("ESDF at Position: %f", planner.esdf.data[i]);
        
        // Call msfm3d function
        tStart = clock();
        ROS_INFO("Reachability matrix calculating...");
        reach(planner, 1, 1);
        ROS_INFO("Reachability Grid Calculated in: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);
        // ROS_INFO("Reachability matrix calculated.");

        // Find frontiers
        findFrontier(planner, frontierList, frontierCost);
        for (int i=0; i<5; i++) ROS_INFO("Frontier Position: [x: %f, y: %f, z: %f, cost: %f]", frontierList[3*i], frontierList[3*i+1], frontierList[3*i+2], frontierCost[i]);

        // Output reach matrix to .csv
        // if (spins < 2) {
        //   FILE * myfile;
        //   // myfile = fopen("sensed.csv", "w");
        //   myfile = fopen("esdf.csv", "w");
        //   npixels = planner.esdf.size[0]*planner.esdf.size[1]*planner.esdf.size[2];
        //   fprintf(myfile, "%d, %d, %d, %f\n", planner.esdf.size[0], planner.esdf.size[1], planner.esdf.size[2], planner.voxel_size);
        //   // for (int i=0; i<npixels-1; i++) fprintf(myfile, "%f, ", planner.reach[i]);
        //   // fprintf(myfile, "%f", planner.reach[npixels]);
        //   for (int i=0; i<npixels-1; i++) fprintf(myfile, "%f, ", planner.esdf.data[i]);
        //   fprintf(myfile, "%f", planner.esdf.data[npixels]);
        //   // for (int i=0; i<npixels-1; i++) fprintf(myfile, "%d, ", planner.seen[i]);
        //   // fprintf(myfile, "%d", planner.seen[npixels]);
        //   fclose(myfile);
        //   spins++;
        // }
      }
    }
  }

  return 0;
}