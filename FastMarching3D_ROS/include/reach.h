#ifndef REACH_H
#define REACH_H

#include <iostream>
// #include <numeric>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "msfm3d.c" // This header does not play well with eigen.
#include "utility.h"
#include "gain.h"

std::vector<Point> followGradientPath(const Point start, const Point goal, MapGrid3D<double> *reach);

float dist3(const float a[3], const float b[3]){
  float sum = 0.0;
  for (int i=0; i < 3; i++) sum += (b[i] - a[i])*(b[i] - a[i]);
  return std::sqrt(sum);
}

float dist3(const Point a, const Point b){
  float sum = 0.0;
  sum += (b.x - a.x)*(b.x - a.x);
  sum += (b.y - a.y)*(b.y - a.y);
  sum += (b.z - a.z)*(b.z - a.z);
  return std::sqrt(sum);
}

struct GoalPath {
  geometry_msgs::Pose goal;
  std::vector<Point> path;
  float gain, utility, cost;
};

float CostToTurnPath(geometry_msgs::Pose robot, geometry_msgs::Pose goal, std::vector<Point> path, float turn_rate) {
  if (path.size() < 5) return 0.0;

  // Get unity vectors at the start and end of the path
  Eigen::Vector3f path_start_vec, path_end_vec;
  path_start_vec << path[5].x - path[0].x,
                    path[5].y - path[0].y,
                    path[5].z - path[0].z;
  path_start_vec = path_start_vec/path_start_vec.norm();
  path_end_vec << path[path.size() - 1].x - path[path.size() - 6].x, 
                  path[path.size() - 1].y - path[path.size() - 6].y,
                  path[path.size() - 1].z - path[path.size() - 6].z;
  path_end_vec = path_end_vec/path_end_vec.norm();

  // Convert quaternion poses to unit vectors
  Quaternion q_robot, q_goal;
  q_robot.w = robot.orientation.w;
  q_robot.x = robot.orientation.x;
  q_robot.y = robot.orientation.y;
  q_robot.z = robot.orientation.z;
  q_goal.w = goal.orientation.w;
  q_goal.x = goal.orientation.x;
  q_goal.y = goal.orientation.y;
  q_goal.z = goal.orientation.z;
  Eigen::Matrix3f R_robot = Quaternion2RotationMatrix(q_robot);
  Eigen::Matrix3f R_goal = Quaternion2RotationMatrix(q_goal);
  Eigen::Vector3f robot_vec;
  robot_vec << R_robot(0,0), R_robot(1,0), R_robot(2,0);
  Eigen::Vector3f goal_vec;
  goal_vec << R_goal(0,0), R_goal(1,0), R_goal(2,0);
  
  float robot_to_start_angle = std::acos(robot_vec.dot(path_start_vec));
  float goal_to_end_angle = std::acos(goal_vec.dot(path_end_vec));

  return ((robot_to_start_angle + goal_to_end_angle)*(180.0/M_PI)/turn_rate);
}

float CostToTurnPath(geometry_msgs::Pose robot, std::vector<Point> path, float turn_rate) {
  if (path.size() < 5) return 0.0;

  // Get unity vectors at the start and end of the path
  Eigen::Vector3f path_start_vec;
  path_start_vec << path[5].x - path[0].x,
                    path[5].y - path[0].y,
                    path[5].z - path[0].z;
  path_start_vec = path_start_vec/path_start_vec.norm();

  // Convert quaternion poses to unit vectors
  Quaternion q_robot;
  q_robot.w = robot.orientation.w;
  q_robot.x = robot.orientation.x;
  q_robot.y = robot.orientation.y;
  q_robot.z = robot.orientation.z;
  Eigen::Matrix3f R_robot = Quaternion2RotationMatrix(q_robot);
  Eigen::Vector3f robot_vec;
  robot_vec << R_robot(0,0), R_robot(1,0), R_robot(2,0);
  
  float robot_to_start_angle = std::acos(robot_vec.dot(path_start_vec));

  return ((robot_to_start_angle)*(180.0/M_PI)/turn_rate);
}

float CalculatePathCost(std::vector<Point> path)
{
  float cost = 0.0;
  for (int i=1; i<path.size(); i++) cost += dist3(path[1], path[0]);
  return cost;
}

std::vector<GoalPath> reach(int source[3], geometry_msgs::Pose start, MapGrid3D<double> *speedMap, std::vector<float> goals, MapGrid3D<double> *reachOut,
const bool usesecond, const bool usecross, const int nGoalStop, const float turnRate = 1.0, const double timeOut = 1.0, const float minGoalSeparationDistance = 5.0)
{
    // ROS_INFO("Fast marching from (%0.2f, %0.2f, %0.2f)", (source[0]*speedMap->voxelSize)+speedMap->minBounds.x, (source[1]*speedMap->voxelSize)+speedMap->minBounds.y, (source[2]*speedMap->voxelSize)+speedMap->minBounds.z);
    // ROS_INFO("Speed map details - ");
    // ROS_INFO("Size = %d x %d x %d", speedMap->size.x, speedMap->size.y, speedMap->size.z);
    // ROS_INFO("Total voxels = %d", speedMap->voxels.size());
    // ROS_INFO("Min bounds: (%0.2f, %0.2f, %0.2f)", speedMap->minBounds.x, speedMap->minBounds.y, speedMap->minBounds.z);
    // ROS_INFO("Max bounds: (%0.2f, %0.2f, %0.2f)", speedMap->maxBounds.x, speedMap->maxBounds.y, speedMap->maxBounds.z);
    // ROS_INFO("goals details - ");
    // ROS_INFO("Total voxels = %d", goals.size());
    // ROS_INFO("Reach map details - ");
    // ROS_INFO("Size = %d x %d x %d", reachOut->size.x, reachOut->size.y, reachOut->size.z);
    // ROS_INFO("Total voxels = %d", reachOut->voxels.size());
    // ROS_INFO("Min bounds: (%0.2f, %0.2f, %0.2f)", reachOut->minBounds.x, reachOut->minBounds.y, reachOut->minBounds.z);
    // ROS_INFO("Max bounds: (%0.2f, %0.2f, %0.2f)", reachOut->maxBounds.x, reachOut->maxBounds.y, reachOut->maxBounds.z);
    // ROS_INFO("Max number of goals =  %d", nGoalStop);

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
    int npixels = speedMap->voxels.size();

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
    clock_t tStart = clock();

    // Parse input arguments to relevent function variables
    for (int i=0; i<3; i++){
        SourcePoints[i] = source[i];
    }
    dims[0] = speedMap->size.x;
    dims[1] = speedMap->size.y;
    dims[2] = speedMap->size.z;
    std::vector<GoalPath> goalsReachedSorted;
    F = &speedMap->voxels[0]; // Source esdf
    dims_sp[0] = 3;
    dims_sp[1] = 1;
    dims_sp[2] = 1;
    Ed = 0;
    T = &reachOut->voxels[0];

    // Initialize termination cost to very large number
    float terminationCost = 1e10;
    Point startPosition = {start.position.x, start.position.y, start.position.z};
    std::vector<float> gainsUnreached;
    for (i=0; i < goals.size(); i++){
      if (goals[i] > 0.01) gainsUnreached.push_back(goals[i]);
    }
    std::sort(gainsUnreached.begin(), gainsUnreached.end()); // Sorts lowest to highest for binary search
    ROS_INFO("Planning to %d goals.", gainsUnreached.size());
    // ROS_INFO("Goals have gains:");
    // for (int i=0; i < gainsUnreached.size(); i++) {
    //   if (i == (gainsUnreached.size()-1)) std::cout << gainsUnreached[i] << std::endl;
    //   else std::cout << gainsUnreached[i] << ", ";
    // }

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
                Tt=(1/(std::max(F[IJK_index],eps)));
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
        if(IsInf(neg_listv[index])) {
          break;
        }

        /*index=minarray(neg_listv, neg_pos); */
        x=(int)neg_listx[index]; y=(int)neg_listy[index]; z=(int)neg_listz[index];
        XYZ_index=mindex3(x, y, z, dims[0], dims[1]);
        Frozen[XYZ_index]=1;
        T[XYZ_index]=neg_listv[index];
        if(Ed) {
          Y[XYZ_index]=neg_listo[index];
        }

        /*Remove min value by replacing it with the last value in the array */
        list_remove_replace(listval, listprop, index) ;
        neg_listv=listval[listprop[1]-1];
        if(index<(neg_pos-1)) {
            neg_listx[index]=neg_listx[neg_pos-1];
            neg_listy[index]=neg_listy[neg_pos-1];
            neg_listz[index]=neg_listz[neg_pos-1];
            if(Ed) {
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
        // Check if the current voxel is a frontier and how far it is from the others reached so far
        if (goals[XYZ_index] > 0.1) {
          // Remove goal from list of unreached gains
          float gain = goals[XYZ_index];
          int unreachId = std::find(gainsUnreached.begin(), gainsUnreached.end(), gain) - gainsUnreached.begin();
          ROS_INFO("Found voxel with gain %0.1f.  Erased goal with id %d and gain %0.1f.", gain, unreachId, gainsUnreached[unreachId]);
          if ((unreachId >= 0) && (unreachId < gainsUnreached.size())) gainsUnreached.erase(gainsUnreached.begin() + unreachId);
          
          // Calculate path, cost and utility from the path to get there
          Point newGoalPosition = speedMap->_ConvertIndexToPosition(XYZ_index);
          std::vector<Point> newPath = followGradientPath(startPosition, newGoalPosition, reachOut);
          float cost;
          if (newPath.size() == 0) cost = 1e10;
          // else cost = CalculatePathCost(newPath) + CostToTurnPath(start, newPath, turnRate);
          else cost = T[XYZ_index] + CostToTurnPath(start, newPath, turnRate);
          ROS_INFO("Fast Marching cost: %0.1f, true cost: %0.1f (path = %0.1f, turn = %0.1f)", T[XYZ_index], cost, CalculatePathCost(newPath), CostToTurnPath(start, newPath, turnRate));
          float utility = Utility(gain, cost);

          // Add point to goalsReached
          if (goalsReachedSorted.size() == 0) {
            GoalPath newGoal;
            newGoal.goal.position.x = newGoalPosition.x;
            newGoal.goal.position.y = newGoalPosition.y;
            newGoal.goal.position.z = newGoalPosition.z;
            newGoal.utility = utility;
            newGoal.gain = gain;
            newGoal.cost = cost;
            newGoal.path = newPath;
            goalsReachedSorted.push_back(newGoal);
            // ROS_INFO("Added first reached goal to list");
          }
          else {
            // If the current cell is outside minGoalSeparationDistance of the others in the list, then add it.
            // Erase all reached goals if their utilities are worse and they're not separated enough from the current goal.
            std::vector<bool> separated(goalsReachedSorted.size());
            std::vector<bool> smallerUtility(goalsReachedSorted.size());
            int insertId = -1;
            std::vector<int> eraseIds;
            for (int i=0; i < goalsReachedSorted.size(); i++) {
              GoalPath listGoal = goalsReachedSorted[i];
              Point listGoalPosition = {listGoal.goal.position.x, listGoal.goal.position.y, listGoal.goal.position.z};
              separated[i] = (dist3(listGoalPosition, newGoalPosition) <= minGoalSeparationDistance);
              smallerUtility[i] = (utility > goalsReachedSorted[i].utility);
            }
            // ROS_INFO("Compared current goal to all others in reach list");
            for (int i=0; i < goalsReachedSorted.size(); i++) {
              if ((!smallerUtility[i]) && (!separated[i])) {
                // Goal is too close to one that is better, don't add it
                break;
              }
              if (smallerUtility[i]) {
                if (separated[i] && (insertId == -1)) insertId = i;
                if (!separated[i]) eraseIds.push_back(i);
              } else {
                continue;
              }
            }
            if (insertId >= 0) {
              GoalPath newGoal;
              newGoal.goal.position.x = newGoalPosition.x;
              newGoal.goal.position.y = newGoalPosition.y;
              newGoal.goal.position.z = newGoalPosition.z;
              newGoal.utility = utility;
              newGoal.gain = gain;
              newGoal.cost = cost;
              newGoal.path = newPath;
              goalsReachedSorted.insert(goalsReachedSorted.begin() + insertId, newGoal);
              // ROS_INFO("Added new reached goal at position %d", insertId);
            }
            // ROS_INFO("Erasing %d goals that are worse and too close to current goal.", eraseIds.size());
            for (int i=0; i<eraseIds.size(); i++) {
              goalsReachedSorted.erase(goalsReachedSorted.begin() + eraseIds[eraseIds.size() - i - 1]); // Erase in reverse order
            }
          }
          // Assign termination cost
          if (gainsUnreached.size() > 0) {
            if (goalsReachedSorted.size() >= nGoalStop) {
              // Cost associated with nth best utility and best remaining gain guarantees the best n goals have been found.
              terminationCost = CostFromUtilityGain(gainsUnreached.back(), goalsReachedSorted[nGoalStop-1].utility);
            }
            else {
              terminationCost = 1e10;
            }
          }
          else {
            // No unreached goals, end fast marching
            terminationCost = 0.0;
          }
          // ROS_INFO("Termination cost is %0.2f", terminationCost);
        }
        if (terminationCost < T[XYZ_index]) {
          ROS_INFO("Reachability calculation reached %d goals after %0.3f seconds.", nGoalStop, (double)(clock() - tStart)/CLOCKS_PER_SEC);
          break;
        }
        if (((double)(clock() - tStart)/CLOCKS_PER_SEC) >= timeOut) {
          ROS_INFO("Reachability calculation timed out after %0.3f seconds.", (double)(clock() - tStart)/CLOCKS_PER_SEC);
          break;
        }
    }
    /* Free memory */
    /* Destroy parameter list */
    destroy_list(listval, listprop);
    free(neg_listx);
    free(neg_listy);
    free(neg_listz);
    delete[] Frozen;
    return goalsReachedSorted;
}

struct Neighbor
{
  int id = -1;
  float cost = 0.0;
};

struct Node
{
  int id = -1;
  int parent = -1;
  float g = 1e10;
  float h;
  float f = 1e10;
  std::vector<Neighbor> neighbors;
  Point position;
};

std::vector<Neighbor> getNeighbors(int id, MapGrid3D<double> *reach, MapGrid3D<double> *speedMap)
{
  std::vector<Neighbor> neighborsInitial;
  std::vector<Neighbor> neighbors;
  Neighbor n;
  int dx = 1, dy = reach->size.x, dz = reach->size.x*reach->size.y;
  n.id = id + dx; n.cost = 1.0; neighborsInitial.push_back(n);
  n.id = id - dx; neighborsInitial.push_back(n);
  n.id = id + dy; neighborsInitial.push_back(n);
  n.id = id - dy; neighborsInitial.push_back(n);
  n.id = id + dz; neighborsInitial.push_back(n);
  n.id = id - dz; neighborsInitial.push_back(n);
  // Cross neighbors in x-y plane
  n.id = id + dx + dy; n.cost = 1.4; neighborsInitial.push_back(n);
  n.id = id + dx - dy; neighborsInitial.push_back(n);
  n.id = id - dx + dy; neighborsInitial.push_back(n);
  n.id = id - dx - dy; neighborsInitial.push_back(n);

  // Filter out neighbors that are too close to obstacles or aren't "reachable"
  for (int i=0; i<10; i++) {
    if ((speedMap->voxels[neighborsInitial[i].id] >= (double)speedMap->voxelSize) && (reach->voxels[neighborsInitial[i].id] > 0)) {
      neighbors.push_back(neighborsInitial[i]);
    }
  }

  return neighbors;
}

std::vector<Point> reconstructPath(std::vector<Node> visited, Node end) {
  std::vector<Point> path;
  Node current = end;
  path.push_back(current.position);
  while (current.parent != -1) {
    current = visited[current.parent];
    path.push_back(current.position);
  }
  return path;
}

std::vector<Point> flipPath(std::vector<Point> path) {
  std::vector<Point> pathFlipped;
  for (int i=0; i<path.size(); ++i){
    pathFlipped.push_back(path[path.size()-i-1]);
  }
  return pathFlipped;
}

std::vector<Point> AStar(const Point start, const Point goal, MapGrid3D<double> *reach, MapGrid3D<double> *speedMap)
{
  // This is a standard implimentation of A* from goal to start.
  int npixels = reach->voxels.size(); // size of the reachability array, index in reachability array of the current path voxel.
  int goalIdx = reach->_ConvertPositionToIndex(goal);
  std::vector<Point> emptyPath;
  if (goalIdx < 0 || goalIdx > npixels){
    ROS_INFO("Goal point is not reachable.");
    return emptyPath;
  }
  if (reach->voxels[goalIdx] <= 0.0 || reach->voxels[goalIdx] >= 1e12) {
    ROS_INFO("Goal point is either too far away or is blocked by an obstacle.");
    return emptyPath;
  }
  ROS_INFO("Attempting to find path to [%0.1f, %0.1f, %0.1f] from [%0.1f, %0.1f, %0.1f].", goal.x, goal.y, goal.z, start.x, start.y, start.z);
  std::vector<int> nodeIdList(npixels, -1);
  std::vector<Node> visited;

  // Initialize a-star from goal position back to start.
  Node startNode;
  startNode.id = goalIdx;
  startNode.g = 0.0;
  startNode.h = reach->voxels[goalIdx];
  // start.h = dist3(goal, position)/esdf.data[goal_idx];
  startNode.f = startNode.g + startNode.h;
  startNode.neighbors = getNeighbors(startNode.id, reach, speedMap);
  startNode.parent = -1;
  startNode.position = goal;
  
  visited.push_back(startNode);
  nodeIdList[startNode.id] = visited.size()-1;

  std::vector<Node> openSet; // priority queue of next nodes
  openSet.push_back(startNode);
  int itt = 0;
  while (openSet.size()>0) {
    itt++;
    Node current = openSet.back();
    // if (itt < 15) ROS_INFO("Current node, [%0.1f, %0.1f, %0.1f], g = %0.2f, h = %0.2f, f = %0.2f, has %d neighbors", current.position.x, current.position.y, current.position.z, current.g, current.h, current.f, current.neighbors.size());
    // if (current.id == xyz_index3(position)) {
    if (dist3(current.position, start) <= 1.1*speedMap->voxelSize) {
      // Stop planning, the robot's current position (the goal) has been reached.
      std::vector<Point> path = reconstructPath(visited, current);
      ROS_INFO("Path of length %d generated in %d iterations.", path.size(), itt);
      return path;
    }
    openSet.pop_back();
    for (int i=0; i<current.neighbors.size(); i++) {
      int neighbor_id = current.neighbors[i].id;
      bool neighbor_is_new = false;
      Node neighbor;
      if (nodeIdList[neighbor_id] == -1) {
        neighbor_is_new = true;
        neighbor.id = neighbor_id;
        neighbor.position = reach->_ConvertIndexToPosition(neighbor.id);
        neighbor.h = reach->voxels[neighbor.id];
        // neighbor.h = dist3(neighbor.position, position)/esdf.data[neighbor_id];
        neighbor.neighbors = getNeighbors(neighbor.id, reach, speedMap);
        nodeIdList[neighbor.id] = visited.size();
        visited.push_back(neighbor);
      }
      int visit_id = nodeIdList[neighbor_id];
      // float tentative_g = current.g + dist3(current.position, visited[visit_id].position)/(speedMap->voxels[neighbor_id]*speedMap->voxelSize);
      float tentative_g = current.g + current.neighbors[i].cost/(speedMap->voxels[neighbor_id]);
      // float tentative_g = current.g + (current.h - neighbor.h);
      if (visited[visit_id].g > tentative_g) {
        // Change the neighbors parent to the current
        visited[visit_id].parent = nodeIdList[current.id];
        visited[visit_id].g = tentative_g;
        visited[visit_id].f = tentative_g + visited[visit_id].h;
        if (neighbor_is_new) {
          int j;
          for (j=0; j<openSet.size(); j++) {
            if (openSet[j].f < visited[visit_id].f) break;
          }
          openSet.insert(openSet.begin() + j, visited[visit_id]);
        }
      }
    }
  }
  ROS_INFO("Not able to find the start position after running path updater for %d iterations.", itt);
  return emptyPath;
}

std::vector<Point> followGradientPath(const Point start, const Point goal, MapGrid3D<double> *reach)
{
  // Assumptions:
  // The reach voxel map is the cost-to-go from the goal back to the start.
  // All neighboring voxels to the desired path are inside the map and have a nonzero and noninfinite cost-to-go.
  std::vector<Point> path;
  path.push_back(goal);

  float sobelKernel_x[27] = {1.0, 0.0, -1.0, 2.0, 0.0, -2.0, 1.0, 0.0, -1.0,
                            2.0, 0.0, -2.0, 4.0, 0.0, -4.0, 2.0, 0.0, -2.0,
                            1.0, 0.0, -1.0, 2.0, 0.0, -2.0, 1.0, 0.0, -1.0};
  float sobelKernel_y[27] = {1.0, 2.0, 1.0, 0.0, 0.0, 0.0, -1.0, -2.0, -1.0,
                            2.0, 4.0, 2.0, 0.0, 0.0, 0.0, -2.0, -4.0, -2.0,
                            1.0, 2.0, 1.0, 0.0, 0.0, 0.0, -1.0, -2.0, -1.0};
  float sobelKernel_z[27] = {1.0, 2.0, 1.0, 2.0, 4.0, 2.0, 1.0, 2.0, 1.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            -1.0, -2.0, -1.0, -2.0, -4.0, -2.0, -1.0, -2.0, -1.0};

  // 3D Interpolation intermediate values from https://en.wikipedia.org/wiki/Trilinear_interpolation
  float voxelSize = reach->voxelSize;
  float step = 0.5*voxelSize;
  int goalId = reach->_ConvertPositionToIndex(goal);
  float maxReach = reach->voxels[goalId] + 2.0;

  // Run loop until the path is within a voxel of the robot.
  Point current = goal;
  float gradNorm = 1.0;
  float reachValue = 0.0;
  float cost = 0.0;
  while ((dist3(current, start) > 2.0*voxelSize) && (path.size() < 2000) && gradNorm >= 0.001) {
  	// Find the current point's grid indices and it's 26 neighbor voxel indices.
  	int currentId = reach->_ConvertPositionToIndex(current);
    // ROS_INFO("Current point is (%0.2f, %0.2f, %0.2f with cost-to-go of %0.2f)", current.x, current.y, current.z, reach->voxels[currentId]);
    Point grad = {0.0, 0.0, 0.0};
    Point neighbor = {0.0, 0.0, 0.0};
    for (int i=0; i<3; i++) {
      neighbor.x = current.x + (float)(i-1)*voxelSize;
      for (int j=0; j<3; j++) {
        neighbor.y = current.y + (float)(j-1)*voxelSize;
        for (int k=0; k<3; k++) {
          neighbor.z = current.z + (float)(k-1)*voxelSize;
          int voxel_id = reach->_ConvertPositionToIndex(neighbor);
          int kernel_id = i + 3*j + 9*k;
          // ROS_INFO("Neighbor %d (%d, %d, %d) is (%0.1f, %0.1f, %0.1f) with index %d", kernel_id, i-1, j-1, k-1, neighbor[0], neighbor[1], neighbor[2], voxel_id);
          if (reach->voxels[voxel_id] <= 0.0) {
            reachValue = reach->voxels[currentId];
          } else {
            reachValue = reach->voxels[voxel_id];
          }
          // ROS_INFO("Reach value = %0.2f", reachValue);
          grad.x += sobelKernel_x[kernel_id]*reachValue;
          grad.y += sobelKernel_y[kernel_id]*reachValue;
          grad.z += sobelKernel_z[kernel_id]*reachValue;
        }
      }
    }

    // Normalize the size of the gradient vector if it is too large
    gradNorm = std::sqrt(grad.x*grad.x + grad.y*grad.y + grad.z*grad.z);
    if (gradNorm > 1.0){
      grad.x = grad.x/gradNorm;
      grad.y = grad.y/gradNorm;
      grad.z = grad.z/gradNorm;
    }
    if (gradNorm < 0.05){
      grad.x = std::sqrt(0.05)*grad.x/gradNorm;
      grad.y = std::sqrt(0.05)*grad.y/gradNorm;
      grad.z = std::sqrt(0.05)*grad.z/gradNorm;
    }

    cost += step*std::sqrt(grad.x*grad.x + grad.y*grad.y + grad.z*grad.z);

    current.x += step*grad.x;
    current.y += step*grad.y;
    current.z += step*grad.z;
    path.push_back(current);
  }

  // Check if the path made it back to the vehicle
  if (dist3(path[path.size()-1], start) > 3.0*reach->voxelSize) {
    std::vector<Point> errorPath;
    // ROS_INFO("Path did not make it back to the robot.  Select a different goal point.");
    return errorPath;
  } else {
    path.push_back(start);
  }
  // ROS_INFO("Path found of length %d", path.size());
  return flipPath(path);
}

int indexofSmallestElement(double array[], int size)
{
    int index = 0;

    for(int i = 1; i < size; i++)
    {
        if(array[i] < array[index])
            index = i;              
    }

    return index;
}

std::vector<std::pair<int,float>> getNeighborIds2D(int id, MapGrid3D<double> *reach)
{
  std::vector<int> neighbors;
  int dx = 1; int dy = reach->size.x; int dz = reach->size.x*reach->size.y;
  neighbors.push_back(id + dx + dy); neighbors.push_back(id + dy); neighbors.push_back(id - dx + dy);
  neighbors.push_back(id + dx); neighbors.push_back(id - dx);
  neighbors.push_back(id + dx - dy); neighbors.push_back(id - dy); neighbors.push_back(id - dx - dy);
  
  std::vector<std::pair<int,float>> neighborsMinReach;
  for (int i=0; i<neighbors.size(); i++) {
    double zNeighborReachValues[3] = {reach->voxels[neighbors[i]-dz], reach->voxels[neighbors[i]], reach->voxels[neighbors[i]+dz]};
    int zShift = indexofSmallestElement(zNeighborReachValues, 3) - 1;
    neighborsMinReach.push_back(std::make_pair(neighbors[i] + zShift*dz, zShift*reach->voxelSize));
  }
  return neighborsMinReach;
}

bool checkDuplicates(std::vector<int> list)
{
  std::sort(list.begin(), list.end());
  for (int i=0; i<list.size()-1; i++) {
    if (list[i] == list[i+1]) return true;
  }
  return false;
}

std::vector<Point> followGradientPathManifold2D(const Point start, const Point goal, MapGrid3D<double> *reach)
{
  std::vector<Point> path;
  path.push_back(goal);
  double sobelX[8];
  sobelX[0] = 1.0; sobelX[1] = 0.0; sobelX[2] = -1.0;
  sobelX[3] = 2.0; sobelX[4] = -2.0;
  sobelX[5] = 1.0; sobelX[6] = 0.0; sobelX[7] = -1.0;
  double sobelY[8];
  sobelY[0] = 1.0; sobelY[1] = 2.0; sobelY[2] = 1.0;
  sobelY[3] = 0.0; sobelY[4] = 0.0;
  sobelY[5] = -1.0; sobelY[6] = -2.0; sobelY[7] = -1.0;

  std::vector<int> neighborIntervals;
  int dx = 1; int dy = reach->size.x;
  neighborIntervals.push_back(dx + dy); neighborIntervals.push_back(+dy); neighborIntervals.push_back(dx + dy);
  neighborIntervals.push_back(dx); neighborIntervals.push_back(-dx);
  neighborIntervals.push_back(dx - dy); neighborIntervals.push_back(-dy); neighborIntervals.push_back(dx - dy);

  Point current = goal;
  float gradMag = 100;
  int itt= 0;
  int ittMax = (round(dist3(start,goal)/(reach->voxelSize)) + 10)*15;
  while ((dist3(start, current) > 2.0*reach->voxelSize) && (gradMag > 0.01) && (itt < ittMax)) {
    float gradX = 0.0, gradY = 0.0;
    int currentId = reach->_ConvertPositionToIndex(current);
    std::vector<std::pair<int,float>> neighbors = getNeighborIds2D(currentId, reach);
    for (int i=0; i<8; i++) {
      double reachNeighbor = reach->voxels[neighbors[i].first];
      gradX += sobelX[i]*reachNeighbor;
      gradY += sobelY[i]*reachNeighbor;
    }
    gradMag = std::sqrt(gradX*gradX + gradY*gradY);
    current.x += 0.5*std::min(std::max((double)gradX,0.05),1.0)*reach->voxelSize;
    current.y += 0.5*std::min(std::max((double)gradY,0.05),1.0)*reach->voxelSize;

    // Figure out if the current is now a new id and which neighbor voxel it went to.
    
    int intermediateId = reach->_ConvertPositionToIndex(current);
    int diffId = intermediateId - currentId;
    if (diffId != 0) {
      for (int i=0; i<8; i++) {
        if (neighborIntervals[i] == diffId) {
          current.z += neighbors[i].second;
          break;
        }
      }
    }
    path.push_back(current);
    itt++;
  }
  
  // Check if the path made it back to the vehicle
  if (dist3(path[path.size()-1], start) > 3.0*reach->voxelSize) {
    ROS_INFO("Path did not make it back to the robot.  Select a different goal point.");
  } else {
    path.push_back(start);
  }
  return flipPath(path);
}

std::vector<Point> followGradientPath2D(const Point start, const Point goal, MapGrid3D<double> *reach)
{
  std::vector<Point> emptyPath;
  if (reach->size.z > 1) return emptyPath; // Expects reach map to be 2D
  std::vector<Point> path;
  path.push_back(goal);
  double sobelX[8];
  sobelX[0] = 1.0; sobelX[1] = 0.0; sobelX[2] = -1.0;
  sobelX[3] = 2.0; sobelX[4] = -2.0;
  sobelX[5] = 1.0; sobelX[6] = 0.0; sobelX[7] = -1.0;
  double sobelY[8];
  sobelY[0] = 1.0; sobelY[1] = 2.0; sobelY[2] = 1.0;
  sobelY[3] = 0.0; sobelY[4] = 0.0;
  sobelY[5] = -1.0; sobelY[6] = -2.0; sobelY[7] = -1.0;

  Point current = goal;
  float gradMag = 100;
  int itt= 0;
  int ittMax = (round(dist3(start,goal)/(reach->voxelSize)) + 10)*15;
  while ((dist3(start, current) > 2.0*reach->voxelSize) && (gradMag > 0.01) && (itt < ittMax)) {
    float gradX = 0.0, gradY = 0.0;
    int id = reach->_ConvertPositionToIndex(current);
    std::vector<int> neighbors;
    int dx = 1; int dy = reach->size.x; int dz = (reach->size.x)*(reach->size.y);
    neighbors.push_back(id + dx + dy); neighbors.push_back(id + dy); neighbors.push_back(id - dx + dy);
    neighbors.push_back(id + dx); neighbors.push_back(id - dx);
    neighbors.push_back(id + dx - dy); neighbors.push_back(id - dy); neighbors.push_back(id - dx - dy);

    for (int i=0; i<8; i++) {
      double reachNeighbor = reach->voxels[neighbors[i]];
      gradX += sobelX[i]*reachNeighbor;
      gradY += sobelY[i]*reachNeighbor;
    }
    gradMag = std::sqrt(gradX*gradX + gradY*gradY);
    current.x += 0.5*std::min(std::max((double)gradX,0.05),1.0)*reach->voxelSize;
    current.y += 0.5*std::min(std::max((double)gradY,0.05),1.0)*reach->voxelSize;
    path.push_back(current);
    itt++;
  }
  
  // Check if the path made it back to the vehicle
  if (dist3(path[path.size()-1], start) > 3.0*reach->voxelSize) {
    ROS_INFO("Path did not make it back to the robot.  Select a different goal point.");
  } else {
    path.push_back(start);
  }
  return flipPath(path);
}

void reachRaw(int source[3], MapGrid3D<double> *speedMap, MapGrid3D<double> *reachOut, const bool usesecond, const bool usecross, const double timeOut = 1.0)
{
    // ROS_INFO("Fast marching from (%0.2f, %0.2f, %0.2f)", (source[0]*speedMap->voxelSize)+speedMap->minBounds.x, (source[1]*speedMap->voxelSize)+speedMap->minBounds.y, (source[2]*speedMap->voxelSize)+speedMap->minBounds.z);
    // ROS_INFO("Speed map details - ");
    // ROS_INFO("Size = %d x %d x %d", speedMap->size.x, speedMap->size.y, speedMap->size.z);
    // ROS_INFO("Total voxels = %d", speedMap->voxels.size());
    // ROS_INFO("Min bounds: (%0.2f, %0.2f, %0.2f)", speedMap->minBounds.x, speedMap->minBounds.y, speedMap->minBounds.z);
    // ROS_INFO("Max bounds: (%0.2f, %0.2f, %0.2f)", speedMap->maxBounds.x, speedMap->maxBounds.y, speedMap->maxBounds.z);
    // ROS_INFO("goals details - ");
    // ROS_INFO("Total voxels = %d", goals.size());
    // ROS_INFO("Reach map details - ");
    // ROS_INFO("Size = %d x %d x %d", reachOut->size.x, reachOut->size.y, reachOut->size.z);
    // ROS_INFO("Total voxels = %d", reachOut->voxels.size());
    // ROS_INFO("Min bounds: (%0.2f, %0.2f, %0.2f)", reachOut->minBounds.x, reachOut->minBounds.y, reachOut->minBounds.z);
    // ROS_INFO("Max bounds: (%0.2f, %0.2f, %0.2f)", reachOut->maxBounds.x, reachOut->maxBounds.y, reachOut->maxBounds.z);
    // ROS_INFO("Max number of goals =  %d", nGoalStop);

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
    int npixels = speedMap->voxels.size();

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
    clock_t tStart = clock();

    // Parse input arguments to relevent function variables
    for (int i=0; i<3; i++){
        SourcePoints[i] = source[i];
    }
    dims[0] = speedMap->size.x;
    dims[1] = speedMap->size.y;
    dims[2] = speedMap->size.z;
    F = &speedMap->voxels[0]; // Source esdf
    dims_sp[0] = 3;
    dims_sp[1] = 1;
    dims_sp[2] = 1;
    Ed = 0;
    T = &reachOut->voxels[0];

    // Initialize termination cost to very large number
    float terminationCost = 1e10;

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
                Tt=(1/(std::max(F[IJK_index],eps)));
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
        if(IsInf(neg_listv[index])) {
          break;
        }

        /*index=minarray(neg_listv, neg_pos); */
        x=(int)neg_listx[index]; y=(int)neg_listy[index]; z=(int)neg_listz[index];
        XYZ_index=mindex3(x, y, z, dims[0], dims[1]);
        Frozen[XYZ_index]=1;
        T[XYZ_index]=neg_listv[index];
        if(Ed) {
          Y[XYZ_index]=neg_listo[index];
        }

        /*Remove min value by replacing it with the last value in the array */
        list_remove_replace(listval, listprop, index) ;
        neg_listv=listval[listprop[1]-1];
        if(index<(neg_pos-1)) {
            neg_listx[index]=neg_listx[neg_pos-1];
            neg_listy[index]=neg_listy[neg_pos-1];
            neg_listz[index]=neg_listz[neg_pos-1];
            if(Ed) {
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
        // Check if the reach grid is done calculating
        if (terminationCost < T[XYZ_index]) {
          ROS_INFO("Reachability calculation finished after %0.3f seconds.", (double)(clock() - tStart)/CLOCKS_PER_SEC);
          break;
        }
        if (((double)(clock() - tStart)/CLOCKS_PER_SEC) >= timeOut) {
          ROS_INFO("Reachability calculation timed out after %0.3f seconds.", (double)(clock() - tStart)/CLOCKS_PER_SEC);
          break;
        }
    }
    /* Free memory */
    /* Destroy parameter list */
    destroy_list(listval, listprop);
    free(neg_listx);
    free(neg_listy);
    free(neg_listz);
    delete[] Frozen;
    return;
}

#endif