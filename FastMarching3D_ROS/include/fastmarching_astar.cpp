#include "micropather.cpp"
#include "math.h"
#include <cstdint>

typedef struct Point {
  float x, y, z;
} point_t;

class Map: public Graph {
  public:
    micropather::MicroPather* astar;
    micropather::MPVector<void*> path;
    Map() {
      astar = new micropather::MicroPather(this, 1000, 26);
    }
    ~Map() {
      delete astar;
    }
    int size[3];
    float map_min[3];
    double* reach;
    double* esdf;
    float voxel_size;
    int goal_id = 0;

    int pointToId(point_t p)
    {
      int id;
      id = std::roundf((p.z - map_min[2])/voxel_size)*size[0]*size[1];
      id += std::roundf((p.y - map_min[1])/voxel_size)*size[0];
      id += std::roundf((p.x - map_min[0])/voxel_size);
      return id;
    }

    point_t idToPoint(int id)
    {
      point_t p;
      p.z = map_min[2] + (id/(size[1]*size[0]))*voxel_size;
      p.y = map_min[1] + ((id % (size[1]*size[0]))/size[0])*voxel_size;
      p.x = map_min[0] + ((id % (size[1]*size[0])) % size[0])*voxel_size;
      return p;
    }

    virtual float LeastCostEstimate(void* stateStart, void* stateEnd)
    {
      int start_id = *((int*)(&stateStart));
      int end_id = *((int*)(&stateEnd));
      float cost;
      if (end_id == goal_id) {
        float reach_cost = reach[start_id];
        if (reach_cost >= 0) {
          cost = reach_cost;
        } else {
          cost = (float)1.0e10; // a very expensive cost to go
        }
      }
      else {
        point_t start_p = idToPoint(start_id);
        point_t end_p = idToPoint(end_id);
        float euclidean_distance = std::sqrt((end_p.x - start_p.x)*(end_p.x - start_p.x) +
          (end_p.y - start_p.y)*(end_p.y - start_p.y) + 
          (end_p.z - start_p.z)*(end_p.z - start_p.z));
        cost = euclidean_distance;
      }
      // std::cout << "start:" << start_id << " end: " << end_id << "cost = " << cost << std::endl;
      return cost;
    }

    virtual void AdjacentCost(void* state, MP_VECTOR<micropather::StateCost> *adjacent)
    {
      // Collect all 26 neighbor ids
      std::vector<int> neighbors;
      std::vector<float> dists;
      float dist_list[3] = {1.0, 1.414, 1.732};
      int state_id = *((int*)(&state));
      point_t p = idToPoint(state_id);
      // std::cout << "state position = " << "(" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
      for (int i=-1; i<2; i++) {
        for (int j=-1; j<2; j++) {
          for (int k=-1; k<2; k++) {
            int id = state_id + i + j*size[0] + k*size[0]*size[1];
            if (!(id == state_id)) {
              neighbors.push_back(id);
              float cost = dist_list[std::abs(i) + std::abs(j) + std::abs(k) - 1];
              dists.push_back(cost);
            }
          }
        }
      }

      // Get costs to neighbors from esdf
      int n_map_voxels = size[0]*size[1]*size[2];
      for (int i=0; i<neighbors.size(); i++) {
        int id = neighbors[i];
        if ((id >= 0) && (id < n_map_voxels)) {
          if ((reach[id] > 0.0) && (esdf[id] > 0.19)) {
            micropather::StateCost state_cost;
            state_cost.state = reinterpret_cast<void*>(id);
            state_cost.cost = dists[i]/esdf[id];
            adjacent->push_back(state_cost);
          }
        }
      }
      return;
    }

    virtual void PrintStateInfo(void* state)
    {
      int id = *((int*)(&state));
      point_t p = idToPoint(id);
      std::cout << "id: " << id << " - (" << p.x << ", " << p.y << ", " << p.z << ")";
      return;
    }
};