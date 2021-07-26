#ifndef FRONTIER_HASHGRID_H
#define FRONTIER_HASHGRID_H
#include <omp.h>
#include <frontier.h>
#include <hashGrid3D.h>
#include <hashGrid3D_conversions.h>

bool IsUnseen(double x, double y, double z, HashGrid3D<int>* map)
{
    return (map->Get(x,y,z) == -1);
}

bool IsAdjacentToUnseen(double x, double y, double z, HashGrid3D<int>* map, double voxel_size)
{
    return IsUnseen(x + voxel_size, y, z, map)+IsUnseen(x - voxel_size, y, z, map)+
           IsUnseen(x, y + voxel_size, z, map)+IsUnseen(x, y - voxel_size, z, map)+
           IsUnseen(x, y, z + voxel_size, map)+IsUnseen(x, y, z - voxel_size, map);
}

bool IsAdjacentToUnseenFast(int id, std::vector<int> &data, std::vector<int> &neighbor_deltas)
{
    for (int i=0; i<neighbor_deltas.size(); i++) {
        if (data[id + neighbor_deltas[i]] == -1) return true;
    }
    return false;
}

std::vector<int> GetNeighborDeltas(std::vector<int> sizes)
{
    std::vector<int> neighbor_deltas;
    neighbor_deltas.push_back(1); neighbor_deltas.push_back(-1);
    neighbor_deltas.push_back(sizes[0]); neighbor_deltas.push_back(-sizes[0]);
    neighbor_deltas.push_back(sizes[1]*sizes[0]); neighbor_deltas.push_back(-sizes[1]*sizes[0]);
    return neighbor_deltas;
}

std::vector<bool> GetSubMapEdgeTable(std::vector<int> sizes)
{
  std::vector<bool> subMapEdgeTable = std::vector<bool>(sizes[0]*sizes[1]*sizes[2]);
  for (int k=0; k<sizes[2]; k++) {
    for (int j=0; j<sizes[1]; j++) {
      for (int i=0; i<sizes[0]; i++) {
        int id = i + j*sizes[0] + k*sizes[0]*sizes[1];
        if ((i == 0) || (i == sizes[0]) || (j == 0) || (j == sizes[1]) ||(k == 0) || (k == sizes[2])) subMapEdgeTable[id] = true;
        else subMapEdgeTable[id] = false;
      }
    }
  }
  return subMapEdgeTable;
}

std::vector<Point> CalculateFrontierFromHashGrid3D(HashGrid3D<int>* map)
{
    double voxel_size = map->GetVoxelSize();
    std::vector<Point> frontiers;
    std::vector<int> data = map->GetData();
    for (int i=0; i<data.size(); i++) {
        bool isFree = (data[i] == 0);
        if (isFree) {
            std::vector<double> position = map->_GetPositionFromId(i);
            bool isAdjacentToUnseen = IsAdjacentToUnseen(position[0], position[1], position[2], map, voxel_size);
            if (isAdjacentToUnseen) {
                Point frontier = {(float)position[0], (float)position[1], (float)position[2]};
                frontiers.push_back(frontier);
            } 
        }
    }
    return frontiers;
}

std::vector<Point> CalculateFrontierFromHashGrid3D_OMP(HashGrid3D<int>* map)
{
    double voxel_size = map->GetVoxelSize();
    std::vector<int> data = map->GetData();
    std::vector<int> sizes = map->GetSubMapSizes();
    int submap_size = sizes[0]*sizes[1]*sizes[2];
    std::vector<bool> isFrontierTable = std::vector<bool>(data.size());
    std::fill(isFrontierTable.begin(), isFrontierTable.end(), false);
    
    #pragma omp parallel num_threads(11)
    {
        #pragma omp for
        for (int i=0; i<data.size(); i++) {
            bool isFree = (data[i] == 0);
            if (isFree) {
                std::vector<double> position = map->_GetPositionFromId(i);
                bool isAdjacentToUnseen = IsAdjacentToUnseen(position[0], position[1], position[2], map, voxel_size);
                if (isAdjacentToUnseen) {
                    isFrontierTable[i] = true;
                }
            }
        }
    }

    std::vector<Point> frontiers;
    for (int i=0; i<data.size(); i++) {
        if (isFrontierTable[i]) {
            std::vector<double> position = map->_GetPositionFromId(i);
            Point frontier = {(float)position[0], (float)position[1], (float)position[2]};
            frontiers.push_back(frontier);
        }
    }
    return frontiers;
}

#endif