#include <vector>
#include "math.h"
#include <iostream>
#include <pcl/point_types.h>

struct Point {
    float x, y, z;
};

struct Dimensions {
    int x, y, z;
};

template <typename T>
class MapGrid3D {
  public:
    Dimensions size;
    Point minBounds;
    Point maxBounds;
    std::vector<T> voxels;
    float voxelSize;
    MapGrid3D();
    MapGrid3D(float resolution, int sizeList[3]);
    MapGrid3D(float resolution, int sizeList[3], float minBoundList[3]);
    MapGrid3D(float resolution, Dimensions sizeIn, Point minBoundsIn);
    bool _CheckVoxelPositionInBounds(Point position);
    int _ConvertPositionToIndex(Point position);
    int _ConvertPositionToIndexNoCheck(Point position);
    Point _ConvertIndexToPosition(int idx);
    void _SetMaxBounds();
    bool SetVoxel(float x, float y, float z, T data, bool checkValid=true);
    void SetAll(T data);
    T Query(int idx);
    T Query(float x, float y, float z);
    void GetNeighbors26(float x, float y, float z, std::vector<Point> &neighbors, std::vector<T> &data, std::vector<int> &indices);
    void GetNeighbors6(float x, float y, float z, std::vector<Point> &neighbors, std::vector<T> &data, std::vector<int> &indices);
};

template <typename T>
MapGrid3D<T>::MapGrid3D()
{
  voxelSize = 1.0;
  size.x = 1;
  size.y = 1;
  size.z = 1;
  minBounds.x = 0.0;
  minBounds.y = 0.0;
  minBounds.z = 0.0;
  _SetMaxBounds();
  voxels.resize(size.x*size.y*size.z);
  return;
}

template <typename T>
MapGrid3D<T>::MapGrid3D(float resolution, int sizeList[3])
{
  voxelSize = resolution;
  size.x = sizeList[0];
  size.y = sizeList[1];
  size.z = sizeList[2];
  minBounds.x = 0.0;
  minBounds.y = 0.0;
  minBounds.z = 0.0;
  _SetMaxBounds();
  voxels.resize(size.x*size.y*size.z);
  return;
}

template <typename T>
MapGrid3D<T>::MapGrid3D(float resolution, Dimensions sizeIn, Point minBoundsIn)
{
  voxelSize = resolution;
  size = sizeIn;
  minBounds = minBoundsIn;
  _SetMaxBounds();
  voxels.resize(size.x*size.y*size.z);
  return;
}

template <typename T>
MapGrid3D<T>::MapGrid3D(float resolution, int sizeList[3], float minBoundList[3])
{
  voxelSize = resolution;
  size.x = sizeList[0];
  size.y = sizeList[1];
  size.z = sizeList[2];
  minBounds.x = minBoundList[0];
  minBounds.y = minBoundList[1];
  minBounds.z = minBoundList[2];
  _SetMaxBounds();
  voxels.resize(size.x*size.y*size.z);
  return;
}

int xyz_index3(const double point[3], double min[3], int size[3], double voxel_size)
{
  int ind[3];
  for (int i=0; i<3; i++) ind[i] = round((point[i]-min[i])/voxel_size);
  return (ind[0] + ind[1]*size[0] + ind[2]*size[0]*size[1]);
}

template <typename T>
int MapGrid3D<T>::_ConvertPositionToIndex(Point position)
{
  if (_CheckVoxelPositionInBounds(position)) {
    int id;
    id = std::round((position.z - minBounds.z)/voxelSize)*size.x*size.y;
    id += std::round((position.y - minBounds.y)/voxelSize)*size.x;
    id += std::round((position.x - minBounds.x)/voxelSize);
    return id;
  }
  else {
    return -1;
  }
}

template <typename T>
int MapGrid3D<T>::_ConvertPositionToIndexNoCheck(Point position)
{
  int id;
  id = std::roundf((position.z - minBounds.z)/voxelSize)*size.x*size.y;
  id += std::roundf((position.y - minBounds.y)/voxelSize)*size.x;
  id += std::roundf((position.x - minBounds.x)/voxelSize);
  return id;
}

template <typename T>
Point MapGrid3D<T>::_ConvertIndexToPosition(int idx)
{
  Point p{0.0, 0.0, 0.0};
  if ((idx >=0) && (idx < voxels.size())) {
    p.x = ((idx % (size.y*size.x)) % size.x)*voxelSize + minBounds.x ;
    p.y = ((idx % (size.y*size.x))/size.x)*voxelSize + minBounds.y;
    p.z = (idx/(size.y*size.x))*voxelSize + minBounds.z;
    return p;
  }
  else {
    return p;
  }
}

template <typename T>
bool MapGrid3D<T>::_CheckVoxelPositionInBounds(Point position)
{
  bool inBounds = (position.x >= minBounds.x) &&
  (position.y >= minBounds.y) &&
  (position.z >= minBounds.z) &&
  (position.x <= maxBounds.x) &&
  (position.y <= maxBounds.y) &&
  (position.z <= maxBounds.z);
  return inBounds;
}

template <typename T>
void MapGrid3D<T>::_SetMaxBounds()
{
  maxBounds.x = minBounds.x + voxelSize*(size.x - 1);
  maxBounds.y = minBounds.y + voxelSize*(size.y - 1);
  maxBounds.z = minBounds.z + voxelSize*(size.z - 1);
  return;
}

template <typename T>
bool MapGrid3D<T>::SetVoxel(float x, float y, float z, T data, bool checkValid)
{
  Point query{x, y, z};
  if (checkValid) {
    int idx = _ConvertPositionToIndex(query);
    if ((idx >= 0) && (idx < voxels.size())) {
      voxels[idx] = data;
      return true;
    }
    else {
      std::cout << "Voxel position out of bounds" << std::endl;
      return false;
    }
  } else {
    int idx = _ConvertPositionToIndexNoCheck(query);
    voxels[idx] = data;
    return true;
  }
}

template <typename T>
void MapGrid3D<T>::SetAll(T data)
{
  voxels.assign(size.x*size.y*size.z, data);
  return;
}

template <typename T>
T MapGrid3D<T>::Query(int idx)
{
  if ((idx >= 0) && (idx < voxels.size())) {
    return voxels[idx];
  }
  else {
    // std::cout << "Voxel id out of bounds" << std::endl;
    T garbage;
    return garbage;
  }
}

template <typename T>
T MapGrid3D<T>::Query(float x, float y, float z)
{
  Point query{x, y, z};
  int idx = _ConvertPositionToIndex(query);
  if ((idx >= 0) && (idx < voxels.size())) {
    return voxels[idx];
  }
  else {
    // std::cout << "Voxel query position out of bounds" << std::endl;
    T garbage;
    return garbage;
  }
}

template <typename T>
void MapGrid3D<T>::GetNeighbors26(float x, float y, float z, std::vector<Point> &neighbors, std::vector<T> &data, std::vector<int> &indices)
{
  // Get the 26 voxel neighbors of queried position
  neighbors.resize(26);
  data.resize(26);
  indices.resize(26);
  int neighborId = 0;
  float n_x, n_y, n_z; // neighbor position coordinates
  for (int i=-1; i<2; i++) {
    n_x = x + i*voxelSize;
    for (int j=-1; j<2; j++) {
      n_y = y + j*voxelSize;
      for (int k=-1; k<2; k++) {
        n_z = z + k*voxelSize;
        if (!((i==0) && (j==0) && (k==0))) {
          T neighbor = Query(n_x, n_y, n_z);
          Point query{x, y, z};
          neighbors[neighborId] = query;
          data[neighborId] = neighbor;
          indices[neighborId] = _ConvertPositionToIndex(query);
          neighborId++;
        }
      }
    }
  }
  return;
}

template <typename T>
void MapGrid3D<T>::GetNeighbors6(float x, float y, float z, std::vector<Point> &neighbors, std::vector<T> &data, std::vector<int> &indices)
{
  // Get the 6 voxel neighbors of queried position, 
  neighbors.clear();
  data.clear();
  indices.clear();
  float centerVoxelPosition[3] = {x, y, z};
  float neighborPosition[3] = {x, y, z};

  for (int j=0; j<3; j++) {
    for (int i=-1; i<2; i+=2) {
      neighborPosition[j] = centerVoxelPosition[j] + i*voxelSize;
      T neighbor = Query(neighborPosition[0], neighborPosition[1], neighborPosition[2]);
      Point query{neighborPosition[0], neighborPosition[1], neighborPosition[2]};
      neighbors.push_back(query);
      data.push_back(neighbor);
      indices.push_back(_ConvertPositionToIndex(query));
      neighborPosition[j] = centerVoxelPosition[j];
    }
  }
  return;
}