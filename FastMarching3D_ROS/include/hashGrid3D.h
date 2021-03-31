#ifndef HASH_GRID_H
#define HASH_GRID_H
#include <vector>
#include "math.h"
#include <MapGrid3D.h>

template <typename T>
class HashGrid3D {
  private:
    std::vector<MapGrid3D<T>> submaps;
    std::vector<int> keys;
    std::vector<float> min[3]; // Minimum coordinate in all dimensions
    std::vector<float> max[3]; // Maximum coordinate in all dimensions
    std::vector<int> size[3]; // Largest possible dimensions for this set of maps
    std::vector<int> submapSize[3];
    float voxelSize;
    T initialValue;
    int _GetKeyId(float x, float y, float z);
    int _SetKey(float x, float y, float z, int index);
    void _AddMap(float x, float y, float z);
    bool _CheckKeyInBounds(float x, float y, float z);
  public:
    HashGrid3D(float _min[3], float _max[3], int _subMapSize[3], float voxelSize);
    void Set(float x, float y, float z, T value);
    T Get(float x, float y, float z);
}

template <typename T>
void HashGrid3D<T>::HashGrid3D(float _min[3], float _max[3], int _subMapSize[3], T _initialValue, float voxelSize = 1.0)
{
  voxelSize = _voxelSize;
  initialValue = _initialValue;
  for (int i=0; i<3; i++) {
    min[i] = _min[i];
    max[i] = _max[i];
    subMapSize[i] = _subMapSize[i];
    size[i] = round((max[i] - min[i])/submapSize[i]) + 1;
  }
  keys.resize[size[0]*size[1]*size[2]];
  std::fill(keys.begin(), keys.end(), -1);
  return;
}


template <typename T>
bool HashGrid3D<T>::_CheckKeyInBounds(float x, float y, float z)
{
  return (x >= min[0])*(x <= max[0])*(y >= min[1])*(y <= max[1])*(z >= min[2])*(z <= max[2]);
}

template <typename T>
void HashGrid3D<T>::Set(float x, float y, float z, T value)
{
  int key = keys[_GetKeyId(x, y, z)];

  if (key == -1) {
    // This submap does not exist and must be created.
    void _AddMap(x, y, z);
    key = keys[_GetKeyId(x, y, z)];
  }

  // Set the corresponding value in the correct submap
  submaps[key].SetVoxel(x, y, z, value);
  return;
}

template <typename T>
T HashGrid3D<T>::Get(float x, float y, float z)
{
  int key = keys[_GetKey(x, y, z)];

  if (key == -1) {
    // This submap does not exist return the default value.
    return initialValue;
  } else {
    // Found key, query the value.
    return submaps[key].Query(x, y, z);
  }
}

template <typename T>
int HashGrid3D<T>::_GetKeyId(float x, float y, float z)
{
  int dx = std::floor((x - min[0])/(submapSize[0]*voxelSize));
  int dy = std::floor((y - min[1])/(submapSize[1]*voxelSize));
  int dz = std::floor((z - min[2])/(submapSize[2]*voxelSize));
  return dx + dy*size[0] + dz*size[1]*size[0];
}

template <typename T>
void HashGrid3D<T>::_AddMap(float x, float y, float z)
{
  MapGrid3D<T> newSubmap;
  float query[3] = {x, y, z};
  float minSubmap[3];
  float maxSubmap[3];
  for (int i=0; i<3; i++) {
    int delta = std::floor((query[i] - min[i])/(submapSize[i]*voxelSize));
    minSubmap[i] = min[i] + submapSize[i]*voxelSize*delta;
    maxSubmap[i] = min[i] + submapSize[i]*voxelSize*(delta + 1);
  }
  newSubMap.Reset(voxelSize, minSubmap, maxSubmap, initialValue);
  submaps.push_back(newSubMap);
  keys[_GetKeyId(x,y,z)] = submaps.size()-1;
  return;
}


#endif