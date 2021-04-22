#ifndef HASH_GRID_H
#define HASH_GRID_H
#include <vector>
#include "math.h"

template <typename T>
struct SubMap {
  std::vector<T> voxels;
  std::vector<float> mins = std::vector<float>(3);
};

template <typename T>
class HashGrid3D {
  private:
    std::vector<SubMap<T>> subMaps;
    std::vector<int> keys;
    std::vector<float> mins = std::vector<float>(3); // Minimum coordinate in all dimensions
    std::vector<float> maxes = std::vector<float>(3); // Maximum coordinate in all dimensions
    std::vector<int> sizes = std::vector<int>(3); // Largest possible dimensions for this set of maps
    std::vector<int> subMapSizes = std::vector<int>(3);
    float voxelSize;
    T initialValue;
    bool checkBounds = false;
    int _SetKey(float x, float y, float z, int index);
    void _AddMap(float x, float y, float z);
    std::vector<float> _CalculateSubMapMins(float x, float y, float z);
    int unmappedKey = -1;
  public:
    HashGrid3D(float _min[3], float _max[3], int _subMapSize[3], T _initialValue, float _voxelSize);
    void Set(float x, float y, float z, T value);
    T Get(float x, float y, float z);
    int _GetKeyId(float x, float y, float z);
    int _GetSubMapId(float x, float y, float z);
    bool CheckKeyInBounds(float x, float y, float z);
    void EnableBoundsChecking() { checkBounds = true; }
    void DisableBoundsChecking() { checkBounds = false; }
    std::vector<int> GetSizes() { return sizes; }
    std::vector<int> GetKeys() { return keys; }
    std::vector<SubMap<T>> GetSubMaps() { return subMaps; }
    std::vector<float> GetMins() { return mins; }
    void GetDifferences() { return; }
};

template <typename T>
HashGrid3D<T>::HashGrid3D(float _mins[3], float _maxes[3], int _subMapSizes[3], T _initialValue, float _voxelSize)
{
  voxelSize = _voxelSize;
  initialValue = _initialValue;
  for (int i=0; i<3; i++) {
    mins[i] = _mins[i];
    maxes[i] = _maxes[i];
    subMapSizes[i] = _subMapSizes[i];
    sizes[i] = ceil( (ceil( (maxes[i] - mins[i])/voxelSize) + 1 )/subMapSizes[i] );
  }
  keys.resize(sizes[0]*sizes[1]*sizes[2]);
  std::fill(keys.begin(), keys.end(), unmappedKey);
  return;
}

template <typename T>
int HashGrid3D<T>::_GetSubMapId(float x, float y, float z)
{
  int dx = (int)round(x/voxelSize) % (subMapSizes[0]);
  int dy = (int)round(y/voxelSize) % (subMapSizes[1]);
  int dz = (int)round(z/voxelSize) % (subMapSizes[2]);
  return dx + dy*subMapSizes[0] + dz*subMapSizes[0]*subMapSizes[1];
}

template <typename T>
bool HashGrid3D<T>::CheckKeyInBounds(float x, float y, float z)
{
  return (x >= mins[0])*(x <= maxes[0])*(y >= mins[1])*(y <= maxes[1])*(z >= mins[2])*(z <= maxes[2]);
}

template <typename T>
void HashGrid3D<T>::Set(float x, float y, float z, T value)
{
  // Convert x,y,z to map-relative coordinates
  x = x - mins[0]; y = y - mins[1]; z = z - mins[2];
  int key = keys[_GetKeyId(x, y, z)];
  if (key == unmappedKey) {
    _AddMap(x, y, z);
    key = keys[_GetKeyId(x, y, z)];
  }
  subMaps[key].voxels[_GetSubMapId(x,y,z)] = value;
  return;
}

template <typename T>
T HashGrid3D<T>::Get(float x, float y, float z)
{
  // Convert x,y,z to map-relative coordinates
  x = x - mins[0]; y = y - mins[1]; z = z - mins[2];
  int key = keys[_GetKeyId(x, y, z)];

  if (key == unmappedKey) {
    return initialValue;
  } else {
    return subMaps[key].voxels[_GetSubMapId(x,y,z)];
  }
}

template <typename T>
int HashGrid3D<T>::_GetKeyId(float x, float y, float z)
{
  int dx = std::floor(x/(subMapSizes[0]*voxelSize));
  int dy = std::floor(y/(subMapSizes[1]*voxelSize));
  int dz = std::floor(z/(subMapSizes[2]*voxelSize));
  return dx + dy*sizes[0] + dz*sizes[1]*sizes[0];
}

template <typename T>
std::vector<float> HashGrid3D<T>::_CalculateSubMapMins(float x, float y, float z)
{
  std::vector<float> mins;
  mins.push_back(subMapSizes[0]*std::floor(x/(subMapSizes[0]*voxelSize)));
  mins.push_back(subMapSizes[1]*std::floor(x/(subMapSizes[1]*voxelSize)));
  mins.push_back(subMapSizes[2]*std::floor(x/(subMapSizes[2]*voxelSize)));
  return mins;
}

template <typename T>
void HashGrid3D<T>::_AddMap(float x, float y, float z)
{
  SubMap<T> newSubMap;
  newSubMap.voxels.resize(subMapSizes[0]*subMapSizes[1]*subMapSizes[2]);
  std::fill(newSubMap.voxels.begin(), newSubMap.voxels.end(), initialValue);
  newSubMap.mins = _CalculateSubMapMins(x,y,z);
  subMaps.push_back(newSubMap);
  keys[_GetKeyId(x,y,z)] = subMaps.size()-1;
  return;
}

#endif