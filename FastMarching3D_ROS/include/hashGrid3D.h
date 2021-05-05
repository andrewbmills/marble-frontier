#ifndef HASH_GRID_H
#define HASH_GRID_H
#include <vector>
#include "math.h"

template <typename T>
class HashGrid3D {
  private:
    std::vector<T> data;
    std::vector<T> blankSubMap;
    std::vector<std::vector<double>> mapMinList;
    std::vector<int> keys;
    std::vector<double> mins = std::vector<double>(3); // Minimum coordinate in all dimensions
    std::vector<double> maxes = std::vector<double>(3); // Maximum coordinate in all dimensions
    std::vector<int> sizes = std::vector<int>(3); // Largest possible dimensions for this set of maps
    std::vector<int> subMapSizes = std::vector<int>(3);
    int subMapOffset;
    std::vector<std::vector<double>> idToPositionLookup;
    std::vector<bool> subMapEdgeTable;
    double voxelSize;
    T initialValue;
    bool checkBounds = false;
    int _SetKey(double x, double y, double z, int index);
    void _AddMap(double x, double y, double z);
    void _InitializeIdToPositionLookup();
    void _InitializeBlankSubMap();
    void _InitializeSubMapEdgeLookup();
    int unmappedKey = -1;
  public:
    HashGrid3D(double _min[3], double _max[3], int _subMapSize[3], T _initialValue, double _voxelSize);
    void Set(double x, double y, double z, T value);
    T Get(double x, double y, double z);
    int _GetKeyId(double x, double y, double z);
    int _GetSubMapId(double x, double y, double z);
    std::vector<double> _GetPositionFromId(int id);
    bool CheckKeyInBounds(double x, double y, double z);
    void EnableBoundsChecking() { checkBounds = true; }
    void DisableBoundsChecking() { checkBounds = false; }
    std::vector<int> GetSizes() { return sizes; }
    std::vector<int> GetSubMapSizes() { return subMapSizes; }
    std::vector<int> GetKeys() { return keys; }
    std::vector<T> GetData() { return data; }
    std::vector<std::vector<double>> GetMinList() { return mapMinList; }
    std::vector<double> GetMins() { return mins; }
    double GetVoxelSize() { return voxelSize; }
    void GetDifferences() { return; }
    std::vector<double> _CalculateSubMapMins(double x, double y, double z);
    bool IsSubMapEdge(int id);
};

template <typename T>
HashGrid3D<T>::HashGrid3D(double _mins[3], double _maxes[3], int _subMapSizes[3], T _initialValue, double _voxelSize)
{
  voxelSize = _voxelSize;
  initialValue = _initialValue;
  for (int i=0; i<3; i++) {
    mins[i] = _mins[i];
    maxes[i] = _maxes[i];
    subMapSizes[i] = _subMapSizes[i];
    sizes[i] = ceil( (ceil( (maxes[i] - mins[i])/voxelSize) + 1 )/subMapSizes[i] );
  }
  _InitializeIdToPositionLookup();
  _InitializeBlankSubMap();
  _InitializeSubMapEdgeLookup();
  keys.resize(sizes[0]*sizes[1]*sizes[2]);
  std::fill(keys.begin(), keys.end(), unmappedKey);
  return;
}

template <typename T>
void HashGrid3D<T>::_InitializeIdToPositionLookup()
{
  std::vector<double> position = std::vector<double>(3);
  for (int i=0; i<3; i++) position[i] = 0.0;
  for (int k=0; k<subMapSizes[2]; k++) {
    for (int j=0; j<subMapSizes[1]; j++) {
      for (int i=0; i<subMapSizes[0]; i++) {
        idToPositionLookup.push_back(position);
        position[0] += voxelSize;
      }
      position[0] = 0.0;
      position[1] += voxelSize;
    }
    position[0] = 0.0;
    position[1] = 0.0;
    position[2] += voxelSize;
  }
}

template <typename T>
void HashGrid3D<T>::_InitializeBlankSubMap()
{
  subMapOffset = subMapSizes[0]*subMapSizes[1]*subMapSizes[2];
  blankSubMap = std::vector<T>(subMapOffset);
  std::fill(blankSubMap.begin(), blankSubMap.end(), initialValue);
  return;
}

template <typename T>
void HashGrid3D<T>::_InitializeSubMapEdgeLookup()
{
  subMapEdgeTable = std::vector<bool>(subMapOffset);
  for (int k=0; k<subMapSizes[2]; k++) {
    for (int j=0; j<subMapSizes[1]; j++) {
      for (int i=0; i<subMapSizes[0]; i++) {
        int id = i + j*subMapSizes[0] + k*subMapSizes[0]*subMapSizes[1];
        if ((i == 0) || (i == subMapSizes[0]) || (j == 0) || (j == subMapSizes[1]) ||(k == 0) || (k == subMapSizes[2])) subMapEdgeTable[id] = true;
        else subMapEdgeTable[id] = false;
      }
    }
  }
  return;
}

template <typename T>
bool HashGrid3D<T>::IsSubMapEdge(int id)
{
  id = (id % subMapOffset);
  return subMapEdgeTable[id];
}

template <typename T>
std::vector<double> HashGrid3D<T>::_GetPositionFromId(int id)
{
  std::vector<double> position = std::vector<double>(3);
  int idSubMap = std::floor(id / subMapOffset);
  int idLocal = (id % subMapOffset);
  for (int i=0; i<3; i++) position[i] = mapMinList[idSubMap][i] + idToPositionLookup[idLocal][i];
  return position;
}

template <typename T>
int HashGrid3D<T>::_GetSubMapId(double x, double y, double z)
{
  int dx = (int)round(x/voxelSize) % (subMapSizes[0]);
  int dy = (int)round(y/voxelSize) % (subMapSizes[1]);
  int dz = (int)round(z/voxelSize) % (subMapSizes[2]);
  return dx + dy*subMapSizes[0] + dz*subMapSizes[0]*subMapSizes[1];
}

template <typename T>
bool HashGrid3D<T>::CheckKeyInBounds(double x, double y, double z)
{
  return (x >= mins[0])*(x <= maxes[0])*(y >= mins[1])*(y <= maxes[1])*(z >= mins[2])*(z <= maxes[2]);
}

template <typename T>
void HashGrid3D<T>::Set(double x, double y, double z, T value)
{
  // Convert x,y,z to map-relative coordinates
  x = x - mins[0]; y = y - mins[1]; z = z - mins[2];
  int key = keys[_GetKeyId(x, y, z)];
  if (key == unmappedKey) {
    if (CheckKeyInBounds(x + mins[0], y + mins[1], z + mins[2])) {
      _AddMap(x, y, z);
      key = keys[_GetKeyId(x, y, z)];
    } else {
      return;
    }
  }
  data[key*subMapOffset + _GetSubMapId(x,y,z)] = value;
  return;
}

template <typename T>
T HashGrid3D<T>::Get(double x, double y, double z)
{
  // Convert x,y,z to map-relative coordinates
  x = x - mins[0]; y = y - mins[1]; z = z - mins[2];
  int key = keys[_GetKeyId(x, y, z)];

  if (key == unmappedKey) {
    return initialValue;
  } else {
    return data[key*subMapOffset + _GetSubMapId(x,y,z)];
  }
}

template <typename T>
int HashGrid3D<T>::_GetKeyId(double x, double y, double z)
{
  int dx = std::floor(x/(subMapSizes[0]*voxelSize));
  int dy = std::floor(y/(subMapSizes[1]*voxelSize));
  int dz = std::floor(z/(subMapSizes[2]*voxelSize));
  return dx + dy*sizes[0] + dz*sizes[1]*sizes[0];
}

template <typename T>
std::vector<double> HashGrid3D<T>::_CalculateSubMapMins(double x, double y, double z)
{
  std::vector<double> mins_submap;
  mins_submap.push_back(mins[0] + voxelSize*subMapSizes[0]*std::floor(x/(subMapSizes[0]*voxelSize)));
  mins_submap.push_back(mins[1] + voxelSize*subMapSizes[1]*std::floor(y/(subMapSizes[1]*voxelSize)));
  mins_submap.push_back(mins[2] + voxelSize*subMapSizes[2]*std::floor(z/(subMapSizes[2]*voxelSize)));
  return mins_submap;
}

template <typename T>
void HashGrid3D<T>::_AddMap(double x, double y, double z)
{
  mapMinList.push_back(_CalculateSubMapMins(x,y,z));
  data.resize(data.size() + subMapOffset);
  std::fill(data.end() - subMapOffset, data.end(), initialValue);
  keys[_GetKeyId(x,y,z)] = mapMinList.size()-1;
  return;
}

#endif