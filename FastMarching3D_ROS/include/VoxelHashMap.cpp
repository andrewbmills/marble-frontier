/******************************************************************************

Welcome to GDB Online.
GDB online is an online compiler and debugger tool for C, C++, Python, PHP, Ruby, 
C#, VB, Perl, Swift, Prolog, Javascript, Pascal, HTML, CSS, JS
Code, Compile, Run and Debug online from anywhere in world.

*******************************************************************************/
#include <stdio.h>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <math.h>
#include <assert.h>

struct LocalMap
{
    std::vector<float> local_map;
};

class VoxelHashMap
{
    public:
        VoxelHashMap(int _local_map_size[3], int _max_size[3], float _initial_value=-1.0, float error_value = NAN)
        {
            for (int i=0; i<3; i++) {
                num_buckets[i] = std::ceil(_max_size[i]/_local_map_size[i]);
                local_map_size[i] = _local_map_size[i];
                max_size[i] = _max_size[i];
            }
            int local_map_length = _local_map_size[0]*_local_map_size[1]*_local_map_size[2];
            std::vector<float> _empty_map(local_map_length, _initial_value);
            empty_map.local_map = _empty_map;
        }
        LocalMap empty_map;
        int max_size[3];
        int local_map_size[3];
        double voxel_size = 1;
        float error_value;
        double min[3] = {0.0, 0.0, 0.0};
        int num_buckets[3];
        std::unordered_map<int, LocalMap> data;
        std::pair<int, int> _GetKeyAndIndex(double p[3]);
        void Add(double x, double y, double z, float value);
        float& operator()(double x, double y, double z);
};

std::pair<int, int> VoxelHashMap::_GetKeyAndIndex(double p[3])
{
    int id[3];
    int index[3];
    for (int i=0; i<3; i++) {
        id[i] = std::round((p[i] - min[i])/voxel_size); // number of voxels from lower left of large map to point voxel
        assert((id[i] >= 0) && (id[i] <= max_size[i])); // Check if query is in-bounds
        index[i] = id[i] % local_map_size[i]; // number of voxels from lower left of local map container to point voxel
        id[i] = std::floor(id[i]/local_map_size[i]); // local map container 3d index for the point voxel
    }
    return std::make_pair(id[0] + id[1]*num_buckets[0] + id[2]*num_buckets[0]*num_buckets[1],
                           index[0] + index[1]*local_map_size[0] + index[2]*local_map_size[1]*local_map_size[0]);
}

float& VoxelHashMap::operator()(double x, double y, double z)
{
    // Get the key and point index
    double p[3] = {x, y, z};
    std::pair<int, int> key_and_id = _GetKeyAndIndex(p);
    int key = key_and_id.first;
    int idx = key_and_id.second;
    
    // std::cout << "Point [" << x << ", " << y << ", " << z << "] has key of " << key << std::endl;
    // std::cout << "Local map index is " << idx << std::endl;
    
    // Check if the local map container exists for this key
    if (data.count(key)) {
        return(data[key].local_map[idx]);
    }
    else {
        return(error_value);
    }
}

void VoxelHashMap::Add(double x, double y, double z, float value)
{
    // This function is for setting a value given a query
    
    // Get the key and point index
    double point[3] = {x, y, z};
    std::pair<int, int> key_and_id = _GetKeyAndIndex(point);
    int key = key_and_id.first;
    int id = key_and_id.second;
    // std::cout << "Point [" << x << ", " << y << ", " << z << "] has key of " << key << std::endl;
    // std::cout << "Local map index is " << id << std::endl;
    
     // Check if the local map container exists for this key
    if (!data.count(key)) {
        // std::cout << "Key does not currently exist, adding key" << std::endl;
        // Create the local map container if it doesn't exist yet
        LocalMap new_map = empty_map;
        data.insert({key, new_map});
    }
    data[key].local_map[id] = value;
    return;
}

int main()
{
    // Initialize map data
    int local_map_size[3] = {3,3,3};
    int max_size[3] = {9,9,9};
    VoxelHashMap map(local_map_size, max_size);
    
    // for (int i=0; i<3; i++) std::cout << map.max_size[i] << std::endl;
    // for (int i=0; i<3; i++) std::cout << map.local_map_size[i] << std::endl;
    
    // Set a few data points
    map.Add(1, 0, 2, 2.1);
    // float distance = map(1,0,2);
    // distance = 1.2;
    // map(2,3,1) = 1.9;
 
    // Output values by key
    std::cout << "Map value at [0,0,0] is " << map(0,0,0) << std::endl;
    std::cout << "Map value at [1,0,2] is " << map(1,0,2) << std::endl;
    std::cout << "Map value at [2,4,1] is " << map(2,4,1) << std::endl;
    return 0;
}