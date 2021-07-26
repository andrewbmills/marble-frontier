#ifndef HASH_GRID_CONVERSIONS_H
#define HASH_GRID_CONVERSIONS_H
#include <math.h>
#include <hashGrid3D.h>
#include <octomap/octomap.h>
#include <octomap/AbstractOcTree.h>

struct HashGridExtents
{
    double mins[3];
    double maxes[3];
    int subMapSizes[3];
    double voxelSize;
};

HashGridExtents AlignExtentsWithOctomap(octomap::OcTree* tree, HashGridExtents params)
{
    octomap::OcTree::leaf_iterator it = tree->begin_leafs();
    params.mins[0] = params.mins[0] - std::fmod(std::abs(it.getX()-params.mins[0]), tree->getResolution());
    params.mins[1] = params.mins[1] - std::fmod(std::abs(it.getX()-params.mins[1]), tree->getResolution());
    params.mins[2] = params.mins[2] - std::fmod(std::abs(it.getX()-params.mins[2]), tree->getResolution());
    return params;
}

HashGrid3D<int> ConvertOcTreeBinaryToHashGrid3D(octomap::OcTree* tree, HashGridExtents hashGridParams)
{
    hashGridParams = AlignExtentsWithOctomap(tree, hashGridParams);
    HashGrid3D<int> map(hashGridParams.mins, hashGridParams.maxes, hashGridParams.subMapSizes, -1, tree->getResolution());
    for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end=tree->end_leafs(); it!=end; ++it) {
        map.Set(it.getX(), it.getY(), it.getZ(), 1*(it->getOccupancy() >= 0.5) + 0 );
    }
    return map;
}

#endif