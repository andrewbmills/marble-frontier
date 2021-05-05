#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <hashGrid3D_conversions.h>
#include <chrono>
#include <fstream>

TEST_CASE("Copy an octomap from file to a HashGrid3D object", "hashGrid3D_conversions")
{
    std::string filename = "/home/andrew/Desktop/octomaps/octomap_1.ot";
    octomap::AbstractOcTree* readTreeAbstract = octomap::AbstractOcTree::read(filename);
    octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(readTreeAbstract);

    HashGridExtents mapParams;
    double octomapMins[3]; double octomapMaxes[3];
    tree->getMetricMax(octomapMaxes[0], octomapMaxes[1], octomapMaxes[2]);
    tree->getMetricMin(octomapMins[0], octomapMins[1], octomapMins[2]);
    mapParams.mins[0] = -20.0; mapParams.mins[1] = -100.0; mapParams.mins[2] = -5.0;
    mapParams.maxes[0] = 200.0; mapParams.maxes[1] = 100.0; mapParams.maxes[2] =  10.0;
    mapParams.subMapSizes[0] = 50; mapParams.subMapSizes[1] = 50; mapParams.subMapSizes[2] = 50;
    mapParams.voxelSize = 0.1;
    tree->expand();

    SECTION("Make sure Octomap has loaded correctly")
    {
        INFO("Octomap mins = [" << octomapMins[0] << ", " << octomapMins[1] << ", " << octomapMins[2] << "]");
        INFO("Octomap maxes = [" << octomapMaxes[0] << ", " << octomapMaxes[1] << ", " << octomapMaxes[2] << "]");
        REQUIRE(tree->size() >= 0);
    }

    HashGrid3D<int> map = ConvertOcTreeBinaryToHashGrid3D(tree, mapParams);

    SECTION("Initialized HashGridMap3D member value checks")
    {
        INFO("Octree is of resolution " << tree->getResolution());
        std::vector<std::vector<double>> minList = map.GetMinList();
        REQUIRE(minList.size() > 0);
    }
    SECTION("Check to make sure every voxel is the same")
    {
        int number_same = 0;
        int voxel_count = 0;
        for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end=tree->end_leafs(); it!=end; ++it) {
            int value = map.Get(it.getX(), it.getY(), it.getZ());
            if (value == (1*(it->getOccupancy() >= 0.5) + 0)) number_same++;
            voxel_count++;
        }
        REQUIRE(voxel_count == number_same);
    }
    delete tree;
}