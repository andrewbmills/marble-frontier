#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <hashGrid3D_conversions.h>
#include <chrono>
#include <fstream>

TEST_CASE("Copy an octomap from file to a HashGrid3D object", "hashGrid3D_conversions")
{
    // std::string filename = "/home/andrew/Desktop/octomaps/octomap_15.ot";
    std::string filename = "/home/andrew/Desktop/octomaps/octomap_urban_practice_14.ot";
    octomap::AbstractOcTree* readTreeAbstract = octomap::AbstractOcTree::read(filename);
    octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(readTreeAbstract);

    HashGridExtents mapParams;
    double octomapMins[3]; double octomapMaxes[3];
    tree->getMetricMax(octomapMaxes[0], octomapMaxes[1], octomapMaxes[2]);
    tree->getMetricMin(octomapMins[0], octomapMins[1], octomapMins[2]);
    for (int i=0; i<3; i++) {
        mapParams.mins[i] = octomapMins[i] - 1.5*tree->getResolution();
        mapParams.maxes[i] = octomapMaxes[i] + 1.5*tree->getResolution();
    }
    mapParams.subMapSizes[0] = 20; mapParams.subMapSizes[1] = 20; mapParams.subMapSizes[2] = 20;
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

    std::vector<int> sizes = map.GetSizes();
    INFO("Hashgrid has sizes of [" << sizes[0] << ", " << sizes[1] << ", " << sizes[2] << "] submaps.");

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

    SECTION("Make sure all the voxel positions map forwards and backwards from the internal index.")
    {
        std::vector<double> mins = map.GetMins();
        std::vector<int> submap_sizes = map.GetSubMapSizes();
        std::vector<int> data = map.GetData();
        std::vector<int> keys = map.GetKeys();
        double half_voxel = map.GetVoxelSize()/2.0f;

        for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end=tree->end_leafs(); it!=end; ++it) {
            double x = it.getX(); double y = it.getY(); double z = it.getZ();
            int key_global = map._GetKeyId(x - mins[0], y - mins[1], z - mins[2]);
            int key = keys[map._GetKeyId(x - mins[0], y - mins[1], z - mins[2])];
            int id_submap = map._GetSubMapId(x - mins[0], y - mins[1], z - mins[2]);
            int id_point = map._GetSubMapId(x - mins[0], y - mins[1], z - mins[2]) + key*submap_sizes[0]*submap_sizes[1]*submap_sizes[2];
            std::vector<std::vector<double>> min_list = map.GetMinList();
            std::vector<double> point = map._GetPositionFromId(id_point);
            double dx = x - point[0]; double dy = y - point[1]; double dz = z - point[2];
            // INFO("Real point = [" << x << ", " << y << ", " << z << "]");
            // INFO("Key = " << key);
            // INFO("Submap number = " << key_global);
            // INFO("Id (global) = " << id_point);
            // INFO("Id (submap) = " << id_submap);
            // INFO("Processed Point = [" << point[0] << ", " << point[1] << ", " << point[2] << "]");
            // INFO("SubMap mins = [" << min_list[key][0] << ", " << min_list[key][1] << ", " << min_list[key][2] << "]");
            // INFO("Global mins = [" << mins[0] << ", " << mins[1] << ", " << mins[2] << "]");
            // INFO("SubMap sizes = [" << submap_sizes[0] << ", " << submap_sizes[1] << ", " << submap_sizes[2] << "]");
            REQUIRE( std::abs(dx) <= half_voxel );
            REQUIRE( std::abs(dy) <= half_voxel );
            REQUIRE( std::abs(dz) <= half_voxel );
        }
    }

    delete tree;
}