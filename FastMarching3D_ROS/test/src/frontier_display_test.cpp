#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "ros.h"
#include <chrono>
#include <frontier.h>
#include <mapGrid3D.h>
#include <hashGrid3D.h>
#include <hashGrid3D_conversions.h>
#include <octomap/octomap.h>
#include <octomap/AbstractOcTree.h>


// Frontier using raw octomap map
bool IsUnseen(double x, double y, double z, octomap::OcTree* tree)
{
    octomap::OcTreeNode* node = tree->search(x, y, z);
    return ((!(node)) || ((node->getOccupancy() >= 0.45) && (node->getOccupancy() <= 0.55)));
}

bool IsAdjacentToUnseen(double x, double y, double z, octomap::OcTree* tree)
{
    double voxel_size = tree->getResolution();
    return IsUnseen(x + voxel_size,y,z,tree)+IsUnseen(x - voxel_size,y,z,tree)+
           IsUnseen(x,y + voxel_size,z,tree)+IsUnseen(x,y - voxel_size,z,tree)+
           IsUnseen(x,y,z + voxel_size,tree)+IsUnseen(x,y,z - voxel_size,tree);
}

std::vector<Point> CalculateFrontierFromOctomap(octomap::OcTree* tree)
{
    std::vector<Point> frontiers;
    for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end=tree->end_leafs(); it!=end; ++it) {
        bool isFree = (it->getOccupancy() <= 0.5);
        if (isFree) {
            if (IsAdjacentToUnseen(it.getX(), it.getY(), it.getZ(), tree)) {
                Point frontier {(float)it.getX(), (float)it.getY(), (float)it.getZ()};
                frontiers.push_back(frontier);
            }
        }
    }
    return frontiers;
}

void ConvertPointVectorToPointCloud(std::vector<Point> v, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    for (int i=0; i<v.size(); i++) {
        pcl::PointXYZ p = {v[i].x, v[i].y, v[i].z};
        cloud->points.push_back(p);
    }
}

TEST_CASE("Publish ROS messages of raw frontiers", "frontier")
{
    SECTION("Publish frontier calculation on raw Octomap.") {
        for (int i=0; i<15; i++) {
            std::string filename = "/home/andrew/Desktop/octomaps/octomap_" + std::to_string(i+1) + ".ot";
            octomap::AbstractOcTree* readTreeAbstract = octomap::AbstractOcTree::read(filename);
            octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(readTreeAbstract);
            tree->expand();
            
            auto start = std::chrono::high_resolution_clock::now();
            std::vector<Point> frontiers = CalculateFrontierFromOctomap(tree);
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            INFO(frontiers.size() << " frontiers found in " << duration.count() << " ms for octomap of size " << tree->size());
            REQUIRE(frontiers.size() > 0);

            pcl::PointCloud<pcl::PointXYZ>::Ptr frontier_cloud (new pcl::PointCloud<pcl::PointXYZ>::Ptr);

            delete tree;
        }
    }
}


