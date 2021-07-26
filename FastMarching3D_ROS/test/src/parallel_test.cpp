#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <chrono>
#include <cmath>
#include <frontier.h>
#include <frontier_octomap.h>
#include <frontier_hashgrid.h>

TEST_CASE("OcTree frontier finding in parallel", "parallel")
{
    std::string filename_prefix = "/home/andrew/Desktop/octomaps/octomap_urban_practice_";
    std::string filename = filename_prefix + std::to_string(15) + ".ot";
    octomap::AbstractOcTree* readTreeAbstract = octomap::AbstractOcTree::read(filename);
    octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(readTreeAbstract);
    tree->expand();

    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Point> frontiers_serial = CalculateFrontierFromOctomap(tree);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_serial = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    start = std::chrono::high_resolution_clock::now();
    std::vector<Point> frontiers_parallel = CalculateFrontierFromOctomap_OMP(tree);
    end = std::chrono::high_resolution_clock::now();
    auto duration_parallel = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    INFO(frontiers_serial.size() << " frontiers found in " << duration_serial.count() << " ms for octomap of size " << tree->size() << " using OcTree serial");
    INFO(frontiers_parallel.size() << " frontiers found in " << duration_parallel.count() << " ms for octomap of size " << tree->size() << " using OcTree parallel.");
    REQUIRE(frontiers_serial.size() > 0);
    delete tree;
}

TEST_CASE("HashGrid frontier finding in parallel", "parallel")
{
    HashGridExtents mapParams;
    mapParams.subMapSizes[0] = 20; mapParams.subMapSizes[1] = 20; mapParams.subMapSizes[2] = 20;
    mapParams.voxelSize = 0.1;

    std::string filename_prefix = "/home/andrew/Desktop/octomaps/octomap_urban_practice_";
    std::string filename = filename_prefix + std::to_string(15) + ".ot";
    octomap::AbstractOcTree* readTreeAbstract = octomap::AbstractOcTree::read(filename);
    octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(readTreeAbstract);
    tree->expand();

    double octomapMins[3]; double octomapMaxes[3];
    tree->getMetricMax(octomapMaxes[0], octomapMaxes[1], octomapMaxes[2]);
    tree->getMetricMin(octomapMins[0], octomapMins[1], octomapMins[2]);
    for (int i=0; i<3; i++) {
        mapParams.mins[i] = octomapMins[i] - 1.5*tree->getResolution();
        mapParams.maxes[i] = octomapMaxes[i] + 1.5*tree->getResolution();
    }

    auto start = std::chrono::high_resolution_clock::now();
    HashGrid3D<int> map = ConvertOcTreeBinaryToHashGrid3D(tree, mapParams);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_convert = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    start = std::chrono::high_resolution_clock::now();
    std::vector<Point> frontiers_serial = CalculateFrontierFromHashGrid3D(&map);
    end = std::chrono::high_resolution_clock::now();
    auto duration_serial = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    start = std::chrono::high_resolution_clock::now();
    std::vector<Point> frontiers_parallel = CalculateFrontierFromHashGrid3D_OMP(&map);
    end = std::chrono::high_resolution_clock::now();
    auto duration_parallel = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    INFO("Map conversion in " << duration_convert.count() << " ms");
    INFO(frontiers_serial.size() << " frontiers found in " << duration_serial.count() << " ms for octomap of size " << tree->size() << " using HashGrid3D serial");
    INFO(frontiers_parallel.size() << " frontiers found in " << duration_parallel.count() << " ms for octomap of size " << tree->size() << " using HashGrid3D parallel");
    REQUIRE(frontiers_serial.size() > 0);
    delete tree;
}

TEST_CASE("MapGrid frontier finding in parallel", "parallel")
{
    std::string filename_prefix = "/home/andrew/Desktop/octomaps/octomap_urban_practice_";
    std::string filename = filename_prefix + std::to_string(15) + ".ot";
    octomap::AbstractOcTree* readTreeAbstract = octomap::AbstractOcTree::read(filename);
    octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(readTreeAbstract);
    tree->expand();

    auto start = std::chrono::high_resolution_clock::now();
    MapGrid3D<std::pair<bool,bool>> seenOcc;
    ConvertOctomapToSeenOccGrid(tree, &seenOcc);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_convert = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    start = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr frontiers_serial(new pcl::PointCloud<pcl::PointXYZ>);
    CalculateFrontierRawPCL(&seenOcc, frontiers_serial);
    end = std::chrono::high_resolution_clock::now();
    auto duration_serial = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    start = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr frontiers_parallel(new pcl::PointCloud<pcl::PointXYZ>);
    CalculateFrontierRawPCL_OMP(&seenOcc, frontiers_parallel);
    end = std::chrono::high_resolution_clock::now();
    auto duration_parallel = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    INFO("Map conversion in " << duration_convert.count() << " ms");
    INFO(frontiers_serial->points.size() << " frontiers found in " << duration_serial.count() << " ms for octomap of size " << tree->size() << " using MapGrid3D serial");
    INFO(frontiers_parallel->points.size() << " frontiers found in " << duration_parallel.count() << " ms for octomap of size " << tree->size() << " using MapGrid3D parallel");
    REQUIRE(frontiers_serial->points.size() > 0);
    delete tree;
}

TEST_CASE("Performance benchmarks for raw frontier calculations done in parallel", "frontier")
{
    HashGridExtents mapParams;
    mapParams.subMapSizes[0] = 20; mapParams.subMapSizes[1] = 20; mapParams.subMapSizes[2] = 20;
    mapParams.voxelSize = 0.1;

    int map_start = 1;
    int map_end = 15;

    std::ofstream file;
    file.open("/home/andrew/Desktop/octomaps/frontier_parallel_urban_benchmarks.csv");
    // file.open("/home/andrew/Desktop/octomaps/frontier_parallel_benchmarks.csv");
    std::string filename_prefix = "/home/andrew/Desktop/octomaps/octomap_urban_practice_";
    // std::string filename_prefix = "/home/andrew/Desktop/octomaps/octomap_";

    for (int i=map_start; i<(map_end+1); i++) {
        std::string filename = filename_prefix + std::to_string(i) + ".ot";
        octomap::AbstractOcTree* readTreeAbstract = octomap::AbstractOcTree::read(filename);
        octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(readTreeAbstract);
        tree->expand();
        
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<Point> frontiers = CalculateFrontierFromOctomap_OMP(tree);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        INFO(frontiers.size() << " frontiers found in " << duration.count() << " ms for octomap of size " << tree->size() << " using OcTree");
        double octree_size = ((double)tree->memoryUsage())/1000.0;
        INFO("OcTree is of size " << octree_size << "kB");
        REQUIRE(frontiers.size() > 0);
        file << i << "," << frontiers.size() << "," << duration.count() << "," << tree->size() << "," << octree_size <<std::endl;
        delete tree;
    }  

    for (int i=map_start; i<(map_end+1); i++) {
        std::string filename = filename_prefix + std::to_string(i) + ".ot";
        octomap::AbstractOcTree* readTreeAbstract = octomap::AbstractOcTree::read(filename);
        octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(readTreeAbstract);
        tree->expand();

        auto start = std::chrono::high_resolution_clock::now();
        MapGrid3D<std::pair<bool,bool>> seenOcc;
        ConvertOctomapToSeenOccGrid(tree, &seenOcc);
        pcl::PointCloud<pcl::PointXYZ>::Ptr frontiers(new pcl::PointCloud<pcl::PointXYZ>);
        CalculateFrontierRawPCL_OMP(&seenOcc, frontiers);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        INFO(frontiers->points.size() << " frontiers found in " << duration.count() << " ms for octomap of size " << tree->size() << " using MapGrid3D");
        double mapGrid_size = seenOcc.size.x*seenOcc.size.y*seenOcc.size.z*2.0/1000.0;
        INFO("MapGrid3D is of size " << mapGrid_size << "kB");
        REQUIRE(frontiers->points.size() > 0);
        file << i << "," << frontiers->points.size() << "," << duration.count() << "," << tree->size() << "," << mapGrid_size << std::endl;
        delete tree;
    }

    for (int i=map_start; i<(map_end+1); i++) {
        std::string filename = filename_prefix + std::to_string(i) + ".ot";
        octomap::AbstractOcTree* readTreeAbstract = octomap::AbstractOcTree::read(filename);
        octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(readTreeAbstract);
        tree->expand();

        double octomapMins[3]; double octomapMaxes[3];
        tree->getMetricMax(octomapMaxes[0], octomapMaxes[1], octomapMaxes[2]);
        tree->getMetricMin(octomapMins[0], octomapMins[1], octomapMins[2]);
        for (int i=0; i<3; i++) {
            mapParams.mins[i] = octomapMins[i] - 1.5*tree->getResolution();
            mapParams.maxes[i] = octomapMaxes[i] + 1.5*tree->getResolution();
        }

        auto start = std::chrono::high_resolution_clock::now();
        HashGrid3D<int> map = ConvertOcTreeBinaryToHashGrid3D(tree, mapParams);
        std::vector<Point> frontiers = CalculateFrontierFromHashGrid3D_OMP(&map);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        std::vector<int> map_data = map.GetData();
        double hashGrid_size = map_data.size()*4.0 / 1000.0;
        INFO(frontiers.size() << " frontiers found in " << duration.count() << " ms for octomap of size " << tree->size() << " using HashGrid3D");
        INFO("HashGrid3D is of size " << hashGrid_size << "kB");
        REQUIRE(frontiers.size() > 0);
        file << i << "," << frontiers.size() << "," << duration.count() << "," << tree->size() << "," << hashGrid_size << std::endl;
        delete tree;
    }
    file.close();
}