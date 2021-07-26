#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <chrono>
#include <frontier_octomap.h>
#include <frontier_hashgrid.h>
#include <mapGrid3D.h>

void ConvertPointVectorToPointCloud(std::vector<Point> v, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    for (int i=0; i<v.size(); i++) {
        pcl::PointXYZ p = {v[i].x, v[i].y, v[i].z};
        cloud->points.push_back(p);
    }
}

void ConvertSeenOccGridToPointCloud(MapGrid3D<std::pair<bool,bool>>* seenOcc, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    cloud->points.clear();
    for (int i=0; i<seenOcc->voxels.size(); i++) {
        bool seen = seenOcc->voxels[i].first;
        bool occ = seenOcc->voxels[i].second;
        if (seen && occ) {
            Point p = seenOcc->_ConvertIndexToPosition(i);
            pcl::PointXYZ q = {p.x, p.y, p.z};
            cloud->points.push_back(q);
        }
    }
}

void TestConvertOctomapToSeenOccGrid(octomap::OcTree* map, MapGrid3D<std::pair<bool,bool>>* seenOccGrid)
{
  // Initialize MapGrid3D object of the appropriate size
  clock_t tStart = clock();
  double xMin, yMin, zMin, xMax, yMax, zMax;
  seenOccGrid->voxelSize = map->getResolution();
  map->expand();
  map->getMetricMin(xMin, yMin, zMin);
  map->getMetricMax(xMax, yMax, zMax);
  ROS_INFO("Octomap has bounds (%0.5f, %0.5f %0.5f) to (%0.5f, %0.5f %0.5f)", xMin, yMin, zMin, xMax, yMax, zMax);  
  double minBounds[3], maxBounds[3];
  AlignMinsToOctomap(map, minBounds);
  AlignMaxesToOctomap(map, maxBounds);
  seenOccGrid->Reset(seenOccGrid->voxelSize, minBounds, maxBounds, std::make_pair(false, false));
  ROS_INFO("SeenOccGrid has bounds (%0.5f, %0.5f %0.5f) to (%0.5f, %0.5f %0.5f)", seenOccGrid->minBounds.x, seenOccGrid->minBounds.y,
           seenOccGrid->minBounds.z, seenOccGrid->maxBounds.x, seenOccGrid->maxBounds.y, seenOccGrid->maxBounds.z);
  ROS_INFO("SeenOccGrid has sizes (%d, %d, %d)", seenOccGrid->size.x, seenOccGrid->size.y, seenOccGrid->size.z);
  
  for(octomap::OcTree::leaf_iterator it=map->begin_leafs(), end=map->end_leafs(); it!=end; ++it) {
    seenOccGrid->SetVoxel(it.getX(), it.getY(), it.getZ(), std::make_pair(true, (it->getOccupancy()>0.5)));
    Point p = {it.getX(), it.getY(), it.getZ()};
    int id = seenOccGrid->_ConvertPositionToIndex(p);
    Point p_query = seenOccGrid->_ConvertIndexToPosition(id);
    INFO("[" << it.getX() << ", " << it.getY() << ", " << it.getZ() << "] maps to id " << id);
    INFO("Id "<< id << " maps to [" << p_query.x << ", " << p_query.y << ", " << p_query.z << "]");
    REQUIRE( std::abs(it.getX() - p_query.x) <= seenOccGrid->voxelSize/2.0f );
    REQUIRE( std::abs(it.getY() - p_query.y) <= seenOccGrid->voxelSize/2.0f );
    REQUIRE( std::abs(it.getZ() - p_query.z) <= seenOccGrid->voxelSize/2.0f );
    REQUIRE( seenOccGrid->Query(it.getX(), it.getY(), it.getZ()).first == true);
    REQUIRE( seenOccGrid->Query(it.getX(), it.getY(), it.getZ()).second == (it->getOccupancy()>0.5));
  }
  ROS_INFO("Copying data from Octomap took: %.5fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  return;
}


void ConvertOctomapToPointCloud(octomap::OcTree* tree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    cloud->points.clear();
    for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end=tree->end_leafs(); it!=end; ++it) {
        if (it->getOccupancy() > 0.5) {
            pcl::PointXYZ p {it.getX(), it.getY(), it.getZ()};
            cloud->points.push_back(p);
        }
    }
    return;
}


TEST_CASE("Performance benchmarks for raw frontier calculations on map types", "frontier")
{
    HashGridExtents mapParams;
    mapParams.subMapSizes[0] = 20; mapParams.subMapSizes[1] = 20; mapParams.subMapSizes[2] = 20;
    mapParams.voxelSize = 0.1;

    int map_start = 1;
    int map_end = 15;

    std::ofstream file;
    // file.open("/home/andrew/Desktop/octomaps/frontier_urban_benchmarks.csv");
    // std::string filename_prefix = "/home/andrew/Desktop/octomaps/octomap_urban_practice_";
    file.open("/home/andrew/Desktop/octomaps/frontier_benchmarks.csv");
    std::string filename_prefix = "/home/andrew/Desktop/octomaps/octomap_";

    for (int i=map_start; i<(map_end+1); i++) {
        std::string filename = filename_prefix + std::to_string(i) + ".ot";
        octomap::AbstractOcTree* readTreeAbstract = octomap::AbstractOcTree::read(filename);
        octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(readTreeAbstract);
        tree->expand();
        
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<Point> frontiers = CalculateFrontierFromOctomap(tree);
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
        CalculateFrontierRawPCL(&seenOcc, frontiers);
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
        std::vector<Point> frontiers = CalculateFrontierFromHashGrid3D(&map);
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