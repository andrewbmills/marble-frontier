#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <frontier_octomap.h>
#include <frontier_hashgrid.h>
#include <mapGrid3D.h>
#include <hashGrid3D.h>
#include <hashGrid3D_conversions.h>


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


TEST_CASE("Raw frontier calculations on OcTree, MapGrid3D, and HashGrid3D map types", "frontier")
{
    HashGridExtents mapParams;
    mapParams.subMapSizes[0] = 50; mapParams.subMapSizes[1] = 50; mapParams.subMapSizes[2] = 50;
    mapParams.voxelSize = 0.1;

    int map_start = 1;
    int map_end = 15;
    std::vector<int> required_frontier_sizes;
    // std::string filename_prefix = "/home/andrew/Desktop/octomaps/octomap_";
    std::string filename_prefix = "/home/andrew/Desktop/octomaps/octomap_urban_practice_";

    for (int i=map_start; i<(map_end+1); i++) {
        std::string filename = filename_prefix + std::to_string(i) + ".ot";
        octomap::AbstractOcTree* readTreeAbstract = octomap::AbstractOcTree::read(filename);
        octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(readTreeAbstract);
        tree->expand();
        
        std::vector<Point> frontiers = CalculateFrontierFromOctomap(tree);
        REQUIRE(frontiers.size() > 0);
        required_frontier_sizes.push_back(frontiers.size());
        delete tree;
    }  

    for (int i=map_start; i<(map_end+1); i++) {
        std::string filename = filename_prefix + std::to_string(i) + ".ot";
        octomap::AbstractOcTree* readTreeAbstract = octomap::AbstractOcTree::read(filename);
        octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(readTreeAbstract);
        tree->expand();

        MapGrid3D<std::pair<bool,bool>> seenOcc;
        ConvertOctomapToSeenOccGrid(tree, &seenOcc);
        // TestConvertOctomapToSeenOccGrid(tree, &seenOcc);
        pcl::PointCloud<pcl::PointXYZ>::Ptr frontiers(new pcl::PointCloud<pcl::PointXYZ>);
        CalculateFrontierRawPCL(&seenOcc, frontiers);
        REQUIRE((int)frontiers->points.size() == required_frontier_sizes[i-map_start]);
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

        HashGrid3D<int> map = ConvertOcTreeBinaryToHashGrid3D(tree, mapParams);
        std::vector<Point> frontiers = CalculateFrontierFromHashGrid3D(&map);
        REQUIRE((int)frontiers.size() == required_frontier_sizes[i-map_start]);
        delete tree;
    }
}