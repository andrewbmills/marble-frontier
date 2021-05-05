#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <chrono>
#include <frontier.h>
#include <mapGrid3D.h>
#include <hashGrid3D.h>
#include <hashGrid3D_conversions.h>
#include <octomap/octomap.h>
#include <octomap/AbstractOcTree.h>

void ConvertPointVectorToPointCloud(std::vector<Point> v, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    for (int i=0; i<v.size(); i++) {
        pcl::PointXYZ p = {v[i].x, v[i].y, v[i].z};
        cloud->points.push_back(p);
    }
}

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

// bool IsAdjacentToUnseen(double x, double y, double z, octomap::OcTree* tree)
// {
//     double voxel_size = tree->getResolution();
//     if (IsUnseen(x + voxel_size,y,z,tree)) return true;
//     else if (IsUnseen(x - voxel_size,y,z,tree)) return true;
//     else if (IsUnseen(x,y + voxel_size,z,tree)) return true;
//     else if (IsUnseen(x,y - voxel_size,z,tree)) return true;
//     else if (IsUnseen(x,y,z + voxel_size,tree)) return true;
//     else if (IsUnseen(x,y,z - voxel_size,tree)) return true;
//     else return false;
// }


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

// Frontier using hashGrid3D
bool IsUnseen(double x, double y, double z, HashGrid3D<int>* map)
{
    return (map->Get(x,y,z) == -1);
}

bool IsAdjacentToUnseen(double x, double y, double z, HashGrid3D<int>* map, double voxel_size)
{
    return IsUnseen(x + voxel_size, y, z, map)+IsUnseen(x - voxel_size, y, z, map)+
           IsUnseen(x, y + voxel_size, z, map)+IsUnseen(x, y - voxel_size, z, map)+
           IsUnseen(x, y, z + voxel_size, map)+IsUnseen(x, y, z - voxel_size, map);
}

bool IsAdjacentToUnseen2D(double x, double y, double z, HashGrid3D<int>* map, double voxel_size)
{
    return IsUnseen(x + voxel_size, y, z, map)+IsUnseen(x - voxel_size, y, z, map)+
           IsUnseen(x, y + voxel_size, z, map)+IsUnseen(x, y - voxel_size, z, map);
}

bool IsAdjacentToUnseenFast(int id, std::vector<int> &data, std::vector<int> &neighbor_deltas)
{
    for (int i=0; i<neighbor_deltas.size(); i++) {
        if (data[id + neighbor_deltas[i]] == -1) return true;
    }
    return false;
}

bool IsAdjacentToUnseenFast2D(int id, std::vector<int> &data, std::vector<int> &neighbor_deltas)
{
    for (int i=0; i<4; i++) {
        if (data[id + neighbor_deltas[i]] == -1) return true;
    }
    return false;
}

std::vector<int> GetNeighborDeltas(std::vector<int> sizes)
{
    std::vector<int> neighbor_deltas;
    neighbor_deltas.push_back(1); neighbor_deltas.push_back(-1);
    neighbor_deltas.push_back(sizes[0]); neighbor_deltas.push_back(-sizes[0]);
    neighbor_deltas.push_back(sizes[1]*sizes[0]); neighbor_deltas.push_back(-sizes[1]*sizes[0]);
    return neighbor_deltas;
}

std::vector<bool> GetSubMapEdgeTable(std::vector<int> sizes)
{
  std::vector<bool> subMapEdgeTable = std::vector<bool>(sizes[0]*sizes[1]*sizes[2]);
  for (int k=0; k<sizes[2]; k++) {
    for (int j=0; j<sizes[1]; j++) {
      for (int i=0; i<sizes[0]; i++) {
        int id = i + j*sizes[0] + k*sizes[0]*sizes[1];
        if ((i == 0) || (i == sizes[0]) || (j == 0) || (j == sizes[1]) ||(k == 0) || (k == sizes[2])) subMapEdgeTable[id] = true;
        else subMapEdgeTable[id] = false;
      }
    }
  }
  return subMapEdgeTable;
}

std::vector<Point> CalculateFrontierFromHashGrid3D(HashGrid3D<int>* map)
{
    double voxel_size = map->GetVoxelSize();
    std::vector<Point> frontiers;
    std::vector<int> data = map->GetData();
    std::vector<int> sizes = map->GetSubMapSizes();
    // std::vector<int> neighbor_deltas = GetNeighborDeltas(sizes);
    // std::vector<bool> edge_table = GetSubMapEdgeTable(sizes);
    int submap_size = sizes[0]*sizes[1]*sizes[2];
    INFO("Iterating through " << data.size() << " voxels to find frontiers.");
    for (int i=0; i<data.size(); i++) {
        bool isFree = (data[i] == 0);
        if (isFree) {
            // bool isEdge = edge_table[(i%submap_size)];
            // bool isEdge = true;
            // if (isEdge) {
                std::vector<double> position = map->_GetPositionFromId(i);
                bool isAdjacentToUnseen = IsAdjacentToUnseen(position[0], position[1], position[2], map, voxel_size);
                if (isAdjacentToUnseen) {
                    Point frontier = {(float)position[0], (float)position[1], (float)position[2]};
                    frontiers.push_back(frontier);
                }
            // } else {
            //     bool isAdjacentToUnseen = IsAdjacentToUnseenFast(i, data, neighbor_deltas);
            //     if (isAdjacentToUnseen) {
            //         std::vector<double> position = map->_GetPositionFromId(i);
            //         Point frontier = {(float)position[0], (float)position[1], (float)position[2]};
            //         frontiers.push_back(frontier);
            //     }
            // }
            
        }
    }
    return frontiers;
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
    int argc = 0;
    char* arg = "";
    char** argv = &arg;
    std::string node_name = "frontier_test_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;

    ros::Publisher pubFrontierOcTree = n.advertise<sensor_msgs::PointCloud2>("frontier_octree", 5);
    ros::Publisher pubFrontierMapGrid = n.advertise<sensor_msgs::PointCloud2>("frontier_mapgrid", 5);
    ros::Publisher pubFrontierHashGrid = n.advertise<sensor_msgs::PointCloud2>("frontier_hashgrid", 5);
    ros::Publisher pubOccCloud = n.advertise<sensor_msgs::PointCloud2>("occCloud", 5);
    ros::Publisher pubOctomap = n.advertise<octomap_msgs::Octomap>("octomap", 5);

    sensor_msgs::PointCloud2 frontierMsgOcTree;
    sensor_msgs::PointCloud2 frontierMsgMapGrid;
    sensor_msgs::PointCloud2 frontierMsgHashGrid;
    sensor_msgs::PointCloud2 occCloudMsg;
    octomap_msgs::Octomap map_msg;

    HashGridExtents mapParams;
    mapParams.mins[0] = -20.0; mapParams.mins[1] = -100.0; mapParams.mins[2] = -10.0;
    mapParams.maxes[0] = 200.0; mapParams.maxes[1] = 100.0; mapParams.maxes[2] =  5.0;
    mapParams.subMapSizes[0] = 20; mapParams.subMapSizes[1] = 20; mapParams.subMapSizes[2] = 20;
    mapParams.voxelSize = 0.1;

    int map_start = 1;
    int map_end = 15;

    std::ofstream file;
    file.open("/home/andrew/Desktop/octomaps/frontier_benchmarks.csv");

    for (int i=map_start; i<(map_end+1); i++) {
        std::string filename = "/home/andrew/Desktop/octomaps/octomap_" + std::to_string(i) + ".ot";
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
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr frontiersCloud(new pcl::PointCloud<pcl::PointXYZ>);
        ConvertPointVectorToPointCloud(frontiers, frontiersCloud);
        pcl::toROSMsg(*frontiersCloud, frontierMsgOcTree);
        frontierMsgOcTree.header.frame_id = "world";
        delete tree;
    }  

    for (int i=map_start; i<(map_end+1); i++) {
        std::string filename = "/home/andrew/Desktop/octomaps/octomap_" + std::to_string(i) + ".ot";
        octomap::AbstractOcTree* readTreeAbstract = octomap::AbstractOcTree::read(filename);
        octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(readTreeAbstract);
        tree->expand();

        auto start = std::chrono::high_resolution_clock::now();
        MapGrid3D<std::pair<bool,bool>> seenOcc;
        ConvertOctomapToSeenOccGrid(tree, &seenOcc);
        // TestConvertOctomapToSeenOccGrid(tree, &seenOcc);
        pcl::PointCloud<pcl::PointXYZ>::Ptr frontiers(new pcl::PointCloud<pcl::PointXYZ>);
        CalculateFrontierRawPCL(&seenOcc, frontiers);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        INFO(frontiers->points.size() << " frontiers found in " << duration.count() << " ms for octomap of size " << tree->size() << " using MapGrid3D");
        double mapGrid_size = seenOcc.size.x*seenOcc.size.y*seenOcc.size.z*2.0/1000.0;
        INFO("MapGrid3D is of size " << mapGrid_size << "kB");
        REQUIRE(frontiers->points.size() > 0);
        file << i << "," << frontiers->points.size() << "," << duration.count() << "," << tree->size() << "," << mapGrid_size << std::endl;
        pcl::toROSMsg(*frontiers, frontierMsgMapGrid);
        frontierMsgMapGrid.header.frame_id = "world";
        pcl::PointCloud<pcl::PointXYZ>::Ptr occCloud(new pcl::PointCloud<pcl::PointXYZ>);
        ConvertSeenOccGridToPointCloud(&seenOcc, occCloud);
        pcl::toROSMsg(*occCloud, occCloudMsg);
        occCloudMsg.header.frame_id = "world";
        delete tree;
    }

    // TO-DO: Implement HashGrid3D test
    for (int i=map_start; i<(map_end+1); i++) {
        std::string filename = "/home/andrew/Desktop/octomaps/octomap_" + std::to_string(i) + ".ot";
        octomap::AbstractOcTree* readTreeAbstract = octomap::AbstractOcTree::read(filename);
        octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(readTreeAbstract);
        tree->expand();
        octomap_msgs::fullMapToMsg(*tree, map_msg);
        map_msg.header.frame_id = "world";

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

        pcl::PointCloud<pcl::PointXYZ>::Ptr frontiersCloud(new pcl::PointCloud<pcl::PointXYZ>);
        ConvertPointVectorToPointCloud(frontiers, frontiersCloud);
        pcl::toROSMsg(*frontiersCloud, frontierMsgHashGrid);
        frontierMsgHashGrid.header.frame_id = "world";
        delete tree;
    }

    ros::Rate r(1.0);
    while (ros::ok()) {
        r.sleep();
        ros::spinOnce();
        pubOctomap.publish(map_msg);
        pubFrontierMapGrid.publish(frontierMsgMapGrid);
        pubFrontierOcTree.publish(frontierMsgOcTree);
        pubFrontierHashGrid.publish(frontierMsgHashGrid);
        pubOccCloud.publish(occCloudMsg);
    }

    file.close();
}