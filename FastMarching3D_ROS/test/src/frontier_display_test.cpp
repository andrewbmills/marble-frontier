#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <ros/ros.h>
#include <chrono>
#include <frontier.h>
#include <frontier_hashgrid.h>
#include <frontier_octomap.h>

void ConvertPointVectorToPointCloud(std::vector<Point> v, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    for (int i=0; i<v.size(); i++) {
        pcl::PointXYZ p = {v[i].x, v[i].y, v[i].z};
        cloud->points.push_back(p);
    }
}

void ConvertHashGrid3DToOccupiedCloud(HashGrid3D<int>* map, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    cloud->points.clear();
    std::vector<int> data = map->GetData();
    for (int i=0; i<data.size(); i++) {
        if (data[i] == 1) {
            std::vector<double> position = map->_GetPositionFromId(i);
            pcl::PointXYZ p = {(float)position[0], (float)position[1], (float)position[2]};
            cloud->points.push_back(p);
        }
    }
    return;
}

TEST_CASE("Publish frontier for each case to a ROS msg for debugging purposes", "frontier")
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
    ros::Publisher pubOctomap = n.advertise<octomap_msgs::Octomap>("octomap", 5);
    ros::Publisher pubOccupiedHashGrid = n.advertise<sensor_msgs::PointCloud2>("occupied_hashgrid", 5);

    sensor_msgs::PointCloud2 frontierMsgOcTree;
    sensor_msgs::PointCloud2 frontierMsgMapGrid;
    sensor_msgs::PointCloud2 frontierMsgHashGrid;
    sensor_msgs::PointCloud2 occupiedMsgHashGrid;
    octomap_msgs::Octomap map_msg;

    int map_number = 14;
    // std::string filename_prefix = "/home/andrew/Desktop/octomaps/octomap_";
    std::string filename_prefix = "/home/andrew/Desktop/octomaps/octomap_urban_practice_";
    std::string filename = filename_prefix + std::to_string(map_number) + ".ot";
    octomap::AbstractOcTree* readTreeAbstract = octomap::AbstractOcTree::read(filename);
    octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(readTreeAbstract);
    tree->expand();
    octomap_msgs::fullMapToMsg(*tree, map_msg);
    map_msg.header.frame_id = "world";
    
    std::vector<Point> frontiers = CalculateFrontierFromOctomap(tree);
    INFO(frontiers.size() << " frontiers found for octomap of size " << tree->size() << " using OcTree");
    REQUIRE(frontiers.size() > 0);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr frontiersCloud(new pcl::PointCloud<pcl::PointXYZ>);
    ConvertPointVectorToPointCloud(frontiers, frontiersCloud);
    pcl::toROSMsg(*frontiersCloud, frontierMsgOcTree);
    frontierMsgOcTree.header.frame_id = "world";

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
    HashGrid3D<int> map = ConvertOcTreeBinaryToHashGrid3D(tree, mapParams);
    frontiers = CalculateFrontierFromHashGrid3D(&map);

    INFO(frontiers.size() << " frontiers found using hashgrid3D for octomap of size " << tree->size() << " using OcTree");
    REQUIRE(frontiers.size() > 0);

    frontiersCloud->points.clear();
    ConvertPointVectorToPointCloud(frontiers, frontiersCloud);
    pcl::toROSMsg(*frontiersCloud, frontierMsgHashGrid);
    frontierMsgHashGrid.header.frame_id = "world";

    pcl::PointCloud<pcl::PointXYZ>::Ptr occCloud(new pcl::PointCloud<pcl::PointXYZ>);
    ConvertHashGrid3DToOccupiedCloud(&map, occCloud);
    pcl::toROSMsg(*occCloud, occupiedMsgHashGrid);
    occupiedMsgHashGrid.header.frame_id = "world";
    delete tree;    

    ros::Rate r(1.0);
    while (ros::ok()) {
        r.sleep();
        ros::spinOnce();
        pubOctomap.publish(map_msg);
        pubFrontierMapGrid.publish(frontierMsgMapGrid);
        pubFrontierOcTree.publish(frontierMsgOcTree);
        pubFrontierHashGrid.publish(frontierMsgHashGrid);
        pubOccupiedHashGrid.publish(occupiedMsgHashGrid);
    }
}
