#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <chrono>
#include <frontier_octomap.h>

void ConvertPointVectorToPointCloud(std::vector<Point> v, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    for (int i=0; i<v.size(); i++) {
        pcl::PointXYZ p = {v[i].x, v[i].y, v[i].z};
        cloud->points.push_back(p);
    }
}

TEST_CASE("Frontier filtering", "frontier_filters")
{
    std::string filename = "/home/andrew/Desktop/octomaps/octomap_1.ot";
    octomap::AbstractOcTree* readTreeAbstract = octomap::AbstractOcTree::read(filename);
    octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(readTreeAbstract);
    tree->expand();
    std::vector<Point> frontiers = CalculateFrontierFromOctomap(tree);
    pcl::PointCloud<pcl::PointXYZ>::Ptr frontierCloudRaw(new pcl::PointCloud<pcl::PointXYZ>);
    ConvertPointVectorToPointCloud(frontiers, frontierCloudRaw);
    REQUIRE(frontiers.size() > 0);

    pcl::PointXYZ bbx_min = {-20.0, -100.0, -10.0};
    pcl::PointXYZ bbx_max = {200.0, 100.0, 10.0};
    pcl::PointCloud<pcl::PointXYZ>::Ptr frontierCloudRawBox(new pcl::PointCloud<pcl::PointXYZ>);
    FilterCloudByBoundingBox(frontierCloudRaw, frontierCloudRawBox, bbx_min, bbx_max);

    pcl::PointCloud<pcl::PointNormal>::Ptr frontierCloudFilteredNormals(new pcl::PointCloud<pcl::PointNormal>);
    float min_normal_z = 0.4;
    FilterFrontierByNormal(frontierCloudRawBox, frontierCloudFilteredNormals, 2.1*tree->getResolution(), min_normal_z);
    REQUIRE(frontierCloudFilteredNormals->points.size() > 0);

    int min_cluster_size = 50;
    pcl::PointCloud<pcl::PointNormal>::Ptr frontierCloudFilteredClustered(new pcl::PointCloud<pcl::PointNormal>);
    std::vector<pcl::PointIndices> frontierClusterIndices;
    FilterFrontierByCluster(frontierCloudFilteredNormals, frontierCloudFilteredClustered, frontierClusterIndices, 1.5*tree->getResolution(), min_cluster_size);
    REQUIRE(frontierCloudFilteredClustered->points.size() > 0);
}

TEST_CASE("Performance benchmarks for frontier filter calculations", "frontier_filters")
{
    std::ofstream file;
    file.open("/home/andrew/Desktop/octomaps/frontier_filters_urban_benchmarks.csv");

    int map_start = 1;
    int map_end = 15;
    std::string filename_prefix = "/home/andrew/Desktop/octomaps/octomap_urban_practice_";

    for (int i=map_start; i<(map_end+1); i++) {
        std::string filename = filename_prefix + std::to_string(i) + ".ot";
        octomap::AbstractOcTree* readTreeAbstract = octomap::AbstractOcTree::read(filename);
        octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(readTreeAbstract);
        tree->expand();
        
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<Point> frontiers = CalculateFrontierFromOctomap(tree);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration_raw = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        INFO(frontiers.size() << " frontiers found in " << duration_raw.count() << " ms for octomap of size " << tree->size() << " using OcTree");
        REQUIRE(frontiers.size() > 0);

        pcl::PointCloud<pcl::PointXYZ>::Ptr frontierCloudRaw(new pcl::PointCloud<pcl::PointXYZ>);
        ConvertPointVectorToPointCloud(frontiers, frontierCloudRaw);
        pcl::PointCloud<pcl::PointNormal>::Ptr frontierCloudFilteredNormals(new pcl::PointCloud<pcl::PointNormal>);

        float min_normal_z = 0.4;
        start = std::chrono::high_resolution_clock::now();
        FilterFrontierByNormal(frontierCloudRaw, frontierCloudFilteredNormals, 2.1*tree->getResolution(), min_normal_z);
        end = std::chrono::high_resolution_clock::now();
        auto duration_normal = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        REQUIRE(frontierCloudFilteredNormals->points.size() > 0);

        int min_cluster_size = 50;
        pcl::PointCloud<pcl::PointNormal>::Ptr frontierCloudFilteredClustered(new pcl::PointCloud<pcl::PointNormal>);
        std::vector<pcl::PointIndices> frontierClusterIndices;
        start = std::chrono::high_resolution_clock::now();
        FilterFrontierByCluster(frontierCloudFilteredNormals, frontierCloudFilteredClustered, frontierClusterIndices, 1.5*tree->getResolution(), min_cluster_size);
        end = std::chrono::high_resolution_clock::now();
        auto duration_cluster = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        REQUIRE(frontierCloudFilteredClustered->points.size() > 0);

        file << i << "," << tree->size() << "," << frontiers.size() << "," << duration_raw.count() << ","
        << frontierCloudFilteredNormals->points.size() << "," << duration_normal.count() << "," 
        << frontierCloudFilteredClustered->points.size() << "," << duration_cluster.count() << ","
        << std::endl;
        delete tree;
    }
    file.close();
}