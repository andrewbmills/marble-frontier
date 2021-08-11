#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <chrono>
#include <cmath>
#include "frontier_octomap.h"
#include "poseSampling.h"
#include "gain.h"
#include "fast_gain.hpp"

void LoadFibonacci50()
{
  fibonacciSphere50 << 0.98,0.072111692630594,-0.185472110534039,
    0.94,-0.30595937918124,0.150959790311958,
    0.9,0.418517327009501,0.121832864994721,
    0.86,-0.264933775465341,-0.43613082282462,
    0.82,-0.111320249301854,0.561433702315218,
    0.78,0.504380591080092,-0.370405479632385,
    0.74,-0.668673358221108,-0.0726356662067993,
    0.7,0.471412530700303,0.536442192504221,
    0.66,0.0155236451629398,-0.751105196654141,
    0.62,-0.541832329175321,0.567466058069068,
    0.58,0.812795688438898,-0.0544349966027162,
    0.54,-0.657221652859273,-0.525794350495443,
    0.5,0.133192374223317,0.855721795590574,
    0.46,0.491949917567831,-0.739178786630815,
    0.42,-0.881090807080997,0.217437323560969,
    0.38,0.811893086846763,0.443203808118155,
    0.34,-0.304280186736367,-0.889839068573459,
    0.3,-0.38212419102167,0.874060125298045,
    0.26,0.882854951181119,-0.391110131772349,
    0.22,-0.924555559380856,-0.311122190815685,
    0.18,0.475526459442382,0.861089186072031,
    0.14,0.232541985119214,-0.962457388748622,
    0.1,-0.825612568952115,0.55530521876558,
    0.06,0.987059408180681,0.148706841544036,
    0.02,-0.628383482417565,-0.777646577196077,
    -0.0200000000000001,-0.0619422918284693,0.997879327615838,
    -0.0600000000000001,0.71857916320349,-0.692852066612904,
    -0.1,-0.994662865198296,0.0254122922129577,
    -0.14,0.746951063378237,0.649972390889122,
    -0.18,-0.110996672397271,-0.977384130583637,
    -0.22,-0.573566192213418,0.78906389041053,
    -0.26,0.946241986632696,-0.192421679478712,
    -0.3,-0.817705483480962,-0.491281734125304,
    -0.34,0.267254131911314,0.901651389937558,
    -0.38,0.405228219178532,-0.831498701370842,
    -0.42,-0.844227536614828,0.332986285635399,
    -0.46,0.829129003680105,0.317718578708323,
    -0.5,-0.38697520002379,-0.774758152307252,
    -0.54,-0.231303504777414,0.80925811004752,
    -0.58,0.694153323341112,-0.426322839752328,
    -0.62,-0.770354364362138,-0.148842713319149,
    -0.66,0.447630630881683,0.603346350197353,
    -0.7,0.0736568579149558,-0.71033419408198,
    -0.74,-0.503069704462769,0.446453662155148,
    -0.78,0.625701539137303,0.00987845743070272,
    -0.82,-0.415887241337377,-0.393240133369912,
    -0.86,0.0365826888707612,0.50898104765795,
    -0.9,0.270639983440429,-0.341692843594015,
    -0.94,-0.33685530847208,0.0541156276336163,
    -0.98,0.166198214968986,0.109444750176164;
  return;
}

void ConvertPointVectorToPointCloud(std::vector<Point> v, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    for (int i=0; i<v.size(); i++) {
        pcl::PointXYZ p = {(float)v[i].x, (float)v[i].y, (float)v[i].z};
        cloud->points.push_back(p);
    }
}

void LabelFrontiersByCluster(pcl::PointCloud<pcl::PointNormal>::Ptr frontier, std::vector<pcl::PointIndices> clusters, 
pcl::PointCloud<pcl::PointXYZLNormal>::Ptr frontier_labeled)
{
    frontier_labeled->points.clear();
    for (int i=0; i<clusters.size(); i++) {
        for (int j=0; j<clusters[i].indices.size(); j++) {
            int idx = clusters[i].indices[j];
            pcl::PointNormal p = frontier->points[idx];
            pcl::PointXYZLNormal p_new;
            p_new.x = p.x; p_new.y = p.y; p_new.z = p.z;
            p_new.label = i;
            p_new.normal_x = p.normal_x; p_new.normal_y = p.normal_y; p_new.normal_z = p.normal_z;
            frontier_labeled->points.push_back(p_new);
        }
    }
    return;
}

void GetFrontier(octomap::OcTree* map, pcl::PointCloud<pcl::PointXYZLNormal>::Ptr frontier_out, std::vector<pcl::PointIndices> &cluster_indices)
{
    std::vector<Point> frontiers = CalculateFrontierFromOctomap(map);
    pcl::PointCloud<pcl::PointXYZ>::Ptr frontierCloudRaw(new pcl::PointCloud<pcl::PointXYZ>);
    ConvertPointVectorToPointCloud(frontiers, frontierCloudRaw);
    pcl::PointCloud<pcl::PointNormal>::Ptr frontierCloudFilteredNormals(new pcl::PointCloud<pcl::PointNormal>);
    float min_normal_z = 0.4;
    FilterFrontierByNormal(frontierCloudRaw, frontierCloudFilteredNormals, 2.1*map->getResolution(), min_normal_z);
    int min_cluster_size = 50;
    std::vector<pcl::PointIndices> frontierClusterIndices;
    pcl::PointCloud<pcl::PointNormal>::Ptr frontierCloudFilteredClustered(new pcl::PointCloud<pcl::PointNormal>);
    FilterFrontierByCluster(frontierCloudFilteredNormals, frontierCloudFilteredClustered, cluster_indices, 1.8*map->getResolution(), min_cluster_size);
    LabelFrontiersByCluster(frontierCloudFilteredClustered, cluster_indices, frontier_out);
    return;
}

void ConvertOctomapToEDT(octomap::OcTree* map, MapGrid3D<float>* edt)
{
  double xMin, yMin, zMin, xMax, yMax, zMax;
  edt->voxelSize = map->getResolution();
  map->expand();
  map->getMetricMin(xMin, yMin, zMin);
  map->getMetricMax(xMax, yMax, zMax); 
  double minBounds[3], maxBounds[3];
  AlignMinsToOctomap(map, minBounds);
  AlignMaxesToOctomap(map, maxBounds);
  edt->Reset(edt->voxelSize, minBounds, maxBounds, -1.0);
  for(octomap::OcTree::leaf_iterator it=map->begin_leafs(), end=map->end_leafs(); it!=end; ++it) {
    edt->SetVoxel(it.getX(), it.getY(), it.getZ(), map->getResolution()*(it->getOccupancy()<=0.5));
  }
  return;
}

void ConvertOctomapToCloud(octomap::OcTree* map, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    cloud->points.clear();
    for(octomap::OcTree::leaf_iterator it=map->begin_leafs(), end=map->end_leafs(); it!=end; ++it) {
        pcl::PointXYZI point;
        point.x = (float)it.getX(); point.y = (float)it.getY(); point.z = (float)it.getZ();
        point.intensity = 1.0*(it->getOccupancy() <= 0.5);
        cloud->points.push_back(point);
    }
    return;
}

void ConvertOctomapToOccupiedCloud(octomap::OcTree* map, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    cloud->points.clear();
    for(octomap::OcTree::leaf_iterator it=map->begin_leafs(), end=map->end_leafs(); it!=end; ++it) {
        pcl::PointXYZ point;
        point.x = (float)it.getX(); point.y = (float)it.getY(); point.z = (float)it.getZ();
        if (it->getOccupancy() >= 0.5) cloud->points.push_back(point);
    }
    return;
}

TEST_CASE("Sample goal poses in test octomap environments", "goals")
{
    int map_number = 1;
    std::string filename_prefix = "/home/andrew/Desktop/octomaps/octomap_urban_practice_";
    std::string filename = filename_prefix + std::to_string(map_number) + ".ot";
    octomap::AbstractOcTree* readTreeAbstract = octomap::AbstractOcTree::read(filename);
    octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(readTreeAbstract);

    // Get frontier pointcloud
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr frontier (new pcl::PointCloud<pcl::PointXYZLNormal>);
    std::vector<pcl::PointIndices> clusters;
    auto start = std::chrono::high_resolution_clock::now();
    GetFrontier(tree, frontier, clusters);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    INFO(frontier->points.size() << " frontiers labeled from map of size " << tree->size() << " in " << duration.count() << " ms.");

    // Test greedy grouping
    start = std::chrono::high_resolution_clock::now();
    std::vector<Group> groups;
    greedyGrouping(4.1*tree->getResolution(), frontier, groups);
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    INFO(groups.size() << " greedy groups created in " << duration.count() << " ms from " << frontier->points.size() << " frontier voxels");
    // for (int i=0; i<groups.size(); i++) {
    //     INFO("Group " << i << " contains " << groups[i].indices.indices.size() << " frontiers.");
    // }
    REQUIRE(groups.size() > 0);

    SensorFoV robotSensor;
    robotSensor.horizontalFoV = 80.0;
    robotSensor.verticalFoV = 40.0;
    robotSensor.rMin = 1.0;
    robotSensor.rMax = 5.0;
    robotSensor.type = "camera";
    MapGrid3D<float> edt;
    ConvertOctomapToEDT(tree, &edt);
    LoadFibonacci50();
    start = std::chrono::high_resolution_clock::now();
    std::vector<View> goals;
    int sample_limit = 5;
    float min_distance_from_obstacle = tree->getResolution()/2.0; // meters
    // SampleGoals(goals, groups, robotSensor, edt, tree, sample_limit, min_distance_from_obstacle, "uniform");
    // std::vector<Group> groups_abridged;
    // groups_abridged.resize(50);
    // std::copy(groups.begin(), groups.begin()+50, groups_abridged.begin());
    SampleGoals(goals, groups, robotSensor, edt, tree, sample_limit, min_distance_from_obstacle, "fibonacci");
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // INFO(goals.size() << " goals sampled uniformly in " << duration.count() << " ms for octomap of size " << tree->size());
    INFO(goals.size() << " goals sampled using priority queue in " << duration.count() << " ms for octomap of size " << tree->size());
    REQUIRE(goals.size() > 0);

    SensorParams sensor_params;
    sensor_params.hfov = robotSensor.horizontalFoV*M_PI/180.0;
    sensor_params.vfov = robotSensor.verticalFoV*M_PI/180.0;
    sensor_params.range = robotSensor.rMax;
    sensor_params.beam_angle = 2.5*M_PI/180.0;
    start = std::chrono::high_resolution_clock::now();
    FastGainUpdater gain_updater(edt, sensor_params);
    std::vector<View> goals_abridged;
    goals_abridged.resize(5);
    std::copy(goals.begin(), goals.begin()+5, goals_abridged.begin());
    std::vector<float> gains = gain_updater.get_gains(goals);
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    INFO("Approximate gains calculated in " << duration.count() << " ms.");
    REQUIRE(gains[5] > 0.0);

    // Publish to ROS to visualize in rviz
    // int argc = 0;
    // char* arg = "";
    // char** argv = &arg;
    // std::string node_name = "goal_pose_sampling_test_node";
    // ros::init(argc, argv, node_name);
    // ros::NodeHandle n;
    // ros::Publisher pub_goal_pose_array = n.advertise<geometry_msgs::PoseArray>("goal_poses", 1);
    // geometry_msgs::PoseArray goal_pose_array_msg = ConvertGoalsToPoseArray(goals);
    // goal_pose_array_msg.header.frame_id = "world";
    // goal_pose_array_msg.header.stamp = ros::Time::now();
    
    // sensor_msgs::PointCloud2 frontier_msg;
    // ros::Publisher pub_frontier = n.advertise<sensor_msgs::PointCloud2>("frontier", 1);
    // pcl::toROSMsg(*frontier, frontier_msg);
    // frontier_msg.header.frame_id = "world";

    // pcl::PointCloud<pcl::PointXYZ>::Ptr occ_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // ConvertOctomapToOccupiedCloud(tree, occ_cloud);
    // sensor_msgs::PointCloud2 occ_cloud_msg;
    // ros::Publisher pub_octomap = n.advertise<sensor_msgs::PointCloud2>("occupied", 1);
    // pcl::toROSMsg(*occ_cloud, occ_cloud_msg);
    // occ_cloud_msg.header.frame_id = "world";

    // ros::Rate r(1.0);
    // while (ros::ok()) {
    //     r.sleep();
    //     ros::spinOnce();
    //     pub_goal_pose_array.publish(goal_pose_array_msg);
    //     pub_frontier.publish(frontier_msg);
    //     pub_octomap.publish(occ_cloud_msg);
    // }
}

// TEST_CASE("Benchmark grouping, sampling, and gain calculation pieces", "goals")
// {
//     SensorFoV robotSensor;
//     robotSensor.horizontalFoV = 80.0;
//     robotSensor.verticalFoV = 40.0;
//     robotSensor.rMin = 1.0;
//     robotSensor.rMax = 5.0;
//     robotSensor.type = "camera";

//     ros::Publisher pubPlaceholder;
//     std_msgs::Header cloudHeader;

//     int map_start = 1;
//     int map_end = 15;
//     std::ofstream file;
//     file.open("/home/andrew/Desktop/octomaps/goal_pose_sampling_benchmarks.csv");
//     std::string filename_prefix = "/home/andrew/Desktop/octomaps/octomap_";
//     // std::string filename_prefix = "/home/andrew/Desktop/octomaps/octomap_urban_practice_";

//     for (int i=map_start; i<(map_end+1); i++) {
//         std::string filename = filename_prefix + std::to_string(i) + ".ot";
//         octomap::AbstractOcTree* readTreeAbstract = octomap::AbstractOcTree::read(filename);
//         octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(readTreeAbstract);
//         tree->expand();

//         pcl::PointCloud<pcl::PointXYZLNormal>::Ptr frontier (new pcl::PointCloud<pcl::PointXYZLNormal>);
//         std::vector<pcl::PointIndices> clusters;
//         GetFrontier(tree, frontier, clusters);

//         auto start = std::chrono::high_resolution_clock::now();
//         std::vector<Group> groups;
//         greedyGrouping(4.1*tree->getResolution(), frontier, groups);
//         auto end = std::chrono::high_resolution_clock::now();
//         auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
//         INFO(groups.size() << " greedy groups found in " << duration.count() << " ms for octomap of size " << tree->size());
//         file << i << frontier->points.size() << "," << tree->size() << "," << duration.count() << "," << groups.size();
//         REQUIRE(groups.size() > 0);

//         MapGrid3D<float> edt;
//         ConvertOctomapToEDT(tree, &edt);
//         start = std::chrono::high_resolution_clock::now();
//         std::vector<View> goals = SampleGoals(groups, robotSensor, edt, tree, 50, 0.6, "uniform");
//         end = std::chrono::high_resolution_clock::now();
//         duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
//         INFO(goals.size() << " goal sampled in " << duration.count() << " ms");
//         REQUIRE(goals.size() > 0);
//         file << "," << duration.count() << "," << goals.size();

//         // pcl::PointCloud<pcl::PointXYZI>::Ptr edtCloud (new pcl::PointCloud<pcl::PointXYZI>);
//         // ConvertOctomapToCloud(tree, edtCloud);

//         // start = std::chrono::high_resolution_clock::now();
//         // pcl::PointCloud<pcl::PointXYZI>::Ptr mapCloud (new pcl::PointCloud<pcl::PointXYZI>);
//         // FilterCloudNearGoals(edtCloud, mapCloud, goals, robotSensor, tree->getResolution(), frontier, "unseen");
//         // for (int i=0; i<goals.size(); i++) {
//         //     pcl::PointCloud<pcl::PointXYZI>::Ptr seenCloud (new pcl::PointCloud<pcl::PointXYZI>);
//         //     geometry_msgs::Pose goalPose = ConvertPoseToGeometryMsgPose(goals[i].pose);
//         //     goals[i].gain = GainDebug(goalPose, robotSensor, mapCloud, seenCloud, pubPlaceholder, cloudHeader, tree, tree->getResolution(), "unseen", "normal", true);
//         // }
//         // end = std::chrono::high_resolution_clock::now();
//         // duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
//         // INFO(goals.size() << " gains computed in " << duration.count() << " ms");
//         // float gain_average = 0.0;
//         // for (int i=0; i<goals.size(); i++) gain_average += (goals[i].gain)/goals.size();
//         // REQUIRE(gain_average > 0.0);
//         // file << "," << duration.count() << "," << gain_average;
//         file << std::endl;
//     }


//     file.close();
// }