#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <hashGrid3D.h>
#include <chrono>

TEST_CASE("Declaration, Set, and Query", "hashGrid3D")
{
    double mins[3] = {0.0, 0.0, 0.0};
    double maxes[3] = {100.0, 100.0, 100.0};
    double resolution = 1.0;
    int submap_sizes[3] = {10, 10, 10};
    HashGrid3D<double> map(mins, maxes, submap_sizes, 0.0, resolution);
    
    SECTION("Initialized HashGridMap3D member value checks")
    {
        std::vector<int> size = map.GetSizes();
        for (int i=0; i<3; i++) REQUIRE( size[i] == 11 );
        std::vector<int> keys = map.GetKeys();
        int sum = 0;
        for (int i=0; i<keys.size(); i++) sum += keys[i];
        REQUIRE( sum == ((-1)*keys.size()) );
    }

    SECTION("Set a voxel and Query that voxel")
    {
        map.Set(0.0, 0.0, 0.0, 1.1);
        map.Set(6.0, 8.0, 9.0, 1.2);
        REQUIRE( map.Get(0.0, 0.0, 0.0) == 1.1 );
        REQUIRE( map.Get(6.0, 8.0, 9.0) == 1.2 );
        
        std::vector<std::vector<double>> minList = map.GetMinList();
        REQUIRE( minList.size() == 1);

        map.Set(16.0, 8.0, 9.0, 2.1);
        minList = map.GetMinList();
        REQUIRE( minList.size() == 2);

        map.Set(16.0, 7.0, 9.0, 2.2);
        map.Set(15.0, 8.0, 9.0, 2.3);
        map.Set(16.0, 8.0, 3.0, 2.4);
        minList = map.GetMinList();
        REQUIRE( minList.size() == 2);
        REQUIRE( map.Get(16.0, 8.0, 3.0) == 2.4 );

        map.Set(100.0, 100.0, 100.0, 3.1);
        minList = map.GetMinList();
        REQUIRE( minList.size() == 3);
        REQUIRE( map.Get(100.0, 100.0, 100.0) == 3.1 );
    }

    SECTION("Check conversions from position to index and back.")
    {
        double x = 6.2; double y = 8.1; double z = 8.95;
        map.Set(x, y, z, 1.2);
        std::vector<double> data = map.GetData();
        std::vector<int> keys = map.GetKeys();
        int key = keys[map._GetKeyId(x - mins[0], y - mins[1], z - mins[2])];
        int id_point = map._GetSubMapId(x - mins[0], y - mins[1], z - mins[2]) + key*submap_sizes[0]*submap_sizes[1]*submap_sizes[2];
        std::vector<std::vector<double>> min_list = map.GetMinList();
        std::vector<double> point = map._GetPositionFromId(id_point);
        double dx = x - point[0]; double dy = y - point[1]; double dz = z - point[2];
        double half_voxel = map.GetVoxelSize()/2.0f;
        INFO("Key = " << key);
        INFO("Id = " << id_point);
        INFO("Point = [" << point[0] << ", " << point[1] << ", " << point[2] << "]");
        INFO("SubMap mins = [" << min_list[key][0] << ", " << min_list[key][1] << ", " << min_list[key][2] << "]");
        REQUIRE( std::abs(dx) <= half_voxel );
        REQUIRE( std::abs(dy) <= half_voxel );
        REQUIRE( std::abs(dz) <= half_voxel );
    }
}

TEST_CASE("See what voxels and subMaps have changed.", "hashGrid3D")
{
    double mins[3] = {0.0, 0.0, 0.0};
    double maxes[3] = {100.0, 100.0, 100.0};
    double resolution = 1.0;
    int submap_sizes[3] = {10, 10, 10};
    HashGrid3D<double> map(mins, maxes, submap_sizes, 0.0, resolution);

    SECTION("Change a set of voxels and retrieve the diff lists.")
    {
        // map.SetDiff(100.0, 100.0, 100.0, 2.0);
        // map.GetDifferences();
    }
}

struct Point { double x,y,z; };
double Randomdouble(double a, double b) {
    double random = ((double) rand()) / (double) RAND_MAX;
    double diff = b - a;
    double r = random * diff;
    return a + r;
}
std::vector<Point> GenerateRandomPointsOnInterval(double mins[3], double maxes[3], int N)
{
    std::vector<Point> points;
    for (int i=0; i<N; i++) {
        Point p = {Randomdouble(mins[0], maxes[0]), Randomdouble(mins[1], maxes[1]), Randomdouble(mins[2], maxes[2])};
        points.push_back(p);
    }
    return points;
}

TEST_CASE("Benchmark Set and Get Methods.", "hashGrid3D")
{
    double mins[3] = {0.0, 0.0, 0.0};
    double maxes[3] = {100.0, 100.0, 100.0};
    double resolution = 1.0;
    int submap_sizes[3] = {10, 10, 10};
    HashGrid3D<double> map(mins, maxes, submap_sizes, -1.0, resolution);
    int N = 1000000;
    std::vector<Point> randomPoints = GenerateRandomPointsOnInterval(mins, maxes, N);

    SECTION("HashGrid Set and Get, N = 1000000")
    {
        auto start = std::chrono::high_resolution_clock::now();
        for (int i=0; i<(randomPoints.size()); i++) {
            map.Set(randomPoints[i].x, randomPoints[i].y, randomPoints[i].z, 1.0);
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto duration_set = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        INFO(N << " sets performed in " << duration_set.count() << " ms");
        REQUIRE( duration_set.count() >= 0.0);

        start = std::chrono::high_resolution_clock::now();
        double total = 0.0;
        for (int i=0; i<(randomPoints.size()); i++) {
            total += map.Get(randomPoints[i].x, randomPoints[i].y, randomPoints[i].z);
        }
        end = std::chrono::high_resolution_clock::now();
        auto duration_get = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        INFO(N << " gets performed in " << duration_get.count() << " ms");
        REQUIRE( duration_get.count() >= 0.0);
        REQUIRE( total <= (N + 1));
    }
}