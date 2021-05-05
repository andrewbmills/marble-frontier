#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <mapGrid3D.h>

TEST_CASE("Declaration", "[mapGrid3D]") {
    float resolution = 1.0;
    Dimensions sizes{30, 42, 15};
    Point mins{-1.5, -10.1, 100.0};
    MapGrid3D<float> map(resolution, sizes, mins);
    REQUIRE( map.maxBounds.x == (mins.x + (sizes.x-1)*resolution) );
    REQUIRE(map.maxBounds.y == (mins.y + (sizes.y-1)*resolution) );
    REQUIRE( map.maxBounds.z == (mins.z + (sizes.z-1)*resolution) );
    REQUIRE( map.voxels.size() == (sizes.x*sizes.y*sizes.z) );
}

TEST_CASE("Simple Set and Query", "[mapGrid3D]") {
    float resolution = 1.0;
    Dimensions sizes{10, 10, 10};
    Point mins{0.0, 0.0, 0.0};
    MapGrid3D<float> map(resolution, sizes, mins);
    float x = 3.0; float y = 4.0; float z = 5.0;
    float value = 2.0;
    map.SetVoxel(x, y, z, value);
    REQUIRE( map.Query(x, y, z) == value );
    REQUIRE( map.Query(x + resolution/3.0, y, z) == value );
}

TEST_CASE("Check if a voxel is in and out of bounds", "[mapGrid3D]") {
    float resolution = 1.0;
    Dimensions sizes{10, 10, 10};
    Point mins{0.0, 0.0, 0.0};
    MapGrid3D<float> map(resolution, sizes, mins);
    Point query {3.1, 4.2, 5.3};
    REQUIRE( map._CheckVoxelPositionInBounds(query) ==  true );
    query.x = -1.2;
    REQUIRE( map._CheckVoxelPositionInBounds(query) ==  false );
}

TEST_CASE("Declaration with Reset()", "[mapGrid3D]") {
    MapGrid3D<float> map;
    float resolution = 1.0;
    float mins[3] = {0.0, 0.0, 0.0};
    float maxes[3] = {0.0, 0.0, 0.0};
    float default_value = 2.1;
    map.Reset(resolution, mins, maxes, default_value);
    float sum = 0;
    for (int i=0; i<map.voxels.size(); i++) sum += map.Query(i);
    REQUIRE( sum == (default_value*map.voxels.size()) );
}

TEST_CASE("Set Compute Time", "[mapGrid3D]") {

}

TEST_CASE("Query Compute Time", "[mapGrid3D]") {
    
}

// Add tests for neighbors6, neighbors26, and raycasts?