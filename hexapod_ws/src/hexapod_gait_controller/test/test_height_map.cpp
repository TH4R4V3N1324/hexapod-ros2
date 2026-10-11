#include <gtest/gtest.h>

#include "hexapod_gait_controller/height_map.hpp"

using hexapod_gait_controller::HeightMap;
using Eigen::Vector2d;
using Eigen::Vector3d;

/**
 * @brief Points on a floor 1 m square around the origin, with a 3 cm slab where x > 0.2
 * @return The points, 5 mm apart
*/
static std::vector<Vector3d> floorWithSlab() {
    std::vector<Vector3d> points;
    for (double x = -0.5; x <= 0.5; x += 0.005) {
        for (double y = -0.5; y <= 0.5; y += 0.005) {
            points.emplace_back(x, y, x > 0.2 ? 0.03 : 0.0);
        }
    }
    return points;
}

TEST(HeightMap, UnseenGroundHasNoHeight) {
    HeightMap map;
    EXPECT_FALSE(map.height(Vector2d(0.0, 0.0)).has_value());
}

TEST(HeightMap, HeightsComeFromThePoints) {
    HeightMap map;
    map.insert(floorWithSlab());
    EXPECT_NEAR(*map.height(Vector2d(0.0, 0.0)), 0.0, 1e-9);
    EXPECT_NEAR(*map.height(Vector2d(0.3, 0.1)), 0.03, 1e-9);
}

TEST(HeightMap, CellsFollowNewMeasurements) {
    HeightMap map;
    map.insert({Vector3d(0.01, 0.01, 0.0)});
    map.insert({Vector3d(0.01, 0.01, 0.04)});
    EXPECT_NEAR(*map.height(Vector2d(0.01, 0.01)), 0.02, 1e-9);  // halfway after one frame
}

TEST(HeightMap, PathOverTheSlabFindsItsTop) {
    HeightMap map;
    map.insert(floorWithSlab());
    EXPECT_NEAR(*map.maxAlong(Vector2d(0.0, 0.0), Vector2d(0.3, 0.0), 0.01), 0.03, 1e-9);
    EXPECT_NEAR(*map.maxAlong(Vector2d(0.0, 0.0), Vector2d(0.15, 0.0), 0.01), 0.0, 1e-9);
}

TEST(HeightMap, EdgesAreRough) {
    HeightMap map;
    map.insert(floorWithSlab());
    EXPECT_NEAR(*map.roughness(Vector2d(0.0, 0.0), 0.03), 0.0, 1e-9);
    EXPECT_NEAR(*map.roughness(Vector2d(0.2, 0.0), 0.03), 0.03, 1e-9);
}

TEST(HeightMap, FootholdMovesOffAnEdge) {
    HeightMap map;
    map.insert(floorWithSlab());
    // Flat ground stays where it is
    EXPECT_NEAR((*map.flatFoothold(Vector2d(0.0, 0.0), 0.04, 0.025, 0.01) - Vector2d(0.0, 0.0)).norm(), 0.0, 1e-9);
    // Next to the edge it moves at least 2.5 cm away from it, and no further than 4 cm
    const Vector2d moved = *map.flatFoothold(Vector2d(0.205, 0.0), 0.04, 0.025, 0.01);
    EXPECT_GT(std::abs(moved.x() - 0.2), 0.025);
    EXPECT_LE((moved - Vector2d(0.205, 0.0)).norm(), 0.04 + 1e-9);
}

TEST(HeightMap, PruneDropsFarCells) {
    HeightMap map;
    map.insert({Vector3d(0.0, 0.0, 0.0), Vector3d(1.0, 0.0, 0.0)});
    map.prune(Vector2d(0.0, 0.0), 0.5);
    EXPECT_TRUE(map.height(Vector2d(0.0, 0.0)).has_value());
    EXPECT_FALSE(map.height(Vector2d(1.0, 0.0)).has_value());
}
