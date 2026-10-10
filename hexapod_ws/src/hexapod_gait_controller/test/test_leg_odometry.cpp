#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "hexapod_gait_controller/leg_odometry.hpp"

using hexapod_gait_controller::BodyMotion;
using hexapod_gait_controller::LegOdometry;
using Eigen::Vector2d;

// Three planted feet of a tripod, in the body frame (+x forward, +y left)
static const std::vector<Vector2d> TRIPOD = {{0.17, -0.19}, {-0.17, -0.19}, {0.0, 0.24}};

/**
 * @brief Where world-fixed feet appear in the body frame after the body moves
 * @param[in] feet The feet in the body frame before the move
 * @param[in] motion The body's move, in its frame before the move
 * @return The same feet in the body frame after the move
*/
static std::vector<Vector2d> afterBodyMove(const std::vector<Vector2d>& feet, const BodyMotion& motion) {
    Eigen::Rotation2Dd inverse(-motion.dyaw);
    std::vector<Vector2d> moved;
    for (const auto& foot : feet) {
        moved.push_back(inverse * (foot - Vector2d(motion.dx, motion.dy)));
    }
    return moved;
}

/**
 * @brief Check estimateMotion recovers a known body motion from the feet
 * @param[in] truth The body motion to apply
*/
static void expectRecovers(const BodyMotion& truth) {
    BodyMotion estimate;
    ASSERT_TRUE(LegOdometry::estimateMotion(TRIPOD, afterBodyMove(TRIPOD, truth), estimate));
    EXPECT_NEAR(estimate.dx, truth.dx, 1e-9);
    EXPECT_NEAR(estimate.dy, truth.dy, 1e-9);
    EXPECT_NEAR(estimate.dyaw, truth.dyaw, 1e-9);
}

TEST(LegOdometry, RecoversForward) {
    expectRecovers({0.002, 0.0, 0.0});
}

TEST(LegOdometry, RecoversLeft) {
    expectRecovers({0.0, 0.002, 0.0});
}

TEST(LegOdometry, RecoversTurnLeft) {
    expectRecovers({0.0, 0.0, 0.01});
}

TEST(LegOdometry, RecoversCombinedMotion) {
    expectRecovers({0.003, -0.001, -0.02});
}

TEST(LegOdometry, NeedsTwoFeet) {
    BodyMotion estimate;
    EXPECT_FALSE(LegOdometry::estimateMotion({{0.1, 0.1}}, {{0.1, 0.1}}, estimate));
    EXPECT_FALSE(LegOdometry::estimateMotion(TRIPOD, {{0.1, 0.1}}, estimate));
}

TEST(LegOdometry, IntegratesInTheOdomFrame) {
    LegOdometry odom;
    // Turn left 90 degrees in steps, then drive forward 1 m: should end up at (0, 1)
    for (int i = 0; i < 90; ++i) {
        odom.integrate({0.0, 0.0, M_PI / 180.0});
    }
    for (int i = 0; i < 100; ++i) {
        odom.integrate({0.01, 0.0, 0.0});
    }
    EXPECT_NEAR(odom.x(), 0.0, 1e-9);
    EXPECT_NEAR(odom.y(), 1.0, 1e-9);
    EXPECT_NEAR(odom.yaw(), M_PI / 2.0, 1e-9);
}
