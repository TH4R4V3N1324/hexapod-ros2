#include <gtest/gtest.h>

#include <cmath>

#include "hexapod_gait_controller/gait_config.hpp"
#include "hexapod_gait_controller/trajectory_generator.hpp"

using hexapod_gait_controller::BodyPose;
using hexapod_gait_controller::GaitConfig;
using hexapod_gait_controller::MAX_LEGS;
using hexapod_gait_controller::TrajectoryGenerator;
using Eigen::Vector3d;

// Legs 1-3 are on the right, 4-6 on the left; 1 and 6 at the front, 3 and 4 at the rear

/**
 * @brief Each foot's position (leg frame) after the body takes up a pose over the start positions
 * @param[in] pose The body pose
 * @return The foot position of each leg, indexed by leg number
*/
static std::array<Vector3d, MAX_LEGS + 1> feetFor(const BodyPose& pose) {
    TrajectoryGenerator gen;
    GaitConfig config;
    std::array<Vector3d, MAX_LEGS + 1> feet;
    for (int leg = 1; leg <= MAX_LEGS; ++leg) {
        feet[leg] = gen.posedFoot(leg, config.startPosition.at(leg), pose);
    }
    return feet;
}

TEST(BodyPose, NeutralPoseLeavesTheFeetWhereTheyAre) {
    GaitConfig config;
    auto feet = feetFor(BodyPose{});
    for (int leg = 1; leg <= MAX_LEGS; ++leg) {
        EXPECT_NEAR((feet[leg] - config.startPosition.at(leg)).norm(), 0.0, 1e-12) << "leg " << leg;
    }
}

TEST(BodyPose, RaisingTheBodyLowersEveryFoot) {
    GaitConfig config;
    BodyPose pose;
    pose.translation = Vector3d(0.0, 0.0, 0.02);
    auto feet = feetFor(pose);
    for (int leg = 1; leg <= MAX_LEGS; ++leg) {
        EXPECT_NEAR(feet[leg].z(), config.startPosition.at(leg).z() - 0.02, 1e-12) << "leg " << leg;
    }
}

TEST(BodyPose, ShiftingForwardMovesTheFeetBackInTheBodyFrame) {
    TrajectoryGenerator gen;
    GaitConfig config;
    BodyPose pose;
    pose.translation = Vector3d(0.03, 0.0, 0.0);
    auto feet = feetFor(pose);
    for (int leg = 1; leg <= MAX_LEGS; ++leg) {
        Vector3d moved = gen.legToBody(leg, feet[leg]) - gen.legToBody(leg, config.startPosition.at(leg));
        EXPECT_NEAR((moved - Vector3d(-0.03, 0.0, 0.0)).norm(), 0.0, 1e-12) << "leg " << leg;
    }
}

TEST(BodyPose, PositiveRollLowersTheRightSide) {
    GaitConfig config;
    BodyPose pose;
    pose.roll = 0.1;
    auto feet = feetFor(pose);
    // The right hips drop towards the ground, so the right feet rise relative to them
    for (int leg = 1; leg <= 3; ++leg) {
        EXPECT_GT(feet[leg].z(), config.startPosition.at(leg).z() + 0.01) << "leg " << leg;
    }
    for (int leg = 4; leg <= 6; ++leg) {
        EXPECT_LT(feet[leg].z(), config.startPosition.at(leg).z() - 0.01) << "leg " << leg;
    }
}

TEST(BodyPose, PositivePitchLowersTheNose) {
    GaitConfig config;
    BodyPose pose;
    pose.pitch = 0.1;
    auto feet = feetFor(pose);
    for (int leg : {1, 6}) {
        EXPECT_GT(feet[leg].z(), config.startPosition.at(leg).z() + 0.01) << "leg " << leg;
    }
    for (int leg : {3, 4}) {
        EXPECT_LT(feet[leg].z(), config.startPosition.at(leg).z() - 0.01) << "leg " << leg;
    }
}

TEST(BodyPose, PositiveYawSweepsTheFeetClockwiseAroundTheBody) {
    TrajectoryGenerator gen;
    GaitConfig config;
    BodyPose pose;
    pose.yaw = 0.1;
    auto feet = feetFor(pose);
    for (int leg = 1; leg <= MAX_LEGS; ++leg) {
        Vector3d before = gen.legToBody(leg, config.startPosition.at(leg));
        Vector3d after = gen.legToBody(leg, feet[leg]);
        double turned = std::atan2(after.y(), after.x()) - std::atan2(before.y(), before.x());
        EXPECT_NEAR(std::remainder(turned, 2.0 * M_PI), -0.1, 1e-9) << "leg " << leg;
        EXPECT_NEAR(after.head<2>().norm(), before.head<2>().norm(), 1e-12) << "leg " << leg;
    }
}
