#include <gtest/gtest.h>

#include <cmath>

#include "hexapod_gait_controller/kinematic_solver.hpp"

using hexapod_gait_controller::JointAngles;
using hexapod_gait_controller::KinematicSolver;
using Eigen::Vector3d;

// Poses measured from the Hexapod_V6.1 CAD (Hexapod_V6.1_stand.step and _walk.step): the foot
// is the point 0.186 m from the knee axis along the line through the foot ball's centre,
// in the leg frame (+x out from the coxa axis, z from the hip plane)

static constexpr double DEG = M_PI / 180.0;

TEST(Kinematics, IkMatchesTheCadStandPose) {
    KinematicSolver solver;
    const Vector3d foot(0.17965, 0.0, -0.13353);
    const JointAngles angles = solver.solveIK(foot);
    EXPECT_NEAR(angles.coxa, 0.0, 1e-6);
    EXPECT_NEAR(angles.femur, -30.02 * DEG, 0.1 * DEG);
    EXPECT_NEAR(angles.tibia, 20.71 * DEG, 0.1 * DEG);
    EXPECT_NEAR((solver.solveFK(angles) - foot).norm(), 0.0, 1e-9);
}

TEST(Kinematics, IkMatchesTheCadWalkPose) {
    // A swinging leg: coxa turned 12 degrees, femur raised 45 degrees, foot lifted
    KinematicSolver solver;
    const Vector3d foot(0.24896, -0.05286, -0.07071);
    const JointAngles angles = solver.solveIK(foot);
    EXPECT_NEAR(angles.coxa, -12.0 * DEG, 0.1 * DEG);
    EXPECT_NEAR(angles.femur, -45.01 * DEG, 0.1 * DEG);
    EXPECT_NEAR(angles.tibia, 4.50 * DEG, 0.1 * DEG);
    EXPECT_NEAR((solver.solveFK(angles) - foot).norm(), 0.0, 1e-9);
}
