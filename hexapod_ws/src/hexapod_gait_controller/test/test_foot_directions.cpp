#include <gtest/gtest.h>

#include <cmath>

#include "hexapod_gait_controller/trajectory_generator.hpp"

using hexapod_gait_controller::MAX_LEGS;
using hexapod_gait_controller::TrajectoryGenerator;
using Eigen::Vector3d;

// Body frame: +y forward (towards the cameras), +x right, +z up
static const Vector3d FORWARD(0.0, 1.0, 0.0);
static const Vector3d RIGHT(1.0, 0.0, 0.0);

/**
 * @brief Rotate a leg-frame displacement into the body frame
 * @param[in] gen The trajectory generator that owns the leg frames
 * @param[in] legNum The leg number
 * @param[in] legVector The displacement in the leg frame
 * @return The displacement in the body frame
*/
static Vector3d toBody(TrajectoryGenerator& gen, int legNum, const Vector3d& legVector) {
    return Eigen::AngleAxisd(gen.legYaw(legNum), Vector3d::UnitZ()) * legVector;
}

/**
 * @brief Difference between two angles, wrapped to [-pi, pi]
 * @param[in] a The first angle (rad)
 * @param[in] b The second angle (rad)
 * @return a - b, wrapped (rad)
*/
static double angleDiff(double a, double b) {
    return std::remainder(a - b, 2.0 * M_PI);
}

TEST(FootDirections, LegFramesMatchUrdf) {
    TrajectoryGenerator gen;
    // Coxa joint yaws from hexapod_primitives.urdf.xacro
    const double urdf_yaw[MAX_LEGS + 1] = {0.0, 0.5236, 0.0, -0.5236, -2.61799, -3.14159, 2.61799};
    for (int leg = 1; leg <= MAX_LEGS; ++leg) {
        EXPECT_NEAR(angleDiff(gen.legYaw(leg), urdf_yaw[leg]), 0.0, 1e-3) << "leg " << leg;
    }
}

TEST(FootDirections, NeutralFeetPointAwayFromBody) {
    TrajectoryGenerator gen;
    for (int leg = 1; leg <= MAX_LEGS; ++leg) {
        Vector3d foot = gen.neutralFootInBody(leg);
        EXPECT_GT(foot.norm(), 0.2) << "leg " << leg;
        EXPECT_LT(foot.norm(), 0.3) << "leg " << leg;
    }
}

TEST(FootDirections, ForwardMovesEveryFootForward) {
    TrajectoryGenerator gen;
    for (int leg = 1; leg <= MAX_LEGS; ++leg) {
        Vector3d body = toBody(gen, leg, gen.footDisplacement(0.1, 0.0, 0.0, leg, 1.0));
        EXPECT_NEAR((body - 0.1 * FORWARD).norm(), 0.0, 1e-9) << "leg " << leg;
    }
}

TEST(FootDirections, PositiveLateralMovesEveryFootRight) {
    TrajectoryGenerator gen;
    for (int leg = 1; leg <= MAX_LEGS; ++leg) {
        Vector3d body = toBody(gen, leg, gen.footDisplacement(0.0, 0.1, 0.0, leg, 1.0));
        EXPECT_NEAR((body - 0.1 * RIGHT).norm(), 0.0, 1e-9) << "leg " << leg;
    }
}

TEST(FootDirections, PositiveYawIsAnticlockwiseAboutBodyCentre) {
    TrajectoryGenerator gen;
    const double yaw = 0.5;       // rad/s
    const double duration = 0.01; // s, short so the arc is close to a straight line
    for (int leg = 1; leg <= MAX_LEGS; ++leg) {
        Vector3d foot = gen.neutralFootInBody(leg);
        Vector3d body = toBody(gen, leg, gen.footDisplacement(0.0, 0.0, yaw, leg, duration));
        // Tangent to the circle about the body centre, anticlockwise, with arc length r * yaw * t
        EXPECT_NEAR(body.normalized().dot(foot.normalized()), 0.0, 0.01) << "leg " << leg;
        EXPECT_GT(foot.cross(body).z(), 0.0) << "leg " << leg;
        EXPECT_NEAR(body.norm(), foot.norm() * yaw * duration, 1e-6) << "leg " << leg;
    }
}
