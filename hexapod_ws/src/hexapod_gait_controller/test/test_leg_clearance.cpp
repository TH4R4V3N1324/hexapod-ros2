#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <optional>
#include <vector>

#include "hexapod_gait_controller/kinematic_solver.hpp"
#include "hexapod_gait_controller/trajectory_generator.hpp"

using hexapod_gait_controller::JointAngles;
using hexapod_gait_controller::KinematicSolver;
using hexapod_gait_controller::MAX_STRIDE_LENGTH;
using hexapod_gait_controller::TrajectoryGenerator;
using Eigen::Vector3d;

// Link lengths used by KinematicSolver (m)
static constexpr double COXA = 0.0505;
static constexpr double FEMUR = 0.090;
static constexpr double TIBIA = 0.15035;

// Half-thickness of each segment in the Gazebo model (m)
static constexpr double COXA_RADIUS = 0.03;
static constexpr double FEMUR_RADIUS = 0.029;
static constexpr double TIBIA_RADIUS = 0.0225;

// Smallest gap allowed between neighbouring legs' segments (m)
static constexpr double MIN_CLEARANCE = 0.01;

/**
 * @brief Joint positions of one leg in its own frame
 * @param[in] angles The leg's joint angles
 * @return Coxa joint, femur joint, knee and foot (leg frame)
*/
static std::array<Vector3d, 4> legChain(const JointAngles& angles) {
    double femurElevation = -angles.femur;
    double tibiaElevation = femurElevation - angles.tibia - M_PI / 2.0;
    Vector3d dir(std::cos(angles.coxa), std::sin(angles.coxa), 0.0);
    Vector3d femurJoint = dir * COXA;
    Vector3d knee = femurJoint + dir * FEMUR * std::cos(femurElevation) + Vector3d(0, 0, FEMUR * std::sin(femurElevation));
    Vector3d foot = knee + dir * TIBIA * std::cos(tibiaElevation) + Vector3d(0, 0, TIBIA * std::sin(tibiaElevation));
    return {Vector3d::Zero(), femurJoint, knee, foot};
}

/**
 * @brief Shortest distance between two line segments
 * @param[in] p0 Start of the first segment
 * @param[in] p1 End of the first segment
 * @param[in] q0 Start of the second segment
 * @param[in] q1 End of the second segment
 * @return The distance (m), accurate to about a millimetre
*/
static double segmentDistance(const Vector3d& p0, const Vector3d& p1, const Vector3d& q0, const Vector3d& q1) {
    double best = 1e9;
    for (int i = 0; i <= 20; ++i) {
        Vector3d a = p0 + (p1 - p0) * (i / 20.0);
        for (int j = 0; j <= 20; ++j) {
            best = std::min(best, (a - (q0 + (q1 - q0) * (j / 20.0))).norm());
        }
    }
    return best;
}

TEST(LegClearance, ChainMatchesForwardKinematics) {
    KinematicSolver solver;
    JointAngles angles{0.3, -0.4, 0.2};
    EXPECT_NEAR((legChain(angles)[3] - solver.solveFK(angles)).norm(), 0.0, 1e-9);
}

TEST(LegClearance, NeighbouringLegsNeverTouchWithinTheirStrideCircles) {
    TrajectoryGenerator gen;
    KinematicSolver solver;
    const double radii[3] = {COXA_RADIUS, FEMUR_RADIUS, TIBIA_RADIUS};

    // Foot offsets across the stride circle, at heights from walking (standing and mid-swing) and
    // from tilt mode, where a tilted or shifted body puts feet higher or lower than standing
    // Walk leveling can put a foot up to 7 mm beyond its stride circle (see test_body_pose)
    constexpr double LEVELING_SHIFT = 0.007;  // m
    std::vector<Vector3d> offsets = {Vector3d::Zero()};
    for (int k = 0; k < 16; ++k) {
        double a = 2.0 * M_PI * k / 16.0;
        double r = MAX_STRIDE_LENGTH + LEVELING_SHIFT;
        offsets.emplace_back(r * std::cos(a), r * std::sin(a), 0.0);
    }
    const std::vector<double> heights = {-0.10, -0.05, 0.0, 0.05, 0.10};

    // Body-frame joint positions of a leg with its foot at neutral + offset (leg frame), lifted by
    // lift. Empty if the foot is out of reach, since the gait controller never sends it there.
    auto segmentsFor = [&](int leg, const Vector3d& offset, double lift) -> std::optional<std::array<Vector3d, 4>> {
        Vector3d foot = gen.swingTarget(leg, offset);   // neutral + offset, on the ground
        foot.z() += lift;
        JointAngles angles = solver.solveIK(foot);
        if ((solver.solveFK(angles) - foot).norm() > 1e-3) {
            return std::nullopt;
        }
        auto chain = legChain(angles);
        std::array<Vector3d, 4> body;
        for (int i = 0; i < 4; ++i) {
            body[i] = gen.legToBody(leg, chain[i]);
        }
        return body;
    };

    double worst = 1e9;
    for (auto [a, b] : std::vector<std::pair<int, int>>{{1, 2}, {2, 3}, {4, 5}, {5, 6}}) {
        for (const auto& offsetA : offsets) {
            for (double liftA : heights) {
                auto legA = segmentsFor(a, offsetA, liftA);
                if (!legA) continue;
                for (const auto& offsetB : offsets) {
                    for (double liftB : heights) {
                        auto legB = segmentsFor(b, offsetB, liftB);
                        if (!legB) continue;
                        for (int i = 0; i < 3; ++i) {
                            for (int j = 0; j < 3; ++j) {
                                double gap = segmentDistance((*legA)[i], (*legA)[i + 1], (*legB)[j], (*legB)[j + 1]) - radii[i] - radii[j];
                                worst = std::min(worst, gap);
                            }
                        }
                    }
                }
            }
        }
    }
    EXPECT_GT(worst, MIN_CLEARANCE) << "smallest gap between neighbouring legs: " << worst * 100.0 << " cm";
}
