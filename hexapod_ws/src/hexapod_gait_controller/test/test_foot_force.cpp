#include <gtest/gtest.h>

#include "hexapod_gait_controller/kinematic_solver.hpp"

using hexapod_gait_controller::JointAngles;
using hexapod_gait_controller::KinematicSolver;
using Eigen::Vector3d;

TEST(FootForce, RecoversAForceFromTheTorquesItCauses) {
    KinematicSolver solver;
    JointAngles angles = solver.solveIK(Vector3d(0.15, 0.02, -0.12));
    const Vector3d force(3.0, -1.0, 7.5);  // N, ground on foot
    const Vector3d torques = -solver.jacobian(angles).transpose() * force;
    Vector3d estimated;
    ASSERT_TRUE(solver.groundForce(angles, torques, estimated));
    EXPECT_NEAR((estimated - force).norm(), 0.0, 1e-6);
}

TEST(FootForce, JacobianMatchesAFiniteStep) {
    KinematicSolver solver;
    JointAngles angles{0.2, -0.3, 0.4};
    const Vector3d step(0.001, -0.002, 0.0015);
    JointAngles moved{angles.coxa + step.x(), angles.femur + step.y(), angles.tibia + step.z()};
    const Vector3d predicted = solver.solveFK(angles) + solver.jacobian(angles) * step;
    EXPECT_NEAR((solver.solveFK(moved) - predicted).norm(), 0.0, 1e-5);
}

TEST(FootForce, StraightLegIsRejected) {
    KinematicSolver solver;
    // Femur and tibia in line: no vertical force can be found from the torques
    JointAngles straight{0.0, 0.0, -M_PI / 2.0};
    Vector3d force;
    EXPECT_FALSE(solver.groundForce(straight, Vector3d(0.1, 0.2, 0.3), force));
}
