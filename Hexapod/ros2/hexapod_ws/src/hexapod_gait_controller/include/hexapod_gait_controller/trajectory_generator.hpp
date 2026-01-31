#ifndef HEXAPOD_GAIT_CONTROLLER__TRAJECTORY_GENERATOR_HPP
#define HEXAPOD_GAIT_CONTROLLER__TRAJECTORY_GENERATOR_HPP

#include "hexapod_gait_controller/kinematic_solver.hpp"

namespace hexapod_gait_controller {

using Eigen::Vector3d;

class TrajectoryGenerator {
public:
    void GenStraightTrajectory(Vector3d* trajectory, int& outSize, const Vector3d& start, const Vector3d& end, int resolution);
    void GenBezierTrajectory(Vector3d* trajectory, int& outSize, const Vector3d& start, const Vector3d& end, int liftHeight, int resolution, bool invert);
    Vector3d BlendTargetPosition(const Vector3d& currentPos, const Vector3d& targetPos, const Vector3d& rotationPos);
};

}  // namespace hexapod_gait_controller

#endif // HEXAPOD_GAIT_CONTROLLER__TRAJECTORY_GENERATOR_HPP