#include "hexapod_gait_controller/trajectory_generator.hpp"

namespace hexapod_gait_controller {

/*
@brief Generates a straight trajectory between start and end positions
@param trajectory Pointer to an array to store the trajectory points
@param outSize Reference to an integer to store the number of points generated
@param start The starting position
@param end The ending position
@param resolution The number of points to generate
@return void
*/
void TrajectoryGenerator::GenStraightTrajectory(Vector3d* trajectory, int& outSize, const Vector3d& start, const Vector3d& end, int resolution) {
    outSize = 0;
    if (resolution <= 0 || resolution > 10000) {
        RCLCPP_ERROR(rclcpp::get_logger("TrajectoryGenerator"), "Invalid resolution: %d", resolution);
        return;
    }

    // If start and end are (almost) the same, return a flat trajectory
    if (std::abs(start.x() - end.x()) < 1e-6 &&
        std::abs(start.y() - end.y()) < 1e-6 &&
        std::abs(start.z() - end.z()) < 1e-6) {
        for (int i = 0; i <= resolution; ++i) {
            trajectory[i] = start;
        }
        outSize = resolution + 1;
        return;
    }

    for (int i = 0; i <= resolution; i++) {
        double t = static_cast<double>(i) / resolution;
        trajectory[i] = {
            start.x() + (end.x() - start.x()) * t,
            start.y() + (end.y() - start.y()) * t,
            start.z() + (end.z() - start.z()) * t
        };
    }
    outSize = resolution + 1;
}

/*
@brief Generates a Bezier curve trajectory between start and end positions
@param trajectory Pointer to an array to store the trajectory points
@param outSize Reference to an integer to store the number of points generated
@param start The starting position
@param end The ending position
@param liftHeight The height to lift the trajectory
@param resolution The number of points to generate
@param invert Whether to invert the trajectory
@return void
*/
void TrajectoryGenerator::GenBezierTrajectory(Vector3d* trajectory, int& outSize, const Vector3d& start, const Vector3d& end, int liftHeight, int resolution, bool invert) {
    outSize = 0;
    if (resolution <= 0 || resolution > 10000) {
        std::cerr << "Invalid resolution: " << resolution << std::endl;
        std::terminate();
    }

    Vector3d dir = end - start;
    if (dir.norm() == 0.0) {
        // Stationary case: generate flat path
        for (int i = 0; i <= resolution; ++i) {
            trajectory[i] = start;
        }
        outSize = resolution + 1;
        return;
    }

    Vector3d dirNorm = dir.normalized();
    double offsetScale = dir.norm() * 0.25; // tweak as needed

    // Place control points before start and after end along the movement direction
    Vector3d P0 = start;
    Vector3d P3 = end;
    Vector3d P1 = start - dirNorm * offsetScale;
    Vector3d P2 = end + dirNorm * offsetScale;

    P1.z() = start.z() + liftHeight;
    P2.z() = end.z() + liftHeight;

    for (int i = 0; i <= resolution; ++i) {
        double t = static_cast<double>(i) / resolution;
        double u = 1.0 - t;

        Vector3d point =
            P0 * (u * u * u) +
            P1 * (3 * u * u * t) +
            P2 * (3 * u * t * t) +
            P3 * (t * t * t);

        trajectory[i] = point;
    }
    outSize = resolution + 1;
}

/*
@brief Blend target positions from forward and rotation inputs
@param currentPos The current position of the leg
@param forwardPos The target position based on forward input
@param rotationPos The target position based on rotation input
@return The blended target position
*/
Vector3d TrajectoryGenerator::BlendTargetPosition(const Vector3d& currentPos, const Vector3d& forwardPos, const Vector3d& rotationPos) {
    // Compute deltas from current position
    Vector3d forwardDelta = forwardPos - currentPos;
    Vector3d rotationDelta = rotationPos - currentPos;

    // Add the deltas
    Vector3d blended = currentPos + forwardDelta + rotationDelta;

    // Optionally, clamp the stride to a maximum distance from currentPos if needed
    double maxStride = 100.0;
    if ((blended - currentPos).norm() > maxStride) {
         blended = currentPos + (blended - currentPos).normalized() * maxStride;
    }

    return blended;
}

/*
@brief Calculate stride multiplier based on gait configuration
@return The stride multiplier
*/
double TrajectoryGenerator::CalculateStrideMultiplier() {
    return (gaitState.config.size() > 1) ? 1.0 / (gaitState.config.size() - 1) : 1.0;
}

/*
@brief Ensure the gait configuration is set up correctly
*/
void TrajectoryGenerator::EnsureGaitConfig() {
    if (gaitState.config.empty()) {
        gaitState.config = GetLegConfig(GaitConfig::currentGait);
    }
}

}  // namespace hexapod_gait_controller