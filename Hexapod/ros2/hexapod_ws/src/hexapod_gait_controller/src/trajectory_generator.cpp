#include "hexapod_gait_controller/trajectory_generator.hpp"

namespace hexapod_gait_controller {

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

}  // namespace hexapod_gait_controller