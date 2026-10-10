#ifndef HEXAPOD_GAIT_CONTROLLER__LEG_ODOMETRY_HPP
#define HEXAPOD_GAIT_CONTROLLER__LEG_ODOMETRY_HPP

#include <vector>

#include <Eigen/Dense>

namespace hexapod_gait_controller {

using Eigen::Vector2d;

/**
 * @brief Body motion over one update, in the body frame at the start of the update
*/
struct BodyMotion {
    double dx;     // m, forward
    double dy;     // m, left
    double dyaw;   // rad, anticlockwise
};

/**
 * @brief Estimates body motion from planted feet and integrates it into a 2D pose
 * @note Feet on the ground are assumed not to slip, so the body motion is the rigid
 *       transform that best explains how those feet moved relative to the body.
*/
class LegOdometry {
public:
    static bool estimateMotion(
        const std::vector<Vector2d>& before,
        const std::vector<Vector2d>& after,
        BodyMotion& motion
    );
    void integrate(const BodyMotion& motion);
    void reset();

    double x() const { return x_; }
    double y() const { return y_; }
    double yaw() const { return yaw_; }

private:
    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;
};

}  // namespace hexapod_gait_controller

#endif  // HEXAPOD_GAIT_CONTROLLER__LEG_ODOMETRY_HPP
