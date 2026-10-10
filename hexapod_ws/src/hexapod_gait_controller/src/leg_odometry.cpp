#include "hexapod_gait_controller/leg_odometry.hpp"

#include <cmath>

namespace hexapod_gait_controller {

/**
 * @brief Find the body motion that best explains how planted feet moved relative to the body
 * @param[in] before Planted foot positions at the start of the update (body frame)
 * @param[in] after The same feet at the end of the update (body frame), in the same order
 * @param[out] motion The body's motion, in the body frame at the start of the update
 * @return true if there were enough feet (at least 2) to estimate the motion, false otherwise
 * @note A least-squares rigid fit: the feet are fixed in the world, so after = R(-dyaw) * (before - t).
*/
bool LegOdometry::estimateMotion(
    const std::vector<Vector2d>& before,
    const std::vector<Vector2d>& after,
    BodyMotion& motion
) {
    if (before.size() < 2 || before.size() != after.size()) {
        return false;
    }

    Vector2d meanBefore = Vector2d::Zero();
    Vector2d meanAfter = Vector2d::Zero();
    for (size_t i = 0; i < before.size(); ++i) {
        meanBefore += before[i];
        meanAfter += after[i];
    }
    meanBefore /= static_cast<double>(before.size());
    meanAfter /= static_cast<double>(after.size());

    // Rotation that best maps the feet before onto the feet after
    double sumCos = 0.0;
    double sumSin = 0.0;
    for (size_t i = 0; i < before.size(); ++i) {
        Vector2d a = before[i] - meanBefore;
        Vector2d b = after[i] - meanAfter;
        sumCos += a.x() * b.x() + a.y() * b.y();
        sumSin += a.x() * b.y() - a.y() * b.x();
    }
    double footRotation = std::atan2(sumSin, sumCos);

    // The feet appear to rotate the opposite way to the body
    motion.dyaw = -footRotation;
    Eigen::Rotation2Dd bodyRotation(motion.dyaw);
    Vector2d translation = meanBefore - bodyRotation * meanAfter;
    motion.dx = translation.x();
    motion.dy = translation.y();
    return true;
}

/**
 * @brief Add one update's body motion to the pose
 * @param[in] motion The body's motion, in the body frame at the start of the update
*/
void LegOdometry::integrate(const BodyMotion& motion) {
    x_ += motion.dx * std::cos(yaw_) - motion.dy * std::sin(yaw_);
    y_ += motion.dx * std::sin(yaw_) + motion.dy * std::cos(yaw_);
    yaw_ = std::remainder(yaw_ + motion.dyaw, 2.0 * M_PI);
}

/**
 * @brief Put the pose back at the origin
*/
void LegOdometry::reset() {
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
}

}  // namespace hexapod_gait_controller
