#include "hexapod_gait_controller/trajectory_generator.hpp"

namespace hexapod_gait_controller {

/**
 * @brief Generates a straight trajectory between start and end positions
 * @param[out] trajectory Pointer to an array to store the trajectory points
 * @param[out] outSize Reference to an integer to store the number of points generated
 * @param[in] start The starting position
 * @param[in] end The ending position
*/
void TrajectoryGenerator::GenStraightTrajectory(Vector3d* trajectory, int& outSize, const Vector3d& start, const Vector3d& end) {
    outSize = 0;

    // If start and end are (almost) the same, return a flat trajectory
    if (std::abs(start.x() - end.x()) < 1e-6 &&
        std::abs(start.y() - end.y()) < 1e-6 &&
        std::abs(start.z() - end.z()) < 1e-6) {
        for (int i = 0; i <= MAX_RESOLUTION; ++i) {
            trajectory[i] = start;
        }
        outSize = MAX_RESOLUTION + 1;
        return;
    }

    for (int i = 0; i <= MAX_RESOLUTION; i++) {
        double t = static_cast<double>(i) / MAX_RESOLUTION;
        trajectory[i] = {
            start.x() + (end.x() - start.x()) * t,
            start.y() + (end.y() - start.y()) * t,
            start.z() + (end.z() - start.z()) * t
        };
    }
    outSize = MAX_RESOLUTION + 1;
}

/**
 * @brief Generates a Bezier curve trajectory between start and end positions
 * @param[out] trajectory Pointer to an array to store the trajectory points
 * @param[out] outSize Reference to an integer to store the number of points generated
 * @param[in] start The starting position
 * @param[in] end The ending position
*/
void TrajectoryGenerator::GenBezierTrajectory(Vector3d* trajectory, int& outSize, const Vector3d& start, const Vector3d& end) {
    outSize = 0;

    Vector3d dir = end - start;
    if (dir.norm() == 0.0) {
        // Stationary case: generate flat path
        for (int i = 0; i <= MAX_RESOLUTION; ++i) {
            trajectory[i] = start;
        }
        outSize = MAX_RESOLUTION + 1;
        return;
    }

    // Place control points before start and after end along the movement direction
    Vector3d P0 = start;
    Vector3d P4 = end;

    Vector3d P1 = start;
    P1.z() += LIFT_HEIGHT;

    Vector3d P2 = start + dir * 0.60;
    P2.z() += LIFT_HEIGHT * 0.5;

    Vector3d P3 = end;
    P3.z() += LIFT_HEIGHT * 1.5;

    for (int i = 0; i <= MAX_RESOLUTION; ++i) {
        double t = static_cast<double>(i) / MAX_RESOLUTION;
        double st = t * t * t * (t * (6.0 *  t - 15.0) + 10.0);
        double u = 1.0 - st;

        Vector3d point =
            P0 * (u * u * u * u) +
            P1 * (4.0 * u * u * u * st) +
            P2 * (6.0 * u * u * st * st) +
            P3 * (4.0 * u * st * st * st) +
            P4 * (st * st * st * st);

        trajectory[i] = point;
    }
    outSize = MAX_RESOLUTION + 1;
}

/**
 * @brief Ensure the gait configuration is set up correctly
*/
void TrajectoryGenerator::EnsureGaitConfig() {
    if (gaitState.config.empty()) {
        gaitState.config = gaitConfig.getGaitConfig(gaitConfig.currentGait);
    }
}

/**
 * @brief Generate swing and stance trajectories for the legs
 * @param[in] swingTargetFunc A function to compute the swing target position for a leg
 * @param[in] stanceTargetFunc A function to compute the stance target position for a leg
 * @param[in] currentPositions The current position of each leg
 * @param[in] currentPhase The gait phase whose legs swing
*/
void TrajectoryGenerator::GenerateTrajectories(
    std::function<Vector3d(int, const Vector3d&)> swingTargetFunc,
    std::function<Vector3d(int, const Vector3d&)> stanceTargetFunc,
    std::array<Vector3d, MAX_LEGS + 1> currentPositions,
    uint8_t currentPhase
) {
    // Assign legs to their respective swing and stance groups
    auto swingGroup = gaitState.config[currentPhase];
    std::vector<int> stanceGroup;
    for (size_t idx = 0; idx < gaitState.config.size(); ++idx) {
        if (idx == currentPhase) continue;
        for (int legNum : gaitState.config[idx])
            stanceGroup.push_back(legNum);
    }
    for (int i = 1; i <= MAX_LEGS; ++i) {
        gaitState.stanceSplines[i].active = false;
    }

    std::map<int, Vector3d> swingTargets;
    std::map<int, Vector3d> stanceTargets;

    // Calculate swing and stance targets in body frame
    for (int legNum : swingGroup) {
        Vector3d currentPos = currentPositions[legNum];
        swingTargets[legNum] = swingTargetFunc(legNum, currentPos);
    }
    for (int legNum : stanceGroup) {
        Vector3d currentPos = currentPositions[legNum];
        stanceTargets[legNum] = stanceTargetFunc(legNum, currentPos);
    }

    // Clear all swing splines first
    for (int i = 1; i <= MAX_LEGS; ++i) {
        gaitState.swingSplines[i].active = false;
    }

    // Setup swing splines
    for (int legNum : swingGroup) {
        SwingSpline& spline = gaitState.swingSplines[legNum];
        Vector3d start = currentPositions[legNum];
        Vector3d end = swingTargets[legNum];
        Vector3d dir = end - start;

        if (dir.norm() < 1e-6) {
            spline.s = 0.0;
            spline.P0 = start;
            spline.P1 = start;
            spline.P2 = start;
            spline.P3 = start;
            spline.P4 = end;
            spline.active = false;
            continue;
        }

        genSwingSpline(spline, start, end);
    }

    // Stance
    for (int legNum : stanceGroup) {
        LineSpline& spline = gaitState.stanceSplines[legNum];
        Vector3d targetLegFrame = stanceTargets[legNum];
        spline.P0 = currentPositions[legNum];
        spline.P1 = targetLegFrame;
        spline.s = 0.0;
        spline.active = true;
    }
}

/**
 * @brief Yaw of a leg's frame relative to the body frame
 * @param[in] legNum The leg number
 * @return The yaw (rad), matching the coxa joint origins in the URDF
*/
double TrajectoryGenerator::legYaw(int legNum) {
    return converter.legConfigs.at(legNum).yaw;
}

/**
 * @brief Convert a position in a leg's frame to the body frame
 * @param[in] legNum The leg number
 * @param[in] position The position in the leg frame
 * @return The position in the body frame
*/
Vector3d TrajectoryGenerator::legToBody(int legNum, const Vector3d& position) {
    return converter.legToBodyFrame(position, legNum);
}

/**
 * @brief Position of a leg's neutral foot in the body frame
 * @param[in] legNum The leg number
 * @return The foot position relative to the body centre (body frame, z = 0)
*/
Vector3d TrajectoryGenerator::neutralFootInBody(int legNum) {
    const Vector3d& neutral = gaitConfig.startPosition.at(legNum);
    Vector3d foot = converter.legToBodyFrame(Vector3d(neutral.x(), neutral.y(), 0.0), legNum);
    foot.z() = 0.0;
    return foot;
}

/**
 * @brief Foot displacement in a leg's frame for a body velocity held for a duration
 * @param[in] forward Forward body velocity (m/s)
 * @param[in] left Leftward body velocity (m/s)
 * @param[in] yaw Body yaw rate (rad/s, positive anticlockwise)
 * @param[in] legNum The leg number
 * @param[in] duration How long the velocity is held (s)
 * @return The displacement (leg frame, z = 0), in the direction the foot swings
 * @note Worked out in the body frame (+x forward, +y left), then rotated into the leg frame.
 *       Yaw rotates the neutral foot about the body centre. A stance foot moves by the negative of this.
*/
Vector3d TrajectoryGenerator::footDisplacement(double forward, double left, double yaw, int legNum, double duration) {
    Vector3d foot = neutralFootInBody(legNum);
    Vector3d linear(forward * duration, left * duration, 0.0);
    Vector3d rotation = Eigen::AngleAxisd(yaw * duration, Vector3d::UnitZ()) * foot - foot;
    return Eigen::AngleAxisd(-legYaw(legNum), Vector3d::UnitZ()) * (linear + rotation);
}

/**
 * @brief Distance from the body centre to a leg's neutral foot position
 * @param[in] legNum The leg number
 * @return The horizontal distance (m)
*/
double TrajectoryGenerator::bodyRadius(int legNum) {
    return neutralFootInBody(legNum).norm();
}

/**
 * @brief Largest bodyRadius of any leg
 * @return The distance (m)
*/
double TrajectoryGenerator::maxBodyRadius() {
    double radius = 0.0;
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        radius = std::max(radius, bodyRadius(legNum));
    }
    return radius;
}

/**
 * @brief Where a swinging foot should land
 * @param[in] legNum The leg number
 * @param[in] displacement The foot displacement ahead of neutral (leg frame)
 * @return The target foot position (leg frame), at most MAX_STRIDE_LENGTH from neutral
*/
Vector3d TrajectoryGenerator::swingTarget(int legNum, const Vector3d& displacement) {
    return clampToStride(legNum, gaitConfig.startPosition.at(legNum) + displacement);
}

/**
 * @brief Where a stance foot should be at the end of the phase
 * @param[in] legNum The leg number
 * @param[in] currentPos The foot's current position (leg frame)
 * @param[in] displacement The body's movement over the phase, as a foot displacement (leg frame)
 * @return The target foot position (leg frame), at most MAX_STRIDE_LENGTH from neutral
*/
Vector3d TrajectoryGenerator::stanceTarget(int legNum, const Vector3d& currentPos, const Vector3d& displacement) {
    return clampToStride(legNum, currentPos - displacement);
}

/**
 * @brief Keep a foot target within MAX_STRIDE_LENGTH of neutral and on the ground
 * @param[in] legNum The leg number
 * @param[in] target The foot target (leg frame)
 * @return The clamped target, with z at the current body height
*/
Vector3d TrajectoryGenerator::clampToStride(int legNum, const Vector3d& target) {
    const Vector3d& neutralPos = gaitConfig.startPosition.at(legNum);
    double dx = target.x() - neutralPos.x();
    double dy = target.y() - neutralPos.y();
    double distance = std::hypot(dx, dy);
    if (distance > MAX_STRIDE_LENGTH) {
        dx *= MAX_STRIDE_LENGTH / distance;
        dy *= MAX_STRIDE_LENGTH / distance;
    }
    return {neutralPos.x() + dx, neutralPos.y() + dy, gaitConfig.currentHeight};
}

/**
 * @brief Set up the control points of a swing-phase Bezier spline
 * @param[out] spline The spline to set up
 * @param[in] start The starting position
 * @param[in] end The ending position
 * @note Resets the spline progress and marks the spline active
*/
void TrajectoryGenerator::genSwingSpline(SwingSpline& spline, const Vector3d& start, const Vector3d& end){
    Eigen::Vector3d dir = end - start;

    spline.P0 = start;
    spline.P4 = end;

    spline.P1 = spline.P0;
    spline.P1.z() += LIFT_HEIGHT;

    spline.P2 = spline.P0 + dir * 0.60;
    spline.P2.z() += LIFT_HEIGHT * 0.5;

    spline.P3 = spline.P4;
    spline.P3.z() += LIFT_HEIGHT * 1.5;

    spline.s = 0.0;
    spline.active = true;
}

/**
 * @brief Evaluate a quartic Bezier curve with smoothstep easing
 * @param[in] P0 The start point
 * @param[in] P1 The first control point
 * @param[in] P2 The second control point
 * @param[in] P3 The third control point
 * @param[in] P4 The end point
 * @param[in] s Normalised progress along the curve [0, 1]
 * @return The point on the curve
*/
Vector3d TrajectoryGenerator::evalBezier(const Vector3d& P0, const Vector3d& P1, const Vector3d& P2, const Vector3d& P3, const Vector3d& P4, const double s) {
    double st = s * s * s * (s * (6.0 * s - 15.0) + 10.0);
    double u = 1.0 - st;

    Eigen::Vector3d point =
        P0 * (u * u * u * u) +
        P1 * (4.0 * u * u * u * st) +
        P2 * (6.0 * u * u * st * st) +
        P3 * (4.0 * u * st * st * st) +
        P4 * (st * st * st * st);
    
    return point;
}

}  // namespace hexapod_gait_controller