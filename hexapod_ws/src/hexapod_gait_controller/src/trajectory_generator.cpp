#include "hexapod_gait_controller/trajectory_generator.hpp"

namespace hexapod_gait_controller {

/*
@brief Generates a straight trajectory between start and end positions
@param trajectory Pointer to an array to store the trajectory points
@param outSize Reference to an integer to store the number of points generated
@param start The starting position
@param end The ending position
@return void
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

/*
@brief Generates a Bezier curve trajectory between start and end positions
@param trajectory Pointer to an array to store the trajectory points
@param outSize Reference to an integer to store the number of points generated
@param start The starting position
@param end The ending position
@param invert Whether to invert the trajectory
@return void
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

/*
@brief Blend target positions from forward and rotation inputs
@param currentPos The current position of the leg (used for reference)
@param forwardPos The target position based on forward input (relative to neutral)
@param rotationPos The target position based on rotation input (relative to neutral)
@return The blended target position
@note Both forwardPos and rotationPos are computed relative to the leg's neutral position.
      We blend them by averaging their displacements from neutral.
*/
Vector3d TrajectoryGenerator::BlendTargetPosition(const Vector3d& currentPos, const Vector3d& forwardPos, const Vector3d& rotationPos, int legNum, double strideMultiplier) {
    auto it = gaitConfig.startPosition.find(legNum);
    Vector3d neutralPos = (it != gaitConfig.startPosition.end()) ? it->second : currentPos;

    const bool incremental = strideMultiplier < 1.0;
    const Vector3d anchorPos = incremental ? currentPos : neutralPos;

    // Convert absolute targets → deltas from neutral
    Vector3d forwardDelta  = forwardPos  - anchorPos;
    Vector3d rotationDelta = rotationPos - anchorPos;

    // Blend deltas
    Vector3d blendedDelta = forwardDelta + rotationDelta;

    // Clamp blended delta using stride multiplier
    double maxStride = MAX_STRIDE_LENGTH* strideMultiplier;
    double deltaMag = std::hypot(blendedDelta.x(), blendedDelta.y());

    if (deltaMag > maxStride && deltaMag > 1e-6) {
        blendedDelta *= (maxStride / deltaMag);
    }

    return {
        anchorPos.x() + blendedDelta.x(),
        anchorPos.y() + blendedDelta.y(),
        gaitConfig.currentHeight
    };
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
        gaitState.config = gaitConfig.getGaitConfig(gaitConfig.currentGait);
    }
}

/*
@brief Generate swing and stance trajectories for the legs
@param swingTargetFunc A function to compute the swing target position for a leg
@param stanceTargetFunc A function to compute the stance target position for a leg
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

/*
@brief Calculate linear target position based on input velocities
@param linearX The forward linear velocity
@param linearY The lateral linear velocity
@param start The starting position (used for reference)
@param legNum The leg number
@param invert Whether to invert the direction (for stance phase)
@param strideMultiplier Multiplier for stride length
@return The calculated target position
@note Linear offset is computed relative to the leg's neutral position to prevent drift.
*/
Vector3d TrajectoryGenerator::linearTarget(const double& linearX, const double& linearY, int legNum, bool invert) {
    double lx = linearX;
    double ly = linearY;
    std::swap(lx, ly);

    // Apply inversion if needed (for stance phase)
    if (invert) {ly = -ly;}

    // Apply leg mirroring
    if (converter.legConfigs[legNum].mirrored) {ly = -ly;}

    double magnitude = std::hypot(lx, ly) / MAX_LINEAR_VELOCITY;
    if (magnitude > 1.0) magnitude = 1.0;

    double stride = MAX_STRIDE_LENGTH * magnitude;
    double angle = atan2(ly, lx);

    double deltaX = stride * cos(angle);
    double deltaY = stride * sin(angle);
    double groundZ = gaitConfig.currentHeight;

    double legAngle = converter.legConfigs[legNum].mounting_angle;
    if (!converter.legConfigs[legNum].mirrored) {legAngle = -legAngle;}

    double dx_rot = deltaX * cos(legAngle) - deltaY * sin(legAngle);
    double dy_rot = deltaX * sin(legAngle) + deltaY * cos(legAngle);

    // Get the neutral/start position for this leg and offset from there
    auto it = gaitConfig.startPosition.find(legNum);
    Vector3d neutralPos = gaitConfig.startPosition.at(legNum);
    
    return {neutralPos.x() + dx_rot, neutralPos.y() + dy_rot, groundZ};
}

/*
@brief Calculate rotational target position based on input angular velocity
@param angularZ The angular velocity around the Z-axis
@param start The starting position (used only for Z reference)
@param legNum The leg number
@param invert Whether to invert the direction (for stance phase)
@param strideMultiplier Multiplier for stride length
@return The calculated target position
@note Rotation is computed relative to the leg's neutral position, not current position.
      This prevents legs from drifting outside their workspace during sustained rotation.
*/
Vector3d TrajectoryGenerator::rotationalTarget(const double& angularZ, int legNum, bool invert) {
    double az = angularZ;

    // Apply inversion if needed (for stance phase)
    if (invert) {az = -az;}

    double magnitude = std::abs(az) / MAX_ANGULAR_VELOCITY;
    if (magnitude > 1.0) magnitude = 1.0;

    // Get the neutral/start position for this leg
    auto it = gaitConfig.startPosition.find(legNum);
    Vector3d neutralPos = gaitConfig.startPosition.at(legNum);
    
    // Calculate rotational offset as arc displacement from neutral
    // For small angles, arc length ≈ radius * angle
    // We use the neutral leg reach as the radius
    double neutralReach = std::hypot(neutralPos.x(), neutralPos.y());
    double maxArcLength = MAX_STRIDE_LENGTH;
    double arcOffset = maxArcLength * magnitude * (az > 0 ? 1.0 : -1.0);
    
    // Convert arc offset to angular displacement (radians)
    double angularOffset = (neutralReach > 0.01) ? (arcOffset / neutralReach) : 0.0;
    
    // Rotate the neutral position by this angle
    double cosA = cos(angularOffset);
    double sinA = sin(angularOffset);
    
    double newX = neutralPos.x() * cosA - neutralPos.y() * sinA;
    double newY = neutralPos.x() * sinA + neutralPos.y() * cosA;
    double groundZ = gaitConfig.currentHeight;

    return {newX, newY, groundZ};
}

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