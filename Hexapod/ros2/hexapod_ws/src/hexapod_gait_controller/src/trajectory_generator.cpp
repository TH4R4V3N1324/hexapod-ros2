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
void TrajectoryGenerator::GenBezierTrajectory(Vector3d* trajectory, int& outSize, const Vector3d& start, const Vector3d& end, double liftHeight, int resolution) {
    outSize = 0;
    if (resolution <= 0 || resolution > 10000) {
        RCLCPP_ERROR(rclcpp::get_logger("TrajectoryGenerator"), "Invalid resolution: %d", resolution);
        return;
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
    double maxStride = gaitConfig.max_stride_length * strideMultiplier;
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
@param liftHeight The height to lift the legs during swing
@param resolution The number of steps in the trajectory
@param swingTargetFunc A function to compute the swing target position for a leg
@param stanceTargetFunc A function to compute the stance target position for a leg
*/
void TrajectoryGenerator::GenerateTrajectories(
    double liftHeight,
    int resolution,
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
        gaitState.swingSizes[i] = 0;
        gaitState.stanceSizes[i] = 0;
    }

    std::map<int, Vector3d> swingTargetsBodyFrame;
    std::map<int, Vector3d> stanceTargetsBodyFrame;

    // Calculate swing and stance targets in body frame
    for (int legNum : swingGroup) {
        Vector3d currentPos = currentPositions[legNum];
        Vector3d targetLegFrame = swingTargetFunc(legNum, currentPos);
        Vector3d targetBodyFrame = converter.legToBodyFrame(targetLegFrame, legNum);
        swingTargetsBodyFrame[legNum] = targetBodyFrame;
    }
    for (int legNum : stanceGroup) {
        Vector3d currentPos = currentPositions[legNum];
        Vector3d targetLegFrame = stanceTargetFunc(legNum, currentPos);
        Vector3d targetBodyFrame = converter.legToBodyFrame(targetLegFrame, legNum);
        stanceTargetsBodyFrame[legNum] = targetBodyFrame;
    }

    // Clear all swing splines first
    for (int i = 1; i <= MAX_LEGS; ++i) {
        gaitState.swingSplines[i].active = false;
    }

    // Setup swing splines
    for (int legNum : swingGroup) {
        SwingSpline& spline = gaitState.swingSplines[legNum];

        spline.P0 = currentPositions[legNum];
        spline.P3 = converter.bodyToLegFrame(
            swingTargetsBodyFrame[legNum], legNum
        );

        Eigen::Vector3d dir = spline.P3 - spline.P0;
        if (dir.norm() < 1e-6) {
            dir = Eigen::Vector3d(1, 0, 0);
        }

        Eigen::Vector3d dirNorm = dir.normalized();
        double offsetScale = dir.norm() * 0.25;

        spline.P1 = spline.P0 - dirNorm * offsetScale;
        spline.P2 = spline.P3 + dirNorm * offsetScale;

        spline.P1.z() = spline.P0.z() + liftHeight;
        spline.P2.z() = spline.P3.z() + liftHeight;

        spline.s = 0.0;
        spline.active = true;
    }

    // Stance
    for (int legNum : stanceGroup) {
        Vector3d targetLegFrame = converter.bodyToLegFrame(stanceTargetsBodyFrame[legNum], legNum);
        int size = 0;
        GenStraightTrajectory(
            gaitState.stanceTrajectory[legNum].data(),
            size,
            currentPositions[legNum],
            targetLegFrame,
            resolution
        );
        gaitState.stanceSizes[legNum] = size;
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
Vector3d TrajectoryGenerator::linearTarget(const double& linearX, const double& linearY, const Vector3d& start, int legNum, bool invert) {
    double lx = linearX;
    double ly = linearY;
    std::swap(lx, ly);

    // Apply inversion if needed (for stance phase)
    if (invert) {ly = -ly;}

    // Apply leg mirroring
    if (converter.legConfigs[legNum].mirrored) {ly = -ly;}

    double magnitude = std::hypot(lx, ly) / gaitConfig.max_velocity;
    if (magnitude > 1.0) magnitude = 1.0;

    double stride = gaitConfig.max_stride_length * magnitude;
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
    Vector3d neutralPos = (it != gaitConfig.startPosition.end()) ? it->second : start;
    
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
Vector3d TrajectoryGenerator::rotationalTarget(const double& angularZ, const Vector3d& start, int legNum, bool invert) {
    double az = angularZ;

    // Apply inversion if needed (for stance phase)
    if (invert) {az = -az;}

    double magnitude = std::abs(az) / gaitConfig.max_angular_velocity;
    if (magnitude > 1.0) magnitude = 1.0;

    // Get the neutral/start position for this leg
    auto it = gaitConfig.startPosition.find(legNum);
    Vector3d neutralPos = (it != gaitConfig.startPosition.end()) ? it->second : start;
    
    // Calculate rotational offset as arc displacement from neutral
    // For small angles, arc length ≈ radius * angle
    // We use the neutral leg reach as the radius
    double neutralReach = std::hypot(neutralPos.x(), neutralPos.y());
    double maxArcLength = gaitConfig.max_stride_length;
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

Vector3d TrajectoryGenerator::evalBezier(const Vector3d& P0, const Vector3d& P1, const Vector3d& P2, const Vector3d& P3, double s) {
    double u = 1.0 - s;
    return u*u*u*P0
         + 3*u*u*s*P1
         + 3*u*s*s*P2
         + s*s*s*P3;
}

}  // namespace hexapod_gait_controller