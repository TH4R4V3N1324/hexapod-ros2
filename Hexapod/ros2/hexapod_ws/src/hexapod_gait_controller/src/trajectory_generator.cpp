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
    if ((blended - currentPos).norm() > gaitConfig.max_stride_length) {
         blended = currentPos + (blended - currentPos).normalized() * gaitConfig.max_stride_length;
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

    // Generate swing trajectories
    for (int legNum : swingGroup) {
        Vector3d targetLegFrame = converter.bodyToLegFrame(swingTargetsBodyFrame[legNum], legNum);
        int size = 0;
        GenBezierTrajectory(
            gaitState.swingTrajectory[legNum].data(),
            size,
            currentPositions[legNum],
            targetLegFrame,
            liftHeight,
            resolution
        );
        gaitState.swingSizes[legNum] = size;
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
@param start The starting position
@param legNum The leg number
@param invert Whether to invert the direction (for stance phase)
@param strideMultiplier Multiplier for stride length
@return The calculated target position
*/
Vector3d TrajectoryGenerator::linearTarget(const double& linearX, const double& linearY, const Vector3d& start, int legNum, bool invert, double strideMultiplier) {
    double lx = linearX;
    double ly = linearY;
    std::swap(lx, ly);

    // Apply inversion if needed (for stance phase)
    if (invert) {ly = -ly;}

    // Apply leg mirroring
    if (converter.legConfigs[legNum].mirrored) {ly = -ly;}

    double magnitude = std::hypot(lx, ly) / gaitConfig.max_velocity;
    if (magnitude > 1.0) magnitude = 1.0;

    double stride = gaitConfig.max_stride_length * magnitude * strideMultiplier;
    double angle = atan2(ly, lx);

    double deltaX = stride * cos(angle);
    double deltaY = stride * sin(angle);
    double groundZ = gaitConfig.currentHeight;

    double legAngle = converter.legConfigs[legNum].mounting_angle;
    if (!converter.legConfigs[legNum].mirrored) {legAngle = -legAngle;}

    double dx_rot = deltaX * cos(legAngle) - deltaY * sin(legAngle);
    double dy_rot = deltaX * sin(legAngle) + deltaY * cos(legAngle);

    return {start.x() + dx_rot, start.y() + dy_rot, groundZ};
}

/*
@brief Calculate rotational target position based on input angular velocity
@param angularZ The angular velocity around the Z-axis
@param start The starting position
@param legNum The leg number
@param invert Whether to invert the direction (for stance phase)
@param strideMultiplier Multiplier for stride length
@return The calculated target position
*/
Vector3d TrajectoryGenerator::rotationalTarget(const double& angularZ, const Vector3d& start, int legNum, bool invert, double strideMultiplier) {
    double az = angularZ;

    // Apply inversion if needed (for stance phase)
    if (invert) {az = -az;}

    double magnitude = std::abs(az) / gaitConfig.max_angular_velocity;
    if (magnitude > 1.0) magnitude = 1.0;

    double rotationRadius = gaitConfig.max_stride_length * strideMultiplier;
    double offset = rotationRadius * magnitude * (az > 0 ? 1.0 : -1.0);
    
    double legAngle = converter.legConfigs[legNum].mounting_angle;

    // Tangential direction: perpendicular to the radial direction from body center
    double deltaX = -offset * sin(legAngle);
    double deltaY = offset * cos(legAngle);
    double groundZ = gaitConfig.currentHeight;

    return {start.x() + deltaX, start.y() + deltaY, groundZ};
}

}  // namespace hexapod_gait_controller