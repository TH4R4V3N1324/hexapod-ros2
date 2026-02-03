#ifndef HEXAPOD_GAIT_CONTROLLER__TRAJECTORY_GENERATOR_HPP
#define HEXAPOD_GAIT_CONTROLLER__TRAJECTORY_GENERATOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "hexapod_gait_controller/gait_config.hpp"
#include "hexapod_gait_controller/frame_converter.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace hexapod_gait_controller {

using Eigen::Vector3d;
using hexapod_gait_controller::GaitConfig;
using geometry_msgs::msg::Twist;

struct GaitState {
    std::vector<std::vector<int>> config;
    std::array<std::array<Vector3d, MAX_RESOLUTION>, MAX_LEGS + 1> swingTrajectory;  // 1-based indexing
    std::array<std::array<Vector3d, MAX_RESOLUTION>, MAX_LEGS + 1> stanceTrajectory;
    std::array<int, MAX_LEGS + 1> swingSizes{};   // Store actual size for each leg
    std::array<int, MAX_LEGS + 1> stanceSizes{};
};

class TrajectoryGenerator {
private:
    void GenStraightTrajectory(Vector3d* trajectory, int& outSize, const Vector3d& start, const Vector3d& end, int resolution);
    void GenBezierTrajectory(Vector3d* trajectory, int& outSize, const Vector3d& start, const Vector3d& end, double liftHeight, int resolution);
    
    GaitConfig gaitConfig;
    FrameConverter converter;
public:
    GaitState gaitState;
    void GenerateTrajectories(
        double liftHeight,
        int resolution,
        std::function<Vector3d(int, const Vector3d&)> swingTargetFunc,
        std::function<Vector3d(int, const Vector3d&)> stanceTargetFunc,
        std::array<Vector3d, MAX_LEGS + 1> currentPositions,
        uint8_t currentPhase
    );
    Vector3d direction(const Twist& cmdVel, const Vector3d& start, int legNum, bool invert = false, double strideMultiplier = 1.0, bool useBodyFrame = true);
    Vector3d BlendTargetPosition(const Vector3d& currentPos, const Vector3d& translationPos, const Vector3d& rotationPos);
    void EnsureGaitConfig();
    double CalculateStrideMultiplier();
};

}  // namespace hexapod_gait_controller

#endif // HEXAPOD_GAIT_CONTROLLER__TRAJECTORY_GENERATOR_HPP