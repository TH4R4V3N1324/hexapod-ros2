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

struct SwingSpline {
    Eigen::Vector3d P0, P1, P2, P3, P4;
    double s = 0.0;        // normalized progress [0,1]
    bool active = false;
};

struct LineSpline {
    Eigen::Vector3d P0, P1;
    double s = 0.0;        // normalized progress [0,1]
    bool active = false;
};

struct GaitState {
    std::vector<std::vector<int>> config;
    std::array<SwingSpline, MAX_LEGS + 1> swingSplines;
    std::array<LineSpline, MAX_LEGS + 1> stanceSplines;
};

class TrajectoryGenerator {
private:
    GaitConfig gaitConfig;
    FrameConverter converter;
public:
    GaitState gaitState;
    void GenStraightTrajectory(
        Vector3d* trajectory, 
        int& outSize, 
        const Vector3d& start, 
        const Vector3d& end
    );
    void GenBezierTrajectory(
        Vector3d* trajectory, 
        int& outSize, 
        const Vector3d& start, 
        const Vector3d& end
    );
    void GenerateTrajectories(
        std::function<Vector3d(int, const Vector3d&)> swingTargetFunc,
        std::function<Vector3d(int, const Vector3d&)> stanceTargetFunc,
        std::array<Vector3d, MAX_LEGS + 1> currentPositions,
        uint8_t currentPhase
    );
    Vector3d linearTarget(
        const double& linearX, 
        const double& linearY, 
        int legNum, 
        bool invert
    );
    Vector3d rotationalTarget(
        const double& angularZ, 
        int legNum, 
        bool invert
    );
    Vector3d BlendTargetPosition(
        const Vector3d& currentPos, 
        const Vector3d& translationPos, 
        const Vector3d& rotationPos,
        int legNum,
        double strideMultiplier = 1.0
    );
    void EnsureGaitConfig();
    double CalculateStrideMultiplier();
    void genSwingSpline(SwingSpline& spline, const Vector3d& start, const Vector3d& end);
    Vector3d evalBezier(
        const Vector3d& P0, 
        const Vector3d& P1, 
        const Vector3d& P2, 
        const Vector3d& P3,
        const Vector3d& P4, 
        double s
    );
};

}  // namespace hexapod_gait_controller

#endif // HEXAPOD_GAIT_CONTROLLER__TRAJECTORY_GENERATOR_HPP