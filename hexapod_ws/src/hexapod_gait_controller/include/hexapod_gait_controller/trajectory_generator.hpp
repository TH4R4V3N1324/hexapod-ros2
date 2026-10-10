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

/**
 * @brief Pose of the body relative to its planted feet, used in tilt mode
*/
struct BodyPose {
    Vector3d translation = Vector3d::Zero();  // m, body shift (+x forward, +y left, +z up)
    double roll = 0.0;   // rad, positive lowers the right side
    double pitch = 0.0;  // rad, positive lowers the nose
    double yaw = 0.0;    // rad, positive turns anticlockwise
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
    double legYaw(int legNum);
    Vector3d legToBody(int legNum, const Vector3d& position);
    Vector3d bodyToLeg(int legNum, const Vector3d& position);
    Vector3d posedFoot(int legNum, const Vector3d& groundFoot, const BodyPose& pose);
    Vector3d unposedBodyPoint(const Vector3d& bodyPoint, const BodyPose& pose);
    static BodyPose rotationAbout(double roll, double pitch, const Vector3d& pivot);
    Vector3d neutralFootInBody(int legNum);
    Vector3d footDisplacement(double forward, double left, double yaw, int legNum, double duration);
    Vector3d footDisplacementFrom(double forward, double left, double yaw, int legNum, const Vector3d& footPos, double duration);
    double bodyRadius(int legNum);
    double maxBodyRadius();
    Vector3d swingTarget(int legNum, const Vector3d& displacement);
    Vector3d stanceTarget(int legNum, const Vector3d& currentPos, const Vector3d& displacement);
    Vector3d clampToStride(int legNum, const Vector3d& target);
    void EnsureGaitConfig();
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