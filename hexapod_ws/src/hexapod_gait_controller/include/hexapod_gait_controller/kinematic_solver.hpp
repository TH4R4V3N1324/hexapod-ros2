#ifndef HEXAPOD_GAIT_CONTROLLER_KINEMATIC_SOLVER_HPP
#define HEXAPOD_GAIT_CONTROLLER_KINEMATIC_SOLVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <hexapod_interfaces/srv/ik_solver.hpp>
#include <hexapod_interfaces/srv/fk_solver.hpp>

namespace hexapod_gait_controller {

using hexapod_interfaces::srv::IKSolver;
using hexapod_interfaces::srv::FKSolver;
using geometry_msgs::msg::Point;
using sensor_msgs::msg::JointState;
using Eigen::Vector3d;

struct JointAngles {
    double coxa;
    double femur;
    double tibia;
};

class KinematicSolver {
private:
    // Hexapod_V6.1, matching the URDF. The tibia is curved: its length is the straight line
    // from the knee axis to the bottom of the foot.
    static constexpr double coxaLength  = 0.063;    // meters
    static constexpr double femurLength = 0.100;    // meters
    static constexpr double tibiaLength = 0.186;    // meters
public:
    JointAngles solveIK(const Vector3d& target, bool mirrored = false);
    Vector3d solveFK(const JointAngles& angles);
    Eigen::Matrix3d jacobian(const JointAngles& angles);
    bool groundForce(const JointAngles& angles, const Vector3d& torques, Vector3d& force);
};

class KinematicSolverService {
public:
    explicit KinematicSolverService(rclcpp::Node* node);
    void handle_ik_request(const std::shared_ptr<IKSolver::Request> request, std::shared_ptr<IKSolver::Response> response);
    void handle_fk_request(const std::shared_ptr<FKSolver::Request> request, std::shared_ptr<FKSolver::Response> response);
private:
    KinematicSolver solver;
    rclcpp::Node* node;
    rclcpp::Service<IKSolver>::SharedPtr ik_service;
    rclcpp::Service<FKSolver>::SharedPtr fk_service;
};

}  // namespace hexapod_gait_controller

#endif  // HEXAPOD_GAIT_CONTROLLER_KINEMATIC_SOLVER_HPP