#ifndef HEXAPOD_GAIT_CONTROLLER_KINEMATIC_SOLVER_HPP
#define HEXAPOD_GAIT_CONTROLLER_KINEMATIC_SOLVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <hexapod_interfaces/srv/ik_solver.hpp>
#include <hexapod_interfaces/srv/fk_solver.hpp>

using hexapod_interfaces::srv::IKSolver;
using hexapod_interfaces::srv::FKSolver;
using geometry_msgs::msg::Point;
using sensor_msgs::msg::JointState;

namespace hexapod_gait_controller {

class KinematicSolver {
private:
    static constexpr double coxaLength  = 0.0505;   // meters
    static constexpr double femurLength = 0.090;    // meters
    static constexpr double tibiaLength = 0.15035;  // meters
public:
    void solve_ik(const Point& target, int leg_index, JointState& joint_state);
    void solve_fk(const JointState& joint_state, Point& position);
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