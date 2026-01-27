#include "rclcpp/rclcpp.hpp"
#include "hexapod_gait_controller/kinematic_solver.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using IKSolver = hexapod_interfaces::srv::IKSolver;
using FKSolver = hexapod_interfaces::srv::FKSolver;

namespace hexapod_gait_controller {

KinematicSolverService::KinematicSolverService(rclcpp::Node* node) : node(node) {
    ik_service = node->create_service<IKSolver>("ik_solver", std::bind(&KinematicSolverService::handle_ik_request, this, _1, _2));
    fk_service = node->create_service<FKSolver>("fk_solver", std::bind(&KinematicSolverService::handle_fk_request, this, _1, _2));

    RCLCPP_INFO(node->get_logger(), "KinematicSolver services initialized");
}

void KinematicSolverService::handle_ik_request(const std::shared_ptr<IKSolver::Request> request, std::shared_ptr<IKSolver::Response> response){
    solver.solve_ik(request->target, request->leg_index, response->joint_state);
}

void KinematicSolverService::handle_fk_request(const std::shared_ptr<FKSolver::Request> request, std::shared_ptr<FKSolver::Response> response){
    solver.solve_fk(request->joint_state, response->foot_position);
}

void KinematicSolver::solve_ik(const Point& target, int leg_index, JointState& joint_state){
    // geometry_msgs/Point target --> sensor_msgs/JointState joint_state
    auto clamp = [](double v) {return std::max(-1.0, std::min(1.0, v));};

    double a1 = coxaLength;
    double a2 = femurLength;
    double a3 = tibiaLength;

    double coxaAngle = atan2(target.y, target.x);
    double r1 = std::hypot(target.x, target.y) - a1;
    double r2 = target.z;
    double q2 = atan(r2/r1);
    double r3 = std::hypot(r1, r2);
    double q1 = acos(clamp((pow(a3,2) - pow(a2,2) - pow(r3,2)) / (-2*a2*r3)));
    double femurAngle = (q2+q1);
    double q3 = acos(clamp((pow(r3,2) - pow(a2,2) - pow(a3,2)) / (-2*a2*a3)));
    double tibiaAngle = M_PI_2 - q3;

    std::string prefix = "leg" + std::to_string(leg_index + 1);

    joint_state.name = {
        prefix + "_coxa_joint",
        prefix + "_femur_joint",
        prefix + "_tibia_joint"
    };

    joint_state.position = {
        coxaAngle, 
        femurAngle, 
        tibiaAngle
    };
}

void KinematicSolver::solve_fk(const JointState& joint_state, Point& position){
    // sensor_msgs/JointState joint_state --> geometry_msgs/Point foot_position
    if (joint_state.position.size() < 3) {
        // Note: Error handling should be done at service level
        return;
    }

    double a1 = coxaLength;
    double a2 = femurLength;
    double a3 = tibiaLength;

    double coxaAngle  = joint_state.position[0];
    double femurAngle = joint_state.position[1];
    double tibiaAngle = joint_state.position[2];

    double tibiaAbsoluteAngle = femurAngle - tibiaAngle - M_PI/2.0;

    double femurVertical = a2 * sin(femurAngle);
    double femurHorizontal = a2 * cos(femurAngle);
    double tibiaVertical = a3 * sin(tibiaAbsoluteAngle);
    double tibiaHorizontal = a3 * cos(tibiaAbsoluteAngle);
    
    double z = femurVertical + tibiaVertical;
    double horizontalReach = a1 + femurHorizontal + tibiaHorizontal;
    double x = horizontalReach * cos(coxaAngle);
    double y = horizontalReach * sin(coxaAngle);

    position.x = x;
    position.y = y;
    position.z = z;
}

}  // namespace hexapod_gait_controller