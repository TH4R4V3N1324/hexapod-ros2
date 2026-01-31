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
    Vector3d target(request->target.x, request->target.y, request->target.z);
    JointAngles angles = solver.solve_ik(target);

    response->joint_state.name = {
        "leg" + std::to_string(request->leg_index + 1) + "_coxa_joint",
        "leg" + std::to_string(request->leg_index + 1) + "_femur_joint",
        "leg" + std::to_string(request->leg_index + 1) + "_tibia_joint"
    };

    response->joint_state.position = {
        angles.coxa, 
        angles.femur, 
        angles.tibia
    };

}

void KinematicSolverService::handle_fk_request(const std::shared_ptr<FKSolver::Request> request, std::shared_ptr<FKSolver::Response> response){
    JointAngles angles{
        request->joint_state.position[0],
        request->joint_state.position[1],
        request->joint_state.position[2]
    };
    Vector3d position = solver.solve_fk(angles);

    response->foot_position.x = position.x();
    response->foot_position.y = position.y();
    response->foot_position.z = position.z();
}

/*
@brief Inverse kinematics, calculates the joint angles based on the desired position
@param position The desired position in 3D space
@param legNum The leg number (1-6)
@return The calculated joint angles
*/
JointAngles KinematicSolver::solve_ik(const Vector3d& target){
    // geometry_msgs/Point target --> sensor_msgs/JointState joint_state
    auto clamp = [](double v) {return std::max(-1.0, std::min(1.0, v));};

    double a1 = coxaLength;
    double a2 = femurLength;
    double a3 = tibiaLength;

    double coxaAngle = atan2(target.y(), target.x());
    double r1 = std::hypot(target.x(), target.y()) - a1;
    double r2 = target.z();
    double q2 = atan(r2/r1);
    double r3 = std::hypot(r1, r2);
    double q1 = acos(clamp((pow(a3,2) - pow(a2,2) - pow(r3,2)) / (-2*a2*r3)));
    double femurAngle = (q2+q1);
    double q3 = acos(clamp((pow(r3,2) - pow(a2,2) - pow(a3,2)) / (-2*a2*a3)));
    double tibiaAngle = M_PI_2 - q3;

    return JointAngles{coxaAngle, femurAngle, tibiaAngle};
}

/*
@brief Forward kinematics, calculates the position based on the joint angles
@param angles The joint angles
@param legNum The leg number (1-6)
@return The calculated position in 3D space
*/
Vector3d KinematicSolver::solve_fk(const JointAngles& angles){
    // sensor_msgs/JointState joint_state --> geometry_msgs/Point foot_position
    double a1 = coxaLength;
    double a2 = femurLength;
    double a3 = tibiaLength;

    double coxaAngle  = angles.coxa;
    double femurAngle = angles.femur;
    double tibiaAngle = angles.tibia;

    double tibiaAbsoluteAngle = femurAngle - tibiaAngle - M_PI/2.0;

    double femurVertical = a2 * sin(femurAngle);
    double femurHorizontal = a2 * cos(femurAngle);
    double tibiaVertical = a3 * sin(tibiaAbsoluteAngle);
    double tibiaHorizontal = a3 * cos(tibiaAbsoluteAngle);
    
    double z = femurVertical + tibiaVertical;
    double horizontalReach = a1 + femurHorizontal + tibiaHorizontal;
    double x = horizontalReach * cos(coxaAngle);
    double y = horizontalReach * sin(coxaAngle);

    return Vector3d(x, y, z);
}

}  // namespace hexapod_gait_controller