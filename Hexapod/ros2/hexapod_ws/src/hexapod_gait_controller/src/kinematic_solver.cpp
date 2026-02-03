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
    JointAngles angles = solver.solveIK(target);

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
    Vector3d position = solver.solveFK(angles);

    response->foot_position.x = position.x();
    response->foot_position.y = position.y();
    response->foot_position.z = position.z();
}

/*
@brief Inverse kinematics, calculates the joint angles based on the desired position
@param target The desired position in 3D space (leg frame, X outward)
@param mirrored Whether this is a mirrored leg (legs 4, 5, 6)
@return The calculated joint angles
*/
JointAngles KinematicSolver::solveIK(const Vector3d& target, bool mirrored){
    auto clamp = [](double v) {return std::max(-1.0, std::min(1.0, v));};

    double a1 = coxaLength;
    double a2 = femurLength;
    double a3 = tibiaLength;

    double x = target.x();
    double y = target.y();
    double z = -target.z();  // Negate: positive Z in input = downward, but IK expects negative for down

    // Compute coxa angle - atan2(y, x) for X-forward convention
    double coxaAngle = atan2(y, x);
    
    // For mirrored legs, negate the coxa angle (joint rotates opposite direction)
    if (mirrored) {
        coxaAngle = -coxaAngle;
    }
    
    double r1 = std::hypot(x, y) - a1;
    double r2 = z;
    double q2 = atan2(r2, r1);  // Use atan2 for robustness
    double r3 = std::hypot(r1, r2);
    double q1 = acos(clamp((pow(a3,2) - pow(a2,2) - pow(r3,2)) / (-2*a2*r3)));
    double femurAngle = (q2 + q1);
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
Vector3d KinematicSolver::solveFK(const JointAngles& angles){
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