#include "hexapod_gait_controller/kinematic_solver.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using IKSolver = hexapod_interfaces::srv::IKSolver;
using FKSolver = hexapod_interfaces::srv::FKSolver;

namespace hexapod_gait_controller {

/**
 * @brief Create the IK and FK solver services
 * @param[in] node The node that owns the services
*/
KinematicSolverService::KinematicSolverService(rclcpp::Node* node) : node(node) {
    ik_service = node->create_service<IKSolver>("ik_solver", std::bind(&KinematicSolverService::handle_ik_request, this, _1, _2));
    fk_service = node->create_service<FKSolver>("fk_solver", std::bind(&KinematicSolverService::handle_fk_request, this, _1, _2));

    RCLCPP_INFO(node->get_logger(), "KinematicSolver services initialized");
}

/**
 * @brief Service callback that solves IK for a requested foot position
 * @param[in] request The service request
 * @param[out] response The service response
 * @note leg_index in the request is zero-based
*/
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

/**
 * @brief Service callback that solves FK for requested joint angles
 * @param[in] request The service request
 * @param[out] response The service response
*/
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

/**
 * @brief Inverse kinematics, calculates the joint angles based on the desired position
 * @param[in] target The desired position in 3D space (leg frame, X outward)
 * @param[in] mirrored Whether this is a mirrored leg (legs 4, 5, 6)
 * @return The calculated joint angles
 * @note If the target is outside the workspace, angles are clamped to the nearest valid solution
*/
JointAngles KinematicSolver::solveIK(const Vector3d& target, bool mirrored){
    auto clamp = [](double v) {return std::max(-1.0, std::min(1.0, v));};

    double a1 = coxaLength;
    double a2 = femurLength;
    double a3 = tibiaLength;

    double x = target.x();
    double y = target.y();
    double z = target.z();

    if(mirrored){
        y = -y;
    }

    double coxaAngle = atan2(y, x);
    double r1 = std::hypot(x, y) - a1;
    double r2 = z;
    double q2 = atan2(r2, r1);
    double r3 = std::hypot(r1, r2);
    double q1 = acos(clamp((pow(a3,2) - pow(a2,2) - pow(r3,2)) / (-2*a2*r3)));
    double femurAngle = -(q2 + q1);
    double q3 = acos(clamp((pow(r3,2) - pow(a2,2) - pow(a3,2)) / (-2*a2*a3)));
    double tibiaAngle = M_PI_2 - q3;

    return JointAngles{coxaAngle, femurAngle, tibiaAngle};
}

/**
 * @brief Forward kinematics, calculates the position based on the joint angles
 * @param[in] angles The joint angles
 * @return The foot position (leg frame, X outward, Z negative below the hip)
*/
Vector3d KinematicSolver::solveFK(const JointAngles& angles){
    // sensor_msgs/JointState joint_state --> geometry_msgs/Point foot_position
    double a1 = coxaLength;
    double a2 = femurLength;
    double a3 = tibiaLength;

    double coxaAngle  = angles.coxa;

    // Elevation of each link above horizontal, matching solveIK:
    // a negative femur angle raises the femur, and tibia = 0 puts the knee at 90°
    double femurElevation = -angles.femur;
    double tibiaElevation = femurElevation - angles.tibia - M_PI / 2.0;

    double horizontalReach = a1 + a2 * cos(femurElevation) + a3 * cos(tibiaElevation);
    double z = a2 * sin(femurElevation) + a3 * sin(tibiaElevation);  // negative = below the hip
    double x = horizontalReach * cos(coxaAngle);
    double y = horizontalReach * sin(coxaAngle);

    return Vector3d(x, y, z);
}

/**
 * @brief Jacobian of the foot position with respect to the joint angles
 * @param[in] angles The joint angles
 * @return d(foot position) / d(coxa, femur, tibia), leg frame, by central differences of solveFK
*/
Eigen::Matrix3d KinematicSolver::jacobian(const JointAngles& angles) {
    constexpr double h = 1e-6;  // rad
    Eigen::Matrix3d J;
    for (int i = 0; i < 3; ++i) {
        JointAngles plus = angles, minus = angles;
        double* p = (i == 0) ? &plus.coxa : (i == 1) ? &plus.femur : &plus.tibia;
        double* m = (i == 0) ? &minus.coxa : (i == 1) ? &minus.femur : &minus.tibia;
        *p += h;
        *m -= h;
        J.col(i) = (solveFK(plus) - solveFK(minus)) / (2.0 * h);
    }
    return J;
}

/**
 * @brief Force the ground pushes on the foot with, from the joint torques holding the leg still
 * @param[in] angles The joint angles
 * @param[in] torques The torque each joint's servo applies (coxa, femur, tibia; N m)
 * @param[out] force The force on the foot (leg frame, N); +z means the ground is pushing up
 * @return false if the leg is too close to straight for the force to be found
 * @note In balance the servo torques cancel the ground force's torque: torques = -J^T force.
 *       The leg's own weight is ignored (about 0.1 N m on the femur).
*/
bool KinematicSolver::groundForce(const JointAngles& angles, const Vector3d& torques, Vector3d& force) {
    const Eigen::Matrix3d Jt = jacobian(angles).transpose();
    if (std::abs(Jt.determinant()) < 1e-6) {
        return false;
    }
    force = -Jt.inverse() * torques;
    return true;
}

}  // namespace hexapod_gait_controller
