#include "rclcpp/rclcpp.hpp"
#include "hexapod_gait_controller/trajectory_generator.hpp"
#include "hexapod_gait_controller/kinematic_solver.hpp"
#include "hexapod_gait_controller/gait_config.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using geometry_msgs::msg::Twist;
using sensor_msgs::msg::JointState;
using hexapod_gait_controller::MAX_LEGS;
using hexapod_gait_controller::TrajectoryGenerator;

class GaitController : public rclcpp::Node { public: GaitController() : Node("gait_controller") {
    cmd_vel_sub = this->create_subscription<Twist>( "cmd_vel", 10, std::bind(&GaitController::cmdVelCallback, this, std::placeholders::_1));
    joint_cmd_pub = this->create_publisher<JointState>("joint_states", 10);
    RCLCPP_INFO(this->get_logger(), "GaitController node has been started.");
}    
private:
    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Publisher<JointState>::SharedPtr joint_cmd_pub;
    Twist last_cmd_vel;
    TrajectoryGenerator trajectoryGen;
    hexapod_gait_controller::KinematicSolver ikSolver;
    uint8_t step = 0;
    uint8_t phase = 0;
public:
    void cmdVelCallback(const Twist::SharedPtr msg);
    void PerformLegStep(bool idle, int resolution, bool handlePhaseTransition);
};

int main(int argc, char **argv){    
    rclcpp::init(argc, argv);    
    auto node = std::make_shared<GaitController>();    
    rclcpp::spin(node);    
    rclcpp::shutdown();    
    return 0;
}

void GaitController::cmdVelCallback(const Twist::SharedPtr msg) {
    last_cmd_vel = *msg;
    RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear(%.2f, %.2f, %.2f), angular(%.2f, %.2f, %.2f)",
                msg->linear.x, msg->linear.y, msg->linear.z,
                msg->angular.x, msg->angular.y, msg->angular.z);
    // Here you would typically update the gait based on the received cmd_vel
}

/*
@brief Perform a leg step based on the current trajectories
@param idle Whether the joystick is idle
@param resolution The resolution of the trajectories
@param handlePhaseTransition Whether to handle phase transitions
@note If idle is true, the step will not advance
@note If handlePhaseTransition is false, phase transitions will be skipped and must be handled externally
*/
void GaitController::PerformLegStep(bool idle, int resolution, bool handlePhaseTransition) {
    auto joint_state_msg = JointState();
    joint_state_msg.header.stamp = this->now();
    
    // Prepare joint names (18 joints total: 6 legs × 3 joints)
    joint_state_msg.name.reserve(18);
    joint_state_msg.position.reserve(18);
    
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        int swingSize = trajectoryGen.gaitState.swingSizes[legNum];
        int stanceSize = trajectoryGen.gaitState.stanceSizes[legNum];
        
        // Get target position for this leg
        Eigen::Vector3d targetPos;
        if (step < swingSize) {
            targetPos = trajectoryGen.gaitState.swingTrajectory[legNum][step];
        } else if (step < stanceSize) {
            targetPos = trajectoryGen.gaitState.stanceTrajectory[legNum][step];
        } else {
            continue; // Skip if no valid trajectory
        }
        
        // Compute IK to get joint angles
        auto angles = ikSolver.solveIK(targetPos);
        
        // Add joint names and positions
        joint_state_msg.name.push_back("leg" + std::to_string(legNum) + "_coxa_joint");
        joint_state_msg.position.push_back(angles.coxa);
        
        joint_state_msg.name.push_back("leg" + std::to_string(legNum) + "_femur_joint");
        joint_state_msg.position.push_back(angles.femur);
        
        joint_state_msg.name.push_back("leg" + std::to_string(legNum) + "_tibia_joint");
        joint_state_msg.position.push_back(angles.tibia);
    }
    
    // Publish joint states
    joint_cmd_pub->publish(joint_state_msg);
    
    // Advance step if not idle
    if (!idle) step++;
    if (!handlePhaseTransition) return;

    // Phase transition
    if (step > resolution) {
        phase = (phase + 1) % trajectoryGen.gaitState.config.size();
        step = 0;
    }    
}