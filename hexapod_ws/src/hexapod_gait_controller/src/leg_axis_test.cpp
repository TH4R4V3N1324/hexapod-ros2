#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "hexapod_gait_controller/kinematic_solver.hpp"
#include "hexapod_gait_controller/trajectory_generator.hpp"

using namespace std::chrono_literals;
using namespace hexapod_gait_controller;

class LegAxisTestNode : public rclcpp::Node {
public:
    LegAxisTestNode() : Node("leg_axis_test_node") {
        RCLCPP_INFO(get_logger(), "Starting leg axis test node...");

        joint_cmd_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_commands", 10);

        // Home position: leg extended outward (X), level (Y=0), foot below body (Z negative)
        // Total leg reach: coxa(0.0505) + femur(0.090) + tibia(0.15035) ≈ 0.29m
        // Use a position with foot ~0.12m below hip level for a natural standing pose
        homePos = {0.15, 0.0, 0};

        // Test parameters
        stepSize = 0.05;       // 5 cm along each axis
        resolution = 50;       // number of trajectory points

        // Start the test loop
        timer = this->create_wall_timer(100ms, std::bind(&LegAxisTestNode::runTest, this));
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub;
    rclcpp::TimerBase::SharedPtr timer;

    TrajectoryGenerator trajGen;
    KinematicSolver solver;

    Vector3d homePos;
    double stepSize;
    int resolution;

    int axisStage = 0;      // 0=X, 1=Y, 2=Z
    std::size_t trajIndex = 0;

    std::vector<Vector3d> currentTrajectory;

    void runTest() {
        // If trajectory not initialized or finished, generate new one
        if (currentTrajectory.empty() || trajIndex >= currentTrajectory.size()) {
            trajIndex = 0;
            currentTrajectory.clear();
            Vector3d start = homePos;
            Vector3d end;

            switch (axisStage) {
                case 0: end = {homePos.x() + stepSize, homePos.y(), homePos.z()}; break; // X
                case 1: end = {homePos.x(), homePos.y() + stepSize, homePos.z()}; break; // Y
                case 2: end = {homePos.x(), homePos.y(), homePos.z() - stepSize}; break; // Z
            }

            currentTrajectory.resize(resolution + 1);
            int size = 0;
            trajGen.GenStraightTrajectory(currentTrajectory.data(), size, start, end, resolution);
            if (size > 0) {
                currentTrajectory.resize(static_cast<std::size_t>(size));
            }

            axisStage = (axisStage + 1) % 3;  // cycle to next axis
        }

        // Publish joint commands for all 6 legs
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = this->now();

        for (int legNum = 1; legNum <= 6; ++legNum) {
            bool mirrored = (legNum > 3);  // right legs

            Vector3d target = currentTrajectory[trajIndex];
            RCLCPP_INFO(get_logger(), "Leg %d target: (%.3f, %.3f, %.3f)", legNum, target.x(), target.y(), target.z());

            JointAngles angles = solver.solveIK(target, mirrored);

            joint_state_msg.name.push_back("leg" + std::to_string(legNum) + "_coxa_joint");
            joint_state_msg.position.push_back(angles.coxa);

            joint_state_msg.name.push_back("leg" + std::to_string(legNum) + "_femur_joint");
            joint_state_msg.position.push_back(angles.femur);

            joint_state_msg.name.push_back("leg" + std::to_string(legNum) + "_tibia_joint");
            joint_state_msg.position.push_back(angles.tibia);

            RCLCPP_INFO(get_logger(), 
                "Leg %d angles: Coxa=%.3f, Femur=%.3f, Tibia=%.3f", 
                legNum, angles.coxa * 180.0 / M_PI, angles.femur * 180.0 / M_PI, angles.tibia * 180.0 / M_PI);
        }

        joint_cmd_pub->publish(joint_state_msg);
        trajIndex++;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LegAxisTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}