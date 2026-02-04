#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "hexapod_gait_controller/kinematic_solver.hpp"
#include "hexapod_gait_controller/gait_config.hpp"
#include <chrono>

using namespace std::chrono_literals;

class LegAxisTester : public rclcpp::Node {
public:
    LegAxisTester()
    : Node("leg_axis_tester")
    {
        joint_cmd_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_commands", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&LegAxisTester::loop, this));

        RCLCPP_INFO(this->get_logger(), "Leg Axis Tester started");
    }

private:
    void loop() {
        static double t = 0.0;
        static int axis = 0; // 0 = X, 1 = Y, 2 = Z
        static int steps = 0;
        const int steps_per_axis = 100;

        double amp = 0.05; // 5 cm
        t += 0.05;
        steps++;

        if (steps > steps_per_axis) {
            steps = 0;
            axis = (axis + 1) % 3; // switch axis
            RCLCPP_INFO(this->get_logger(), "Switching to axis %d", axis);
        }

        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->now();

        for (int legNum = 1; legNum <= 6; ++legNum) {
            Vector3d neutral = gaitConfig.homePos;
            Vector3d target = neutral;

            // Move only one axis at a time
            switch(axis) {
                case 0: target.x() += amp * std::sin(t); break; // X
                case 1: target.y() += amp * std::sin(t); break; // Y
                case 2: target.z() += amp * std::sin(t); break; // Z
            }

            auto angles = ikSolver.solveIK(target, legNum);

            joint_state_msg.name.push_back("leg" + std::to_string(legNum) + "_coxa_joint");
            joint_state_msg.position.push_back(angles.coxa);

            joint_state_msg.name.push_back("leg" + std::to_string(legNum) + "_femur_joint");
            joint_state_msg.position.push_back(angles.femur);

            joint_state_msg.name.push_back("leg" + std::to_string(legNum) + "_tibia_joint");
            joint_state_msg.position.push_back(angles.tibia);

            current_leg_positions[legNum] = target;
        }

        joint_cmd_pub->publish(joint_state_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    std::map<int, Vector3d> current_leg_positions;

    hexapod_gait_controller::KinematicSolver ikSolver;    
    hexapod_gait_controller::GaitConfig gaitConfig; 
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LegAxisTester>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}