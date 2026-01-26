#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "hexapod_interfaces/msg/controller_input.hpp"

using std::placeholders::_1;
using JointState = sensor_msgs::msg::JointState;
using ControllerInput = hexapod_interfaces::msg::ControllerInput;

class EspBridge : public rclcpp::Node{public: EspBridge() : Node("esp_bridge_fake"){
    joint_command_sub = this->create_subscription<JointState>("joint_commands", 10, std::bind(&EspBridge::handle_joint_command, this, _1));
    controller_data_pub = this->create_publisher<ControllerInput>("controller_input_data", 10);
    joint_state_pub = this->create_publisher<JointState>("joint_states", 10);
    imu_data_pub = this->create_publisher<ControllerInput>("imu_data", 10);
    power_status_pub = this->create_publisher<ControllerInput>("power_status", 10);
    leg_contact_pub = this->create_publisher<ControllerInput>("leg_contact_data", 10);
}    
private:
    rclcpp::Subscription<JointState>::SharedPtr joint_command_sub;
    rclcpp::Publisher<ControllerInput>::SharedPtr controller_data_pub;
    rclcpp::Publisher<JointState>::SharedPtr joint_state_pub;
    rclcpp::Publisher<ControllerInput>::SharedPtr imu_data_pub;
    rclcpp::Publisher<ControllerInput>::SharedPtr power_status_pub;
    rclcpp::Publisher<ControllerInput>::SharedPtr leg_contact_pub;
public:
    void handle_joint_command(const JointState::SharedPtr msg);
};

int main(int argc, char **argv){    
    rclcpp::init(argc, argv);    
    auto node = std::make_shared<EspBridge>();    
    rclcpp::spin(node);    
    rclcpp::shutdown();    
    return 0;
}

void EspBridge::handle_joint_command(const JointState::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Received joint command with %zu positions", msg->position.size());
    // Process joint command as needed
}