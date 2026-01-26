#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "hexapod_interfaces/msg/power_status.hpp"
#include "hexapod_interfaces/msg/leg_contact.hpp"
#include "hexapod_interfaces/msg/controller_input.hpp"
#include "std_srvs/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using JointState = sensor_msgs::msg::JointState;
using Imu = sensor_msgs::msg::Imu;
using PowerStatus = hexapod_interfaces::msg::PowerStatus;
using LegContact = hexapod_interfaces::msg::LegContact;
using ControllerInput = hexapod_interfaces::msg::ControllerInput;
using SetBool = std_srvs::srv::SetBool;

class EspBridge : public rclcpp::Node{public: EspBridge() : Node("esp_bridge_fake"){
    joint_command_sub = this->create_subscription<JointState>("joint_commands", 10, std::bind(&EspBridge::handle_joint_command, this, _1));
    controller_data_pub = this->create_publisher<ControllerInput>("controller_input_data", 10);
    joint_state_pub = this->create_publisher<JointState>("joint_states", 10);
    imu_data_pub = this->create_publisher<Imu>("imu_data", 10);
    power_status_pub = this->create_publisher<PowerStatus>("power_status", 10);
    leg_contact_pub = this->create_publisher<LegContact>("leg_contact_data", 10);
    emergency_stop_service = this->create_service<SetBool>("emergency_stop", std::bind(&EspBridge::handle_emergency_stop, this, _1, _2));
}    
private:
    rclcpp::Subscription<JointState>::SharedPtr joint_command_sub;
    rclcpp::Publisher<ControllerInput>::SharedPtr controller_data_pub;
    rclcpp::Publisher<JointState>::SharedPtr joint_state_pub;
    rclcpp::Publisher<Imu>::SharedPtr imu_data_pub;
    rclcpp::Publisher<PowerStatus>::SharedPtr power_status_pub;
    rclcpp::Publisher<LegContact>::SharedPtr leg_contact_pub;
    rclcpp::Service<SetBool>::SharedPtr emergency_stop_service;
public:
    void handle_joint_command(const JointState::SharedPtr msg);
    void handle_emergency_stop(const std::shared_ptr<SetBool::Request> request, std::shared_ptr<SetBool::Response> response);
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

void EspBridge::handle_emergency_stop(const std::shared_ptr<SetBool::Request> request,
                                     std::shared_ptr<SetBool::Response> response){
    if(request->data){
        // Activate emergency stop
        RCLCPP_WARN(this->get_logger(), "Emergency stop activated!");
        response->success = true;
        response->message = "Emergency stop activated.";
    } else {
        // Deactivate emergency stop
        RCLCPP_INFO(this->get_logger(), "Emergency stop deactivated.");
        response->success = true;
        response->message = "Emergency stop deactivated.";
    }
}