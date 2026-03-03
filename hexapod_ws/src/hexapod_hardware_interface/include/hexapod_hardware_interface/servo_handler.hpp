#ifndef HEXAPOD_HARDWARE_INTERFACE__SERVO_HANDLER_HPP_
#define HEXAPOD_HARDWARE_INTERFACE__SERVO_HANDLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <array>
#include <vector>

using sensor_msgs::msg::JointState;
using std::placeholders::_1;

namespace hexapod_hardware_interface {

class ServoHandler {
public:
    explicit ServoHandler(rclcpp::Node* node);
    bool hasNewCommands() const { return has_new_commands; }
    std::vector<uint8_t> buildPacket();
    void publishJointStates(const rclcpp::Time& timestamp);
    const std::array<float, 18>& getCurrentPositions() const { return current_positions; }
    void simulateMovement(double alpha = 0.3);
private:
    void jointCommandCallback(const JointState::SharedPtr msg);
    void packFloat(std::vector<uint8_t>& buffer, float value);
    uint8_t calculateChecksum(const std::vector<uint8_t>& data);
    
    rclcpp::Subscription<JointState>::SharedPtr joint_cmd_sub;
    rclcpp::Publisher<JointState>::SharedPtr joint_state_pub;
    
    std::array<float, 18> current_positions;
    std::array<float, 18> commanded_positions;
    bool has_new_commands{false};
    
    rclcpp::Node* node;
};

} // namespace hexapod_hardware_interface

#endif