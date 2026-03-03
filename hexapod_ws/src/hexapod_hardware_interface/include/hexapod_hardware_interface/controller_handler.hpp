#ifndef HEXAPOD_HARDWARE_INTERFACE__CONTROLLER_HANDLER_HPP_
#define HEXAPOD_HARDWARE_INTERFACE__CONTROLLER_HANDLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <hexapod_interfaces/msg/controller_input.hpp>
#include <vector>

using hexapod_interfaces::msg::ControllerInput;

namespace hexapod_hardware_interface {

class ControllerHandler {
public:
    explicit ControllerHandler(rclcpp::Node* node);
    void parseAndPublish(const std::vector<uint8_t>& data, const rclcpp::Time& timestamp);
    void publishFakeData(const rclcpp::Time& timestamp);
private:
    float unpackFloat(const uint8_t* bytes);
    rclcpp::Publisher<ControllerInput>::SharedPtr controller_pub;
    rclcpp::Node* node;
};

} // namespace hexapod_hardware_interface

#endif