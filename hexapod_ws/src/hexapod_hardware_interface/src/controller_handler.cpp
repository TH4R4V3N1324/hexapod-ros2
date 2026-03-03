#include "hexapod_hardware_interface/controller_handler.hpp"
#include <cstring>

namespace hexapod_hardware_interface {

ControllerHandler::ControllerHandler(rclcpp::Node* node) : node(node) {
    controller_pub = node->create_publisher<ControllerInput>("controller_input",10);

    RCLCPP_INFO(node->get_logger(), "ControllerHandler initialized");
}

void ControllerHandler::parseAndPublish(const std::vector<uint8_t>& data, const rclcpp::Time& timestamp) {
    // Protocol from ESP32:
    // [19-22]: joy_lx (float)
    // [23-26]: joy_ly (float)
    // [27-30]: joy_rx (float)
    // [31-34]: joy_ry (float)
    // [35-36]: command_id (uint16)
    // [37-40]: arg1 (float)
    // [41-44]: arg2 (float)
    // [45-48]: arg3 (float)
    
    if (data.size() < 49) {
        return; // Not enough data
    }
    
    ControllerInput msg;
    msg.stamp = timestamp;
    msg.source_id = ControllerInput::SOURCE_ESPNOW;
    
    // Parse axes
    msg.axes.resize(4);
    msg.axes[0] = unpackFloat(&data[19]); // left_x
    msg.axes[1] = unpackFloat(&data[23]); // left_y
    msg.axes[2] = unpackFloat(&data[27]); // right_x
    msg.axes[3] = unpackFloat(&data[31]); // right_y
    
    // Parse command
    msg.command_id = (data[36] << 8) | data[35];
    
    // Parse arguments
    msg.arguments.resize(3);
    msg.arguments[0] = unpackFloat(&data[37]);
    msg.arguments[1] = unpackFloat(&data[41]);
    msg.arguments[2] = unpackFloat(&data[45]);
    
    controller_pub->publish(msg);
}

void ControllerHandler::publishFakeData(const rclcpp::Time& timestamp) {
    ControllerInput msg;
    msg.stamp = timestamp;
    msg.source_id = ControllerInput::SOURCE_ESPNOW;
    
    // Neutral joystick position
    msg.axes = {0.0, 0.0, 0.0, 0.0};
    msg.command_id = ControllerInput::CMD_NONE;
    msg.arguments = {};
    
    controller_pub->publish(msg);
}

float ControllerHandler::unpackFloat(const uint8_t* bytes) {
    float value;
    std::memcpy(&value, bytes, sizeof(float));
    return value;
}

} // namespace hexapod_hardware_interface