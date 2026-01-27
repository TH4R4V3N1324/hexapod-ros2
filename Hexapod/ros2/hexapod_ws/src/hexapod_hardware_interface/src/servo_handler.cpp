#include "hexapod_hardware_interface/servo_handler.hpp"
#include <cstring>

namespace hexapod_hardware_interface {

ServoHandler::ServoHandler(rclcpp::Node* node) : node(node) {
    joint_cmd_sub = node->create_subscription<JointState>("joint_commands", 10, std::bind(&ServoHandler::jointCommandCallback, this, _1));
    joint_state_pub = node->create_publisher<JointState>("joint_states", 10);
    
    // Initialize to zero
    current_positions.fill(0.0f);
    commanded_positions.fill(0.0f);
    
    RCLCPP_INFO(node->get_logger(), "ServoHandler initialized");
}

void ServoHandler::jointCommandCallback(const JointState::SharedPtr msg) {
    if (msg->position.size() != 18) {
        RCLCPP_WARN(node->get_logger(), 
            "Expected 18 joint angles, got %zu", msg->position.size());
        return;
    }
    
    for (size_t i = 0; i < 18; i++) {
        commanded_positions[i] = static_cast<float>(msg->position[i]);
    }
    
    has_new_commands = true;
}

std::vector<uint8_t> ServoHandler::buildPacket() {
    // Protocol: [0xAA][length][18 floats][checksum]
    std::vector<uint8_t> packet;
    packet.reserve(75);
    
    packet.push_back(0xAA);  // Header
    packet.push_back(72);    // 18 floats * 4 bytes
    
    // Pack all joint angles
    for (float angle : commanded_positions) {
        packFloat(packet, angle);
    }
    
    // Add checksum
    packet.push_back(calculateChecksum(packet));
    
    has_new_commands = false;
    return packet;
}

void ServoHandler::publishJointStates(const rclcpp::Time& timestamp) {
    JointState msg;
    msg.header.stamp = timestamp;
    
    msg.name = {
        "leg1_coxa", "leg1_femur", "leg1_tibia",
        "leg2_coxa", "leg2_femur", "leg2_tibia",
        "leg3_coxa", "leg3_femur", "leg3_tibia",
        "leg4_coxa", "leg4_femur", "leg4_tibia",
        "leg5_coxa", "leg5_femur", "leg5_tibia",
        "leg6_coxa", "leg6_femur", "leg6_tibia"
    };
    
    msg.position.assign(current_positions.begin(), current_positions.end());
    
    joint_state_pub->publish(msg);
}

void ServoHandler::simulateMovement(double alpha) {
    // Gradually move current positions toward commanded positions
    for (size_t i = 0; i < 18; i++) {
        double error = commanded_positions[i] - current_positions[i];
        current_positions[i] += error * alpha;
    }
}

void ServoHandler::packFloat(std::vector<uint8_t>& buffer, float value) {
    uint8_t bytes[4];
    std::memcpy(bytes, &value, sizeof(float));
    buffer.insert(buffer.end(), bytes, bytes + 4);
}

uint8_t ServoHandler::calculateChecksum(const std::vector<uint8_t>& data) {
    uint8_t sum = 0;
    for (uint8_t byte : data) {
        sum ^= byte;  // XOR checksum
    }
    return sum;
}

} // namespace hexapod_hardware_interface