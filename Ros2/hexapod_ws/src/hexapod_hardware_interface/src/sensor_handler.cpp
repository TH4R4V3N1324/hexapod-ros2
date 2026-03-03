#include "hexapod_hardware_interface/sensor_handler.hpp"
#include <cstring>
#include <random>

namespace hexapod_hardware_interface {

SensorHandler::SensorHandler(rclcpp::Node* node) : node(node) {
    imu_pub = node->create_publisher<Imu>("imu_data", 10);
    power_pub = node->create_publisher<PowerStatus>("power_status_data", 10);
    contact_pub = node->create_publisher<LegContact>("leg_contact_data", 10);
    
    RCLCPP_INFO(node->get_logger(), "SensorHandler initialized");
}

void SensorHandler::parseAndPublish(const std::vector<uint8_t>& data, const rclcpp::Time& timestamp) {
    // Protocol from ESP32:
    // [0-1]: header (0xBB + length)
    // [2-5]: pitch (float)
    // [6-9]: roll (float)
    // [10-13]: current (float)
    // [14-17]: voltage (float)
    // [18]: leg contacts (6 bits)
    
    if (data.size() < 19) {
        RCLCPP_WARN(node->get_logger(), "Sensor data packet too short");
        return;
    }
    
    parseIMU(&data[2], timestamp);
    parsePower(&data[10], timestamp);
    parseContacts(&data[18], timestamp);
}

void SensorHandler::parseIMU(const uint8_t* data, const rclcpp::Time& timestamp) {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = timestamp;
    msg.header.frame_id = "imu_link";
    
    float pitch = unpackFloat(&data[0]);
    float roll = unpackFloat(&data[4]);
    
    // Convert Euler angles to quaternion
    double cy = cos(0.0 * 0.5);  // yaw = 0
    double sy = sin(0.0 * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    
    msg.orientation.w = cr * cp * cy + sr * sp * sy;
    msg.orientation.x = sr * cp * cy - cr * sp * sy;
    msg.orientation.y = cr * sp * cy + sr * cp * sy;
    msg.orientation.z = cr * cp * sy - sr * sp * cy;
    
    imu_pub->publish(msg);
}

void SensorHandler::parsePower(const uint8_t* data, const rclcpp::Time& timestamp) {
    hexapod_interfaces::msg::PowerStatus msg;
    
    msg.current = unpackFloat(&data[0]);
    msg.voltage = unpackFloat(&data[4]);
    msg.power = msg.voltage * msg.current;
    
    // Battery percentage (2S LiPo: 8.4V full, 6.0V empty)
    msg.battery_percent = std::clamp((msg.voltage - 6.0f) / 2.4f * 100.0f, 0.0f, 100.0f);
    msg.low_battery_warning = msg.battery_percent < 20.0f;
    
    power_pub->publish(msg);
}

void SensorHandler::parseContacts(const uint8_t* data, const rclcpp::Time& timestamp) {
    hexapod_interfaces::msg::LegContact msg;
    msg.header.stamp = timestamp;
    
    uint8_t contact_byte = data[0];
    for (int i = 0; i < 6; i++) {
        msg.contacts[i] = (contact_byte >> i) & 0x01;
    }
    
    contact_pub->publish(msg);
}

float SensorHandler::unpackFloat(const uint8_t* bytes) {
    float value;
    std::memcpy(&value, bytes, sizeof(float));
    return value;
}

void SensorHandler::publishFakeData(const rclcpp::Time& timestamp) {
    // Fake IMU
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = timestamp;
    imu_msg.header.frame_id = "imu_link";
    imu_msg.orientation.w = 1.0;
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.linear_acceleration.z = 9.81;
    imu_pub->publish(imu_msg);
    
    // Fake power
    static double battery = 8.4;
    battery -= 0.00001;
    if (battery < 6.0) battery = 8.4;
    
    hexapod_interfaces::msg::PowerStatus power_msg;
    power_msg.voltage = battery;
    power_msg.current = 0.5;
    power_msg.power = power_msg.voltage * power_msg.current;
    power_msg.battery_percent = (battery - 6.0) / 2.4 * 100.0;
    power_msg.low_battery_warning = power_msg.battery_percent < 20.0;
    power_pub->publish(power_msg);
    
    // Fake contacts (all legs on ground)
    hexapod_interfaces::msg::LegContact contact_msg;
    contact_msg.header.stamp = timestamp;
    contact_msg.contacts = {true, true, true, true, true, true};
    contact_pub->publish(contact_msg);
}

} // namespace hexapod_hardware_interface