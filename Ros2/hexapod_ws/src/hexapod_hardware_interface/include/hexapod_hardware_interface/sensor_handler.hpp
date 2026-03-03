#ifndef HEXAPOD_HARDWARE_INTERFACE__SENSOR_HANDLER_HPP_
#define HEXAPOD_HARDWARE_INTERFACE__SENSOR_HANDLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <hexapod_interfaces/msg/power_status.hpp>
#include <hexapod_interfaces/msg/leg_contact.hpp>
#include <vector>

using hexapod_interfaces::msg::LegContact;
using hexapod_interfaces::msg::PowerStatus;
using sensor_msgs::msg::Imu;

namespace hexapod_hardware_interface {

class SensorHandler {
public:
    explicit SensorHandler(rclcpp::Node* node);
    void parseAndPublish(const std::vector<uint8_t>& data, const rclcpp::Time& timestamp);
    void publishFakeData(const rclcpp::Time& timestamp);
private:
    void parseIMU(const uint8_t* data, const rclcpp::Time& timestamp);
    void parsePower(const uint8_t* data, const rclcpp::Time& timestamp);
    void parseContacts(const uint8_t* data, const rclcpp::Time& timestamp);
    
    float unpackFloat(const uint8_t* bytes);
    
    rclcpp::Publisher<Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<PowerStatus>::SharedPtr power_pub;
    rclcpp::Publisher<LegContact>::SharedPtr contact_pub;
    
    rclcpp::Node* node;
};

} // namespace hexapod_hardware_interface

#endif