#include "rclcpp/rclcpp.hpp"
#include "hexapod_hardware_interface/sensor_handler.hpp"
#include "hexapod_hardware_interface/servo_handler.hpp"

using namespace hexapod_hardware_interface;

class HardwareInterface : public rclcpp::Node {public: HardwareInterface() : Node("hardware_interface") {
        // Declare parameters
        declare_parameter("publish_rate", 100.0);
        double rate = get_parameter("publish_rate").as_double();
        
        // Create handlers
        sensor_handler = std::make_unique<SensorHandler>(this);
        servo_handler = std::make_unique<ServoHandler>(this);
        
        RCLCPP_INFO(get_logger(), "Hardware interface started (%.1f Hz)", rate);
    }

private:
    std::unique_ptr<SensorHandler> sensor_handler;
    std::unique_ptr<ServoHandler> servo_handler;  
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardwareInterface>());
    rclcpp::shutdown();
    return 0;
}