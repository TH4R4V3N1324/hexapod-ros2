#include <rclcpp/rclcpp.hpp>
#include "hexapod_hardware_interface/sensor_handler.hpp"
#include "hexapod_hardware_interface/servo_handler.hpp"
#include "hexapod_hardware_interface/controller_handler.hpp"

using namespace hexapod_hardware_interface;

class HardwareInterfaceFake : public rclcpp::Node {
public:
    HardwareInterfaceFake() : Node("hardware_interface_fake") {
        // Declare parameters
        declare_parameter("publish_rate", 100.0);
        double rate = get_parameter("publish_rate").as_double();
        
        // Create handlers
        sensor_handler = std::make_unique<SensorHandler>(this);
        servo_handler = std::make_unique<ServoHandler>(this);
        controller_handler = std::make_unique<ControllerHandler>(this);
        
        // Timer for main loop
        auto period = std::chrono::duration<double>(1.0 / rate);
        timer = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period), std::bind(&HardwareInterfaceFake::mainLoop, this));
        
        RCLCPP_INFO(get_logger(), "Fake hardware interface started (%.1f Hz)", rate);
    }

private:
    void mainLoop() {
        auto timestamp = now();
        
        // Simulate servo movement toward commanded positions
        servo_handler->simulateMovement(0.3);
        
        // Publish all feedback
        servo_handler->publishJointStates(timestamp);
        sensor_handler->publishFakeData(timestamp);
        
        // Publish controller input occasionally (to avoid spam)
        static int counter = 0;
        if (++counter % 50 == 0) {  // 2Hz
            controller_handler->publishFakeData(timestamp);
        }
    }
    
    std::unique_ptr<SensorHandler> sensor_handler;
    std::unique_ptr<ServoHandler> servo_handler;
    std::unique_ptr<ControllerHandler> controller_handler;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardwareInterfaceFake>());
    rclcpp::shutdown();
    return 0;
}