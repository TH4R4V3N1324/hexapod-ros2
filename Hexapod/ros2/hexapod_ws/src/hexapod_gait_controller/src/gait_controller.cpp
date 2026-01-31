#include "rclcpp/rclcpp.hpp"
#include "hexapod_gait_controller/trajectory_generator.hpp"
#include "hexapod_gait_controller/kinematic_solver.hpp"

class GaitController : public rclcpp::Node { public: GaitController() : Node("gait_controller") {
    kinematic_service = std::make_unique<hexapod_gait_controller::KinematicSolverService>(this);
    RCLCPP_INFO(this->get_logger(), "GaitController node has been started.");
}    
private:
    std::unique_ptr<hexapod_gait_controller::KinematicSolverService> kinematic_service;
};

int main(int argc, char **argv){    
    rclcpp::init(argc, argv);    
    auto node = std::make_shared<GaitController>();    
    rclcpp::spin(node);    
    rclcpp::shutdown();    
    return 0;
}