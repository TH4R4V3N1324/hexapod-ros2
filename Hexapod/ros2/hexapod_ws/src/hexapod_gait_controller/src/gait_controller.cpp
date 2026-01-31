#include "rclcpp/rclcpp.hpp"
#include "hexapod_gait_controller/kinematic_solver.hpp"
#include "hexapod_gait_controller/trajectory_generator.hpp"
#include "Eigen/Dense"

using Eigen::Vector3d;

enum Gait : uint8_t {
    GAIT_TRIPOD,
    GAIT_RIPPLE,
    GAIT_WAVE,
    NUM_GAITS
};

class GaitController : public rclcpp::Node { public: GaitController() : Node("gait_controller") {
    kinematic_service = std::make_unique<hexapod_gait_controller::KinematicSolverService>(this);
    RCLCPP_INFO(this->get_logger(), "GaitController node has been started.");
}    
private:
    std::unique_ptr<hexapod_gait_controller::KinematicSolverService> kinematic_service;
    std::vector<std::vector<int>> getGaitConfig(Gait gait);
};

int main(int argc, char **argv){    
    rclcpp::init(argc, argv);    
    auto node = std::make_shared<GaitController>();    
    rclcpp::spin(node);    
    rclcpp::shutdown();    
    return 0;
}

/*
@brief Get the leg configuration based on the specified gait
@param gait The gait type
@return A vector of vectors representing the leg configuration
*/
std::vector<std::vector<int>> GaitController::getGaitConfig(Gait gait){
    switch (gait){
        case GAIT_TRIPOD:
            return {{1, 3, 5}, {2, 4, 6}};
            break;
        case GAIT_RIPPLE:
            return {{3, 6}, {2, 4}, {1, 5}};
            break;
        case GAIT_WAVE:
            return {{3}, {2}, {1}, {4}, {5}, {6}};
            break;
        default:
            return {};
            break;
    }
}