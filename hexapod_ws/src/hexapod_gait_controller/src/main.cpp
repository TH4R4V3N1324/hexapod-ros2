#include "hexapod_gait_controller/gait_controller.hpp"

/**
 * @brief Entry point, initialises ROS and spins the node
 * @param[in] argc Number of command line arguments
 * @param[in] argv Command line arguments
 * @return Process exit code
*/
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GaitController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
