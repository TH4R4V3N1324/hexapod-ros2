#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "hexapod_interfaces/srv/get_capabilities.hpp"
#include <algorithm>

using geometry_msgs::msg::Twist;
using sensor_msgs::msg::Joy;
using hexapod_interfaces::srv::GetCapabilities;
using std::placeholders::_1;

/**
 * @brief Gamepad teleop node, converts /joy input into /cmd_vel commands
*/
class ControllerTeleop : public rclcpp::Node {public: ControllerTeleop() : Node("controller_teleop") {
  	RCLCPP_INFO(this->get_logger(), "Controller Teleop Node has been started.");
    cmd_vel_pub = this->create_publisher<Twist>("cmd_vel", 10);
    joy_sub = this->create_subscription<Joy>("joy", rclcpp::SensorDataQoS(), std::bind(&ControllerTeleop::joyCallback, this, _1));
    get_capabilities_client = this->create_client<GetCapabilities>("get_capabilities");

    // Keep asking until the gait controller answers, without blocking the executor
    capabilities_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ControllerTeleop::fetchCapabilities, this));
    fetchCapabilities();
}
private:
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<Joy>::SharedPtr joy_sub;
    rclcpp::Client<GetCapabilities>::SharedPtr get_capabilities_client;
    rclcpp::TimerBase::SharedPtr capabilities_timer;
    std::vector<hexapod_interfaces::msg::LocomotionOption> available_gaits;
	std::vector<hexapod_interfaces::msg::LocomotionOption> available_modes;
    double max_linear_vel = 0.0;   // m/s, full stick
    double max_angular_vel = 0.0;  // rad/s, full stick
    bool capabilities_received = false;
    bool capabilities_pending = false;
public:
    void joyCallback(const Joy::SharedPtr msg);
    void fetchCapabilities();
};

/**
 * @brief Entry point, initialises ROS and spins the node
 * @param[in] argc Number of command line arguments
 * @param[in] argv Command line arguments
 * @return Process exit code
*/
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

/**
 * @brief Convert joystick axes into a velocity command and publish it
 * @param[in] msg The incoming joystick message
 * @note Left stick drives linear X/Y, right stick X drives angular Z.
 *       Full stick is the gait controller's top speed, so nothing is published
 *       until its capabilities have been received.
*/
void ControllerTeleop::joyCallback(const Joy::SharedPtr msg) {
    constexpr std::size_t kAxisLinearY = 0;   // Left/right
    constexpr std::size_t kAxisLinearX = 1;   // Forward/backward
    constexpr std::size_t kAxisAngularZ = 3;  // Right stick left/right

    const std::size_t required_axes = std::max({kAxisLinearY, kAxisLinearX, kAxisAngularZ}) + 1;
    if (msg->axes.size() < required_axes) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Joy message has %zu axes but at least %zu are required",
            msg->axes.size(),
            required_axes);
        return;
    }
    if (!capabilities_received) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for gait controller capabilities, ignoring joystick");
        return;
    }

    const double raw_linear_x = std::clamp(static_cast<double>(msg->axes[kAxisLinearX]), -1.0, 1.0);
    const double raw_linear_y = std::clamp(static_cast<double>(msg->axes[kAxisLinearY]), -1.0, 1.0);
    const double raw_angular_z = std::clamp(static_cast<double>(msg->axes[kAxisAngularZ]), -1.0, 1.0);

    Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = raw_linear_x * max_linear_vel;
    cmd_vel_msg.linear.y = raw_linear_y * max_linear_vel;
    cmd_vel_msg.angular.z = raw_angular_z * max_angular_vel;

    /* RCLCPP_INFO(this->get_logger(), "Received joystick input: axes[0]=%.2f, axes[1]=%.2f, axes[3]=%.2f",
                raw_linear_y, raw_linear_x, raw_angular_z);

    RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x=%.2f, linear.y=%.2f, angular.z=%.2f",
                cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z); */

    cmd_vel_pub->publish(cmd_vel_msg);
}

/**
 * @brief Request the supported gaits, modes and velocity limits from the gait controller
 * @note Called once a second until a reply arrives. The request is asynchronous,
 *       so the node keeps spinning while it waits.
*/
void ControllerTeleop::fetchCapabilities() {
    if (capabilities_received) {
        capabilities_timer->cancel();
        return;
    }
    if (capabilities_pending) {
        return;
    }
    if (!get_capabilities_client->service_is_ready()) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for the get_capabilities service");
        return;
    }

    capabilities_pending = true;
    auto request = std::make_shared<GetCapabilities::Request>();
    get_capabilities_client->async_send_request(request,
        [this](rclcpp::Client<GetCapabilities>::SharedFuture future) {
            capabilities_pending = false;
            auto response = future.get();
            available_gaits = response->gaits;
            available_modes = response->modes;
            max_linear_vel = response->max_linear_vel;
            max_angular_vel = response->max_angular_vel;
            capabilities_received = true;

            RCLCPP_INFO(this->get_logger(), "Received capabilities: %zu gaits, %zu modes",
                        response->gaits.size(), response->modes.size());
            RCLCPP_INFO(this->get_logger(), "Full stick: linear=%.2f m/s, angular=%.2f rad/s",
                        max_linear_vel, max_angular_vel);
        });
}
