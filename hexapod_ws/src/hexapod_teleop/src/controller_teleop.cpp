#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "hexapod_interfaces/srv/get_capabilities.hpp"
#include <algorithm>

using geometry_msgs::msg::Twist;
using sensor_msgs::msg::Joy;
using hexapod_interfaces::srv::GetCapabilities;
using std::placeholders::_1;

class ControllerTeleop : public rclcpp::Node {public: ControllerTeleop() : Node("controller_teleop") {
  	RCLCPP_INFO(this->get_logger(), "Controller Teleop Node has been started.");
    cmd_vel_pub = this->create_publisher<Twist>("cmd_vel", 10);
    joy_sub = this->create_subscription<Joy>("joy", rclcpp::SensorDataQoS(), std::bind(&ControllerTeleop::joyCallback, this, _1));
    get_capabilities_client = this->create_client<GetCapabilities>("get_capabilities");

    // Safe defaults until capabilities are fetched.
    max_linear_vel = 1.0;
    max_angular_vel = 1.0;

    fetchCapabilities();
}
private:
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<Joy>::SharedPtr joy_sub;
    rclcpp::Client<GetCapabilities>::SharedPtr get_capabilities_client;
    std::vector<hexapod_interfaces::msg::LocomotionOption> available_gaits;
	std::vector<hexapod_interfaces::msg::LocomotionOption> available_modes;
    double max_linear_vel;
    double max_angular_vel;
public:
    void joyCallback(const Joy::SharedPtr msg);
    void fetchCapabilities();
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

void ControllerTeleop::joyCallback(const Joy::SharedPtr msg) {
    constexpr std::size_t kAxisLinearY = 0;   // Left/right
    constexpr std::size_t kAxisLinearX = 1;   // Forward/backward
    constexpr std::size_t kAxisAngularZ = 2;  // Rotation

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

void ControllerTeleop::fetchCapabilities() {
    if (!get_capabilities_client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_WARN(this->get_logger(), "GetCapabilities service not available");
        return;
    }

    auto request = std::make_shared<GetCapabilities::Request>();
    auto result_future = get_capabilities_client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result_future.get();

        available_gaits = response->gaits;
        available_modes = response->modes;
        max_linear_vel = response->max_linear_vel;
        max_angular_vel = response->max_angular_vel;

        RCLCPP_INFO(this->get_logger(), "Received capabilities: %zu gaits, %zu modes",
                    response->gaits.size(), response->modes.size());
        RCLCPP_INFO(this->get_logger(), "Max velocities: linear=%.2f m/s, angular=%.2f rad/s",
                    max_linear_vel, max_angular_vel);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call GetCapabilities service");
    }
}