#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

using geometry_msgs::msg::Twist;
using sensor_msgs::msg::Joy;
using std::placeholders::_1;

class ControllerTeleop : public rclcpp::Node {public: ControllerTeleop() : Node("controller_teleop") {
  	RCLCPP_INFO(this->get_logger(), "Controller Teleop Node has been started.");
    cmd_vel_pub = this->create_publisher<Twist>("cmd_vel", 10);
    joy_sub = this->create_subscription<Joy>("joy", rclcpp::SensorDataQoS(), std::bind(&ControllerTeleop::joyCallback, this, _1));
}
private:
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<Joy>::SharedPtr joy_sub;
public:
    void joyCallback(const Joy::SharedPtr msg);
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

void ControllerTeleop::joyCallback(const Joy::SharedPtr msg) {
    Twist cmd_vel_msg;
    // Assuming axes[0] is left stick horizontal, axes[1] is left stick vertical, and axes[3] is right stick horizontal
    cmd_vel_msg.linear.x = msg->axes[1];  // Forward/backward
    cmd_vel_msg.linear.y = msg->axes[0];  // Left/right
    cmd_vel_msg.angular.z = msg->axes[3]; // Rotation

    RCLCPP_INFO(this->get_logger(), "Received joystick input: axes[0]=%.2f, axes[1]=%.2f, axes[3]=%.2f",
                msg->axes[0], msg->axes[1], msg->axes[3]);

    RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x=%.2f, linear.y=%.2f, angular.z=%.2f",
                cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z);

    cmd_vel_pub->publish(cmd_vel_msg);
}