#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "hexapod_interfaces/srv/get_capabilities.hpp"
#include "hexapod_interfaces/srv/set_gait.hpp"
#include "hexapod_interfaces/srv/set_mode.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <algorithm>
#include <cmath>

using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Pose;
using sensor_msgs::msg::Joy;
using hexapod_interfaces::srv::GetCapabilities;
using hexapod_interfaces::srv::SetGait;
using hexapod_interfaces::srv::SetMode;
using std_srvs::srv::SetBool;
using std::placeholders::_1;

/**
 * @brief Gamepad teleop node, converts /joy input into /cmd_vel and /body_pose commands
 * @note Axis and button numbers are parameters, defaulting to an Xbox controller on joy_node
*/
class ControllerTeleop : public rclcpp::Node {public: ControllerTeleop() : Node("controller_teleop") {
  	RCLCPP_INFO(this->get_logger(), "Controller Teleop Node has been started.");
    cmd_vel_pub = this->create_publisher<Twist>("cmd_vel", 10);
    body_pose_pub = this->create_publisher<Pose>("body_pose", 10);
    joy_sub = this->create_subscription<Joy>("joy", rclcpp::SensorDataQoS(), std::bind(&ControllerTeleop::joyCallback, this, _1));
    get_capabilities_client = this->create_client<GetCapabilities>("get_capabilities");
    set_mode_client = this->create_client<SetMode>("set_mode");
    set_gait_client = this->create_client<SetGait>("set_gait");

    axis_left_x = this->declare_parameter("axis_left_x", 0);
    axis_left_y = this->declare_parameter("axis_left_y", 1);
    axis_right_x = this->declare_parameter("axis_right_x", 3);
    axis_right_y = this->declare_parameter("axis_right_y", 4);
    axis_left_trigger = this->declare_parameter("axis_left_trigger", 2);
    axis_right_trigger = this->declare_parameter("axis_right_trigger", 5);
    axis_dpad_y = this->declare_parameter("axis_dpad_y", 7);
    button_mode = this->declare_parameter("button_mode", 7);  // Start
    button_gait = this->declare_parameter("button_gait", 6);  // Back

    // Face buttons switch gait controller features on and off
    gait_controller_params = std::make_shared<rclcpp::AsyncParametersClient>(this, "hexapod_gait_controller");
    for (const auto& [feature, button] : std::vector<std::pair<std::string, int>>{
             {"terrain_adaptation", 0}, {"walk_leveling", 1}, {"load_balancing", 2}, {"tilt_leveling", 3}}) {
        toggles.push_back({feature, static_cast<int>(this->declare_parameter("button_" + feature, button)),
                           this->create_client<SetBool>("set_" + feature)});
    }

    // Keep asking until the gait controller answers, without blocking the executor
    capabilities_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ControllerTeleop::fetchCapabilities, this));
    fetchCapabilities();
}
private:
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<Pose>::SharedPtr body_pose_pub;
    rclcpp::Subscription<Joy>::SharedPtr joy_sub;
    rclcpp::Client<GetCapabilities>::SharedPtr get_capabilities_client;
    rclcpp::Client<SetMode>::SharedPtr set_mode_client;
    rclcpp::Client<SetGait>::SharedPtr set_gait_client;
    rclcpp::TimerBase::SharedPtr capabilities_timer;
    std::vector<hexapod_interfaces::msg::LocomotionOption> available_gaits;
	std::vector<hexapod_interfaces::msg::LocomotionOption> available_modes;
    size_t current_gait_index = 0;
    size_t current_mode_index = 0;
    double max_linear_vel = 0.0;        // m/s, full stick
    double max_angular_vel = 0.0;       // rad/s, full stick
    double max_tilt_angle = 0.0;        // rad, full stick in tilt mode
    double max_tilt_translation = 0.0;  // m, full stick in tilt mode
    bool capabilities_received = false;
    bool capabilities_pending = false;

    int axis_left_x, axis_left_y, axis_right_x, axis_right_y;
    int axis_left_trigger, axis_right_trigger, axis_dpad_y;
    int button_mode, button_gait;
    struct Toggle {
        std::string feature;  // gait controller parameter, also its set_<feature> service
        int button;
        rclcpp::Client<SetBool>::SharedPtr client;
    };
    std::vector<Toggle> toggles;
    std::shared_ptr<rclcpp::AsyncParametersClient> gait_controller_params;
    std::vector<int> previous_buttons;
    bool left_trigger_seen = false;   // joy_node reports 0 for a trigger until it is first moved
    bool right_trigger_seen = false;
public:
    void joyCallback(const Joy::SharedPtr msg);
    void fetchCapabilities();
    void cycleMode();
    void cycleGait();
    void toggleFeature(const Toggle& toggle);
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
 * @brief Convert joystick input into velocity and body pose commands, and handle the buttons
 * @param[in] msg The incoming joystick message
 * @note Walking: left stick drives linear X/Y, right stick X drives angular Z.
 *       Tilt mode (the gait controller ignores /cmd_vel there): left stick shifts the body,
 *       right stick rolls and pitches it, the triggers yaw it and the d-pad raises and lowers it.
 *       Both are always published, and the gait controller uses whichever suits its mode.
 *       Start cycles the mode and Back cycles the gait; A, B, X and Y switch terrain adaptation,
 *       walk leveling, load balancing and tilt leveling on and off. Full stick is the gait controller's
 *       limit, so nothing is published until its capabilities have been received.
*/
void ControllerTeleop::joyCallback(const Joy::SharedPtr msg) {
    if (!capabilities_received) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for gait controller capabilities, ignoring joystick");
        return;
    }

    auto axis = [&msg](int index) {
        if (index < 0 || static_cast<std::size_t>(index) >= msg->axes.size()) return 0.0;
        return std::clamp(static_cast<double>(msg->axes[index]), -1.0, 1.0);
    };
    auto pressed = [&msg, this](int index) {
        if (index < 0 || static_cast<std::size_t>(index) >= msg->buttons.size()) return false;
        bool was = static_cast<std::size_t>(index) < previous_buttons.size() && previous_buttons[index];
        return msg->buttons[index] && !was;
    };

    if (pressed(button_mode)) cycleMode();
    if (pressed(button_gait)) cycleGait();
    for (const auto& toggle : toggles) {
        if (pressed(toggle.button)) toggleFeature(toggle);
    }
    previous_buttons = msg->buttons;

    Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = axis(axis_left_y) * max_linear_vel;
    cmd_vel_msg.linear.y = axis(axis_left_x) * max_linear_vel;
    cmd_vel_msg.angular.z = axis(axis_right_x) * max_angular_vel;
    cmd_vel_pub->publish(cmd_vel_msg);

    // Triggers rest at 1 and read -1 fully pressed
    auto trigger = [&axis](int index, bool& seen) {
        double value = axis(index);
        seen = seen || value != 0.0;
        return seen ? (1.0 - value) / 2.0 : 0.0;
    };
    const double roll = -axis(axis_right_x) * max_tilt_angle;   // stick right lowers the right side
    const double pitch = axis(axis_right_y) * max_tilt_angle;   // stick forward lowers the nose
    const double yaw = (trigger(axis_left_trigger, left_trigger_seen) - trigger(axis_right_trigger, right_trigger_seen)) * max_tilt_angle;

    Pose body_pose_msg;
    body_pose_msg.position.x = axis(axis_left_y) * max_tilt_translation;
    body_pose_msg.position.y = axis(axis_left_x) * max_tilt_translation;
    body_pose_msg.position.z = axis(axis_dpad_y) * max_tilt_translation;
    // Roll, pitch, yaw to a quaternion
    const double cr = std::cos(roll / 2.0), sr = std::sin(roll / 2.0);
    const double cp = std::cos(pitch / 2.0), sp = std::sin(pitch / 2.0);
    const double cy = std::cos(yaw / 2.0), sy = std::sin(yaw / 2.0);
    body_pose_msg.orientation.w = cr * cp * cy + sr * sp * sy;
    body_pose_msg.orientation.x = sr * cp * cy - cr * sp * sy;
    body_pose_msg.orientation.y = cr * sp * cy + sr * cp * sy;
    body_pose_msg.orientation.z = cr * cp * sy - sr * sp * cy;
    body_pose_pub->publish(body_pose_msg);
}

/**
 * @brief Switch the gait controller to the next mode
*/
void ControllerTeleop::cycleMode() {
    if (available_modes.empty() || !set_mode_client->service_is_ready()) return;
    current_mode_index = (current_mode_index + 1) % available_modes.size();
    const auto mode = available_modes[current_mode_index];
    auto request = std::make_shared<SetMode::Request>();
    request->mode = mode.id;
    set_mode_client->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Mode: %s", mode.name.c_str());
}

/**
 * @brief Switch a gait controller feature to the opposite of its current state
 * @param[in] toggle The feature
 * @note Reads the current state from the gait controller first, so a change made some other way
 *       (a service call or ros2 param) is never undone by mistake
*/
void ControllerTeleop::toggleFeature(const Toggle& toggle) {
    if (!gait_controller_params->service_is_ready() || !toggle.client->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "Can't switch %s: the gait controller isn't answering", toggle.feature.c_str());
        return;
    }
    auto client = toggle.client;
    const std::string feature = toggle.feature;
    gait_controller_params->get_parameters({feature},
        [this, client, feature](std::shared_future<std::vector<rclcpp::Parameter>> future) {
            const auto params = future.get();
            if (params.empty() || params[0].get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                RCLCPP_WARN(this->get_logger(), "Can't read %s from the gait controller", feature.c_str());
                return;
            }
            auto request = std::make_shared<SetBool::Request>();
            request->data = !params[0].as_bool();
            client->async_send_request(request, [this](rclcpp::Client<SetBool>::SharedFuture result) {
                RCLCPP_INFO(this->get_logger(), "%s", result.get()->message.c_str());
            });
        });
}

/**
 * @brief Switch the gait controller to the next gait, which it applies at its next step
*/
void ControllerTeleop::cycleGait() {
    if (available_gaits.empty() || !set_gait_client->service_is_ready()) return;
    current_gait_index = (current_gait_index + 1) % available_gaits.size();
    const auto gait = available_gaits[current_gait_index];
    auto request = std::make_shared<SetGait::Request>();
    request->gait = gait.id;
    set_gait_client->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Gait: %s", gait.name.c_str());
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
            max_tilt_angle = response->max_tilt_angle;
            max_tilt_translation = response->max_tilt_translation;
            for (size_t i = 0; i < available_modes.size(); ++i) {
                if (available_modes[i].id == response->current_mode) current_mode_index = i;
            }
            for (size_t i = 0; i < available_gaits.size(); ++i) {
                if (available_gaits[i].id == response->current_gait) current_gait_index = i;
            }
            capabilities_received = true;

            RCLCPP_INFO(this->get_logger(), "Received capabilities: %zu gaits, %zu modes",
                        response->gaits.size(), response->modes.size());
            RCLCPP_INFO(this->get_logger(), "Full stick: linear=%.2f m/s, angular=%.2f rad/s",
                        max_linear_vel, max_angular_vel);
        });
}
