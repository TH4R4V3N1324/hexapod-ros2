#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "hexapod_interfaces/srv/get_capabilities.hpp"
#include "hexapod_interfaces/srv/set_gait.hpp"
#include "hexapod_interfaces/srv/set_height.hpp"
#include "hexapod_interfaces/srv/set_mode.hpp"
#include "hexapod_teleop/gamepad_teleop.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>

using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Pose;
using sensor_msgs::msg::Joy;
using hexapod_interfaces::srv::GetCapabilities;
using hexapod_interfaces::srv::SetGait;
using hexapod_interfaces::srv::SetHeight;
using hexapod_interfaces::srv::SetMode;
using std_srvs::srv::SetBool;
using std_srvs::srv::Trigger;
using std::placeholders::_1;

/**
 * @brief Gamepad teleop node, converts /joy input into /cmd_vel and /body_pose commands
 * @note The layout itself is hexapod_teleop::GamepadTeleop from gait-core, shared with the
 *       microcontroller stacks' receiver so a gamepad drives every robot the same way. This node
 *       turns joy messages into its GamepadState, and its output into topics and service calls.
 *       Axis and button numbers are parameters, defaulting to the SDL game controller layout
 *       that joy_node reports.
*/
class ControllerTeleop : public rclcpp::Node {public: ControllerTeleop() : Node("controller_teleop") {
  	RCLCPP_INFO(this->get_logger(), "Controller Teleop Node has been started.");
    cmd_vel_pub = this->create_publisher<Twist>("cmd_vel", 10);
    body_pose_pub = this->create_publisher<Pose>("body_pose", 10);
    joy_sub = this->create_subscription<Joy>("joy", rclcpp::SensorDataQoS(), std::bind(&ControllerTeleop::joyCallback, this, _1));
    get_capabilities_client = this->create_client<GetCapabilities>("get_capabilities");
    set_mode_client = this->create_client<SetMode>("set_mode");
    set_gait_client = this->create_client<SetGait>("set_gait");
    set_height_client = this->create_client<SetHeight>("set_height");
    sit_client = this->create_client<SetBool>("sit");
    power_down_client = this->create_client<Trigger>("power_down");
    // Latched by the gait controller, so this hears the current state as soon as it connects
    seated_sub = this->create_subscription<std_msgs::msg::Bool>("seated", rclcpp::QoS(1).transient_local(),
        std::bind(&ControllerTeleop::seatedCallback, this, _1));

    // Defaults follow the SDL game controller layout that ROS 2's joy_node reports for any supported
    // gamepad (Xbox, PlayStation, ...). They used to follow the raw Xbox order, which put pitch on a
    // trigger (axis 4), so the robot pitched as soon as tilt mode started.
    axis_left_x = this->declare_parameter("axis_left_x", 0);
    axis_left_y = this->declare_parameter("axis_left_y", 1);
    axis_right_x = this->declare_parameter("axis_right_x", 2);
    axis_right_y = this->declare_parameter("axis_right_y", 3);
    axis_left_trigger = this->declare_parameter("axis_left_trigger", 4);
    axis_right_trigger = this->declare_parameter("axis_right_trigger", 5);
    axis_dpad_y = this->declare_parameter("axis_dpad_y", -1);        // -1: the d-pad is buttons
    button_dpad_up = this->declare_parameter("button_dpad_up", 11);
    button_dpad_down = this->declare_parameter("button_dpad_down", 12);
    button_mode = this->declare_parameter("button_mode", 6);  // Start
    button_gait = this->declare_parameter("button_gait", 4);  // Back
    button_sit = this->declare_parameter("button_sit", 10);   // RB [R1]: sit / stand, hold to power down
    height_step = this->declare_parameter("height_step", 0.01);         // m per d-pad step
    hexapod_teleop::TeleopParams teleop_params;
    teleop_params.powerDownHold = this->declare_parameter("power_down_hold", teleop_params.powerDownHold);  // s
    teleop_params.heightRepeat = this->declare_parameter("height_repeat", teleop_params.heightRepeat);      // s between steps while held
    teleop = hexapod_teleop::GamepadTeleop(teleop_params);

    // Face buttons switch gait controller features on and off, in GamepadTeleop's order (A, B, X, Y)
    gait_controller_params = std::make_shared<rclcpp::AsyncParametersClient>(this, "hexapod_gait_controller");
    for (int i = 0; i < hexapod_teleop::NUM_FEATURES; ++i) {
        const std::string feature = hexapod_teleop::featureName(static_cast<hexapod_teleop::Feature>(i));
        toggles[i] = {feature, static_cast<int>(this->declare_parameter("button_" + feature, i)),
                      this->create_client<SetBool>("set_" + feature)};
    }

    // Keep asking until the gait controller answers, without blocking the executor
    capabilities_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ControllerTeleop::fetchCapabilities, this));
    fetchCapabilities();
    // Runs the layout between joy messages too, so the power-down hold and the height repeat fire on
    // time even without joy_node's autorepeat
    tick_timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ControllerTeleop::tick, this));
}

private:
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<Pose>::SharedPtr body_pose_pub;
    rclcpp::Subscription<Joy>::SharedPtr joy_sub;
    rclcpp::Client<GetCapabilities>::SharedPtr get_capabilities_client;
    rclcpp::Client<SetMode>::SharedPtr set_mode_client;
    rclcpp::Client<SetGait>::SharedPtr set_gait_client;
    rclcpp::Client<SetHeight>::SharedPtr set_height_client;
    rclcpp::Client<SetBool>::SharedPtr sit_client;
    rclcpp::Client<Trigger>::SharedPtr power_down_client;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr seated_sub;
    rclcpp::TimerBase::SharedPtr capabilities_timer;
    rclcpp::TimerBase::SharedPtr tick_timer;

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
    int button_dpad_up, button_dpad_down;
    int button_mode, button_gait, button_sit;
    double height_step;

    hexapod_teleop::GamepadTeleop teleop;
    std::optional<hexapod_teleop::GamepadState> last_pad;  // the latest joy message, for tick()
    bool seated = true;  // the robot starts seated; the gait controller's /seated says otherwise
    bool height_pending = false;
    std::optional<double> height_target;    // m, the last height this asked for

    struct Toggle {
        std::string feature;  // gait controller parameter, also its set_<feature> service
        int button;
        rclcpp::Client<SetBool>::SharedPtr client;
    };
    Toggle toggles[hexapod_teleop::NUM_FEATURES];
    std::shared_ptr<rclcpp::AsyncParametersClient> gait_controller_params;

    std::optional<double> left_trigger_rest;   // a released trigger's reading, taken from the first message
    std::optional<double> right_trigger_rest;

public:
    void joyCallback(const Joy::SharedPtr msg);
    void seatedCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void tick();
    void handleEvents(const hexapod_teleop::TeleopOutput& out);
    hexapod_teleop::RobotState robotState() const;
    void fetchCapabilities();
    void requestCapabilities(std::function<void(const GetCapabilities::Response&)> then);
    void applyCapabilities(const GetCapabilities::Response& response);
    void cycleMode();
    void cycleGait();
    void toggleFeature(const Toggle& toggle);
    void sit(bool down);
    void powerDown();
    void stepHeight(int direction);
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
 * @brief Turn a joy message into GamepadState, run the layout, and publish its commands
 * @param[in] msg The incoming joystick message
 * @note Both /cmd_vel and /body_pose are always published; the gait controller uses whichever
 *       suits its mode. Full stick is the gait controller's limit, so nothing is published until
 *       its capabilities have been received.
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
    auto button = [&msg](int index) {
        return index >= 0 && static_cast<std::size_t>(index) < msg->buttons.size() && msg->buttons[index];
    };
    // Drivers disagree on what a released trigger reads (0, 1 or -1), so take the first reading
    // as released and measure presses from there: over a span of 2 from +-1, or 1 from 0
    auto trigger = [&axis](int index, std::optional<double>& rest) {
        const double value = axis(index);
        if (!rest) rest = value;
        const double span = std::abs(*rest) > 0.5 ? 2.0 : 1.0;
        return std::clamp(std::abs(value - *rest) / span, 0.0, 1.0);
    };

    // joy reports X positive to the left; GamepadState has it positive to the right
    hexapod_teleop::GamepadState pad;
    pad.leftX = static_cast<float>(-axis(axis_left_x));
    pad.leftY = static_cast<float>(axis(axis_left_y));
    pad.rightX = static_cast<float>(-axis(axis_right_x));
    pad.rightY = static_cast<float>(axis(axis_right_y));
    pad.leftTrigger = static_cast<float>(trigger(axis_left_trigger, left_trigger_rest));
    pad.rightTrigger = static_cast<float>(trigger(axis_right_trigger, right_trigger_rest));
    if (axis_dpad_y >= 0) {
        pad.dpadUp = axis(axis_dpad_y) > 0.5;
        pad.dpadDown = axis(axis_dpad_y) < -0.5;
    } else {
        pad.dpadUp = button(button_dpad_up);
        pad.dpadDown = button(button_dpad_down);
    }
    pad.a = button(toggles[0].button);
    pad.b = button(toggles[1].button);
    pad.x = button(toggles[2].button);
    pad.y = button(toggles[3].button);
    pad.rightBumper = button(button_sit);
    pad.start = button(button_mode);
    pad.back = button(button_gait);
    last_pad = pad;

    const hexapod_teleop::TeleopOutput out = teleop.update(pad, robotState(), this->now().seconds());
    handleEvents(out);

    Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = out.vx * max_linear_vel;
    cmd_vel_msg.linear.y = out.vy * max_linear_vel;
    cmd_vel_msg.angular.z = out.wz * max_angular_vel;
    cmd_vel_pub->publish(cmd_vel_msg);

    const double roll = out.roll * max_tilt_angle;    // positive lowers the right side
    const double pitch = out.pitch * max_tilt_angle;  // positive lowers the nose
    const double yaw = out.yaw * max_tilt_angle;      // positive turns left
    Pose body_pose_msg;
    body_pose_msg.position.x = out.shiftX * max_tilt_translation;
    body_pose_msg.position.y = out.shiftY * max_tilt_translation;
    body_pose_msg.position.z = out.shiftZ * max_tilt_translation;
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
 * @brief Run the layout with the last joy message, for events that come from time passing
*/
void ControllerTeleop::tick() {
    if (!capabilities_received || !last_pad) return;
    handleEvents(teleop.update(*last_pad, robotState(), this->now().seconds()));
}

/**
 * @brief What the layout needs to know about the robot
*/
hexapod_teleop::RobotState ControllerTeleop::robotState() const {
    hexapod_teleop::RobotState state;
    state.seated = seated;
    state.tiltMode = current_mode_index < available_modes.size() && available_modes[current_mode_index].name == "tilt";
    return state;
}

/**
 * @brief Act on the layout's events: mode, gait, features, sit/stand, power down, height
*/
void ControllerTeleop::handleEvents(const hexapod_teleop::TeleopOutput& out) {
    if (out.nextMode) cycleMode();
    if (out.nextGait) cycleGait();
    for (int i = 0; i < hexapod_teleop::NUM_FEATURES; ++i) {
        if (out.toggleFeature[i]) toggleFeature(toggles[i]);
    }
    if (out.sit) sit(*out.sit);
    if (out.powerDown) powerDown();
    if (out.heightStep != 0) stepHeight(out.heightStep);
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
    requestCapabilities([this](const GetCapabilities::Response& response) {
        capabilities_pending = false;
        capabilities_received = true;
        RCLCPP_INFO(this->get_logger(), "Received capabilities: %zu gaits, %zu modes",
                    response.gaits.size(), response.modes.size());
        RCLCPP_INFO(this->get_logger(), "Full stick: linear=%.2f m/s, angular=%.2f rad/s",
                    max_linear_vel, max_angular_vel);
    });
}

/**
 * @brief Ask the gait controller for its capabilities and current state, store them, then call then
 * @param[in] then Called with the response once it has been stored; may be empty
*/
void ControllerTeleop::requestCapabilities(std::function<void(const GetCapabilities::Response&)> then) {
    get_capabilities_client->async_send_request(std::make_shared<GetCapabilities::Request>(),
        [this, then](rclcpp::Client<GetCapabilities>::SharedFuture future) {
            const auto response = future.get();
            applyCapabilities(*response);
            if (then) then(*response);
        });
}

/**
 * @brief Store the gaits, modes and limits, and which mode and gait the gait controller is in
 * @note Re-run whenever the mode may have changed behind this node's back (a sit returns to normal)
*/
void ControllerTeleop::applyCapabilities(const GetCapabilities::Response& response) {
    available_gaits = response.gaits;
    available_modes = response.modes;
    max_linear_vel = response.max_linear_vel;
    max_angular_vel = response.max_angular_vel;
    max_tilt_angle = response.max_tilt_angle;
    max_tilt_translation = response.max_tilt_translation;
    for (size_t i = 0; i < available_modes.size(); ++i) {
        if (available_modes[i].id == response.current_mode) current_mode_index = i;
    }
    for (size_t i = 0; i < available_gaits.size(); ++i) {
        if (available_gaits[i].id == response.current_gait) current_gait_index = i;
    }
}

/**
 * @brief Track whether the robot is seated (or sitting down, or standing up)
 * @note A sit puts the gait controller back in normal mode, so the mode is fetched again
*/
void ControllerTeleop::seatedCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data == seated) return;
    seated = msg->data;
    height_target.reset();
    if (capabilities_received && get_capabilities_client->service_is_ready()) requestCapabilities({});
}

/**
 * @brief Sit down, or stand up (which also turns the servos back on)
 * @param[in] down true to sit down
 * @note The layout picks which from seated, which is true while a sit or stand is under way, so a
 *       press then stands up: it reverses a sit
*/
void ControllerTeleop::sit(bool down) {
    if (!sit_client->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "Can't sit or stand: the gait controller isn't answering");
        return;
    }
    auto request = std::make_shared<SetBool::Request>();
    request->data = down;
    RCLCPP_INFO(this->get_logger(), "%s", down ? "Sitting down" : "Standing up");
    sit_client->async_send_request(request, [this](rclcpp::Client<SetBool>::SharedFuture future) {
        const auto response = future.get();
        if (!response->success) RCLCPP_WARN(this->get_logger(), "Refused: %s", response->message.c_str());
    });
}

/**
 * @brief Sit down and then turn the servo power off; standing up turns it back on
*/
void ControllerTeleop::powerDown() {
    if (!power_down_client->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "Can't power down: the gait controller isn't answering");
        return;
    }
    power_down_client->async_send_request(std::make_shared<Trigger::Request>(),
        [this](rclcpp::Client<Trigger>::SharedFuture future) {
            const auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Power down: %s", response->message.c_str());
        });
}

/**
 * @brief Raise (1) or lower (-1) the body by height_step, within the gait controller's limits
 * @note Fetches the current mode and height first. In tilt mode the d-pad moves the body through
 *       body_pose instead, so nothing is sent. hexapod_teleop::stepHeight picks the target: from the
 *       height this last asked for while the body is still on its way there, so quick presses add
 *       up; otherwise from where the body is.
*/
void ControllerTeleop::stepHeight(int direction) {
    if (seated || height_pending) return;
    if (!get_capabilities_client->service_is_ready() || !set_height_client->service_is_ready()) return;
    height_pending = true;
    requestCapabilities([this, direction](const GetCapabilities::Response& response) {
        for (const auto& mode : response.modes) {
            if (mode.id == response.current_mode && mode.name == "tilt") {
                height_pending = false;
                return;
            }
        }
        const std::optional<float> last = height_target ? std::optional<float>(static_cast<float>(*height_target)) : std::nullopt;
        const double target = hexapod_teleop::stepHeight(static_cast<float>(response.current_height), last, direction,
                                                         static_cast<float>(height_step), static_cast<float>(response.min_height),
                                                         static_cast<float>(response.max_height));
        const double from = height_target && std::abs(response.current_height - *height_target) < height_step
            ? *height_target : response.current_height;
        if (std::abs(target - from) < 1e-4) {
            RCLCPP_INFO(this->get_logger(), "Body height already at its %s (%.1f cm)",
                        direction > 0 ? "highest" : "lowest", target * 100.0);
            height_pending = false;
            return;
        }
        auto request = std::make_shared<SetHeight::Request>();
        request->height = -target;
        set_height_client->async_send_request(request, [this, target](rclcpp::Client<SetHeight>::SharedFuture future) {
            height_pending = false;
            const auto result = future.get();
            if (result->success) {
                height_target = target;
                RCLCPP_INFO(this->get_logger(), "Body height %.1f cm", target * 100.0);
            } else {
                RCLCPP_WARN(this->get_logger(), "Height refused: %s", result->message.c_str());
            }
        });
    });
}
