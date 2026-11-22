#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include "hexapod_hardware_interface/st3215_bus.hpp"
#include "hexapod_hardware_interface/st3215_system.hpp"

using control_msgs::msg::DynamicJointState;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using sensor_msgs::msg::JointState;
using namespace hexapod_hardware_interface;

namespace {

// The servo's protection turns it off at its max_temperature register (70 °C by default)
constexpr int SHUTDOWN_TEMPERATURE = 70;

/**
 * @brief One joint's servo health, from its /dynamic_joint_states interfaces
*/
struct ServoHealth {
    std::string joint;
    double temperature = NAN;  // °C
    double voltage = NAN;      // V
    double current = NAN;      // A
    double effort = NAN;       // N·m
    double error = NAN;        // status packet error bits
};

std::string fixed(double value, int decimals) {
    if (!std::isfinite(value)) {
        return "n/a";
    }
    char text[32];
    std::snprintf(text, sizeof(text), "%.*f", decimals, value);
    return text;
}

KeyValue keyValue(const std::string& key, const std::string& value) {
    KeyValue kv;
    kv.key = key;
    kv.value = value;
    return kv;
}

}  // namespace

/**
 * @brief Servo health on /diagnostics, plus throttled copies of the joint states for plotting
 * @note Reads the temperature, voltage, current and servo_error interfaces ST3215System adds to each
 *       joint, which joint_state_broadcaster publishes on /dynamic_joint_states. Every
 *       diagnostics_rate it publishes one DiagnosticStatus per servo and a summary
 *       ("servo_monitor: servos"):
 *       - temperature at or above temperature_warn is a warning, at or above temperature_error an
 *         error (the servo turns itself off at 70 °C)
 *       - a servo more than voltage_drop_warn below the best-supplied one is a warning: a bad
 *         connection, or a long run of servos sharing one supply wire
 *       - the servo's own error bits: overheat, overcurrent and overload are errors, the rest warnings
 *       - a servo that doesn't answer is stale, unless /servo_power says the power is off
 *       /joint_states and /dynamic_joint_states come at the controller rate (100 Hz), too fast to
 *       plot over Wi-Fi, so they're republished on joint_states/throttled (joint_states_rate) and
 *       dynamic_joint_states/throttled (dynamic_joint_states_rate).
*/
class ServoMonitor : public rclcpp::Node {
public:
    ServoMonitor() : Node("servo_monitor") {
        temperature_warn = declare_parameter("temperature_warn", 55.0);
        temperature_error = declare_parameter("temperature_error", 65.0);
        voltage_drop_warn = declare_parameter("voltage_drop_warn", 1.0);
        stale_timeout = declare_parameter("stale_timeout", 2.0);
        const double diagnostics_rate = declare_parameter("diagnostics_rate", 1.0);
        const double joint_states_rate = declare_parameter("joint_states_rate", 10.0);
        const double dynamic_rate = declare_parameter("dynamic_joint_states_rate", 2.0);

        diagnostics_pub = create_publisher<DiagnosticArray>("/diagnostics", 10);
        joint_states_pub = create_publisher<JointState>("joint_states/throttled", 10);
        dynamic_pub = create_publisher<DynamicJointState>("dynamic_joint_states/throttled", 10);

        dynamic_sub = create_subscription<DynamicJointState>("dynamic_joint_states", 10,
            [this](DynamicJointState::SharedPtr msg) {
                latest_dynamic = msg;
                last_dynamic_time = now();
            });
        joint_states_sub = create_subscription<JointState>("joint_states", 10,
            [this](JointState::SharedPtr msg) { latest_joint_states = msg; });
        // Latched by the gait controller
        servo_power_sub = create_subscription<std_msgs::msg::Bool>("servo_power", rclcpp::QoS(1).transient_local(),
            [this](std_msgs::msg::Bool::SharedPtr msg) { servo_power = msg->data; });

        diagnostics_timer = create_wall_timer(period(diagnostics_rate), [this] { publishDiagnostics(); });
        joint_states_timer = create_wall_timer(period(joint_states_rate), [this] {
            if (latest_joint_states) {
                joint_states_pub->publish(*latest_joint_states);
                latest_joint_states.reset();
            }
        });
        dynamic_timer = create_wall_timer(period(dynamic_rate), [this] {
            if (latest_dynamic && latest_dynamic != last_dynamic_sent) {
                dynamic_pub->publish(*latest_dynamic);
                last_dynamic_sent = latest_dynamic;
            }
        });
    }

private:
    static std::chrono::nanoseconds period(double rate_hz) {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / std::max(rate_hz, 0.01)));
    }

    /**
     * @brief The servo health in the latest /dynamic_joint_states, for joints that report it
    */
    std::vector<ServoHealth> readHealth() const {
        std::vector<ServoHealth> servos;
        if (!latest_dynamic) {
            return servos;
        }
        const auto& msg = *latest_dynamic;
        for (size_t i = 0; i < msg.joint_names.size() && i < msg.interface_values.size(); ++i) {
            const auto& iv = msg.interface_values[i];
            auto value = [&iv](const char* name) -> std::optional<double> {
                for (size_t k = 0; k < iv.interface_names.size() && k < iv.values.size(); ++k) {
                    if (iv.interface_names[k] == name) {
                        return iv.values[k];
                    }
                }
                return std::nullopt;
            };
            const auto temperature = value(HW_IF_TEMPERATURE);
            if (!temperature) {
                continue;  // no health interfaces (simulation or mock hardware)
            }
            ServoHealth servo;
            servo.joint = msg.joint_names[i];
            servo.temperature = *temperature;
            servo.voltage = value(HW_IF_VOLTAGE).value_or(NAN);
            servo.current = value(HW_IF_CURRENT).value_or(NAN);
            servo.effort = value("effort").value_or(NAN);
            servo.error = value(HW_IF_SERVO_ERROR).value_or(NAN);
            servos.push_back(servo);
        }
        return servos;
    }

    /**
     * @brief Grade one servo
    */
    DiagnosticStatus servoStatus(const ServoHealth& servo, double best_voltage) const {
        DiagnosticStatus status;
        status.name = "servo_monitor: " + servo.joint;
        status.hardware_id = servo.joint;
        status.values = {
            keyValue("temperature", fixed(servo.temperature, 0)),
            keyValue("voltage", fixed(servo.voltage, 1)),
            keyValue("current", fixed(servo.current, 2)),
            keyValue("load", fixed(servo.effort, 2)),
            keyValue("errors", std::isfinite(servo.error) ? st3215ErrorText(static_cast<uint8_t>(servo.error)) : "n/a"),
        };

        if (!std::isfinite(servo.temperature)) {
            if (!servo_power) {
                status.level = DiagnosticStatus::OK;
                status.message = "Servo power off";
            } else {
                status.level = DiagnosticStatus::STALE;
                status.message = "Not answering";
            }
            return status;
        }

        status.level = DiagnosticStatus::OK;
        std::vector<std::string> problems;
        auto worsen = [&status](uint8_t level) { status.level = std::max(status.level, level); };

        if (servo.temperature >= temperature_error) {
            worsen(DiagnosticStatus::ERROR);
            problems.push_back("Hot, " + fixed(servo.temperature, 0) + " °C (shuts off at " +
                std::to_string(SHUTDOWN_TEMPERATURE) + " °C)");
        } else if (servo.temperature >= temperature_warn) {
            worsen(DiagnosticStatus::WARN);
            problems.push_back("Warm, " + fixed(servo.temperature, 0) + " °C");
        }

        if (std::isfinite(servo.voltage) && std::isfinite(best_voltage) && best_voltage - servo.voltage > voltage_drop_warn) {
            worsen(DiagnosticStatus::WARN);
            problems.push_back("Low supply, " + fixed(servo.voltage, 1) + " V (best servo " + fixed(best_voltage, 1) + " V)");
        }

        if (std::isfinite(servo.error) && servo.error != 0.0) {
            const auto bits = static_cast<uint8_t>(servo.error);
            const uint8_t serious = ST3215_ERROR_OVERHEAT | ST3215_ERROR_OVERCURRENT | ST3215_ERROR_OVERLOAD;
            worsen((bits & serious) ? DiagnosticStatus::ERROR : DiagnosticStatus::WARN);
            problems.push_back("Servo reports " + st3215ErrorText(bits));
        }

        if (problems.empty()) {
            status.message = "OK";
        } else {
            for (size_t i = 0; i < problems.size(); ++i) {
                status.message += (i == 0 ? "" : ", ") + problems[i];
            }
        }
        return status;
    }

    /**
     * @brief Publish every servo's status and the summary
    */
    void publishDiagnostics() {
        DiagnosticArray array;
        array.header.stamp = now();

        DiagnosticStatus summary;
        summary.name = "servo_monitor: servos";
        summary.hardware_id = "servos";

        const bool stale = !latest_dynamic || (now() - last_dynamic_time).seconds() > stale_timeout;
        const auto servos = stale ? std::vector<ServoHealth>{} : readHealth();
        if (servos.empty()) {
            summary.level = DiagnosticStatus::STALE;
            summary.message = stale ? "No /dynamic_joint_states" : "No servo health readings";
            array.status.push_back(summary);
            diagnostics_pub->publish(array);
            return;
        }

        double best_voltage = NAN;
        for (const auto& servo : servos) {
            if (std::isfinite(servo.voltage) && !(servo.voltage <= best_voltage)) {
                best_voltage = servo.voltage;
            }
        }

        // The summary: the worst servo's level, with its problem, and the hottest servo
        summary.level = DiagnosticStatus::OK;
        const ServoHealth* hottest = nullptr;
        const ServoHealth* most_current = nullptr;
        double min_voltage = NAN;
        int not_answering = 0;
        std::vector<std::pair<uint8_t, std::string>> problems;  // level, "joint: message"
        for (const auto& servo : servos) {
            const DiagnosticStatus status = servoStatus(servo, best_voltage);
            array.status.push_back(status);
            if (status.level == DiagnosticStatus::STALE) {
                ++not_answering;
                continue;
            }
            if (status.level != DiagnosticStatus::OK) {
                summary.level = std::max(summary.level, status.level);
                problems.emplace_back(status.level, servo.joint + ": " + status.message);
            }
            if (std::isfinite(servo.temperature) && (!hottest || servo.temperature > hottest->temperature)) {
                hottest = &servo;
            }
            if (std::isfinite(servo.current) && (!most_current || std::abs(servo.current) > std::abs(most_current->current))) {
                most_current = &servo;
            }
            if (std::isfinite(servo.voltage) && !(servo.voltage >= min_voltage)) {
                min_voltage = servo.voltage;
            }
        }
        if (not_answering > 0) {
            summary.level = std::max(summary.level, static_cast<uint8_t>(DiagnosticStatus::WARN));
        }

        // Errors first, and only the first MAX_LISTED, so the message stays short enough to show
        constexpr size_t MAX_LISTED = 2;
        std::stable_sort(problems.begin(), problems.end(),
            [](const auto& a, const auto& b) { return a.first > b.first; });
        std::vector<std::string> parts;
        for (size_t i = 0; i < problems.size() && i < MAX_LISTED; ++i) {
            parts.push_back(problems[i].second);
        }
        if (problems.size() > MAX_LISTED) {
            parts.push_back("and " + std::to_string(problems.size() - MAX_LISTED) + " more");
        }
        if (not_answering > 0) {
            parts.push_back(std::to_string(not_answering) + " not answering");
        }

        if (!parts.empty()) {
            for (size_t i = 0; i < parts.size(); ++i) {
                summary.message += (i == 0 ? "" : "; ") + parts[i];
            }
        } else if (hottest) {
            summary.message = "OK, hottest " + hottest->joint + " " + fixed(hottest->temperature, 0) + " °C";
        } else {
            summary.message = servo_power ? "OK" : "Servo power off";
        }
        summary.values = {
            keyValue("hottest", hottest ? hottest->joint : "n/a"),
            keyValue("max_temperature", hottest ? fixed(hottest->temperature, 0) : "n/a"),
            keyValue("max_current_joint", most_current ? most_current->joint : "n/a"),
            keyValue("max_current", most_current ? fixed(std::abs(most_current->current), 2) : "n/a"),
            keyValue("min_voltage", fixed(min_voltage, 1)),
            keyValue("not_answering", std::to_string(not_answering)),
        };
        array.status.insert(array.status.begin(), summary);
        diagnostics_pub->publish(array);
    }

    double temperature_warn;
    double temperature_error;
    double voltage_drop_warn;
    double stale_timeout;
    bool servo_power = true;  // until /servo_power says otherwise

    DynamicJointState::SharedPtr latest_dynamic;
    DynamicJointState::SharedPtr last_dynamic_sent;
    rclcpp::Time last_dynamic_time;
    JointState::SharedPtr latest_joint_states;

    rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub;
    rclcpp::Publisher<JointState>::SharedPtr joint_states_pub;
    rclcpp::Publisher<DynamicJointState>::SharedPtr dynamic_pub;
    rclcpp::Subscription<DynamicJointState>::SharedPtr dynamic_sub;
    rclcpp::Subscription<JointState>::SharedPtr joint_states_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servo_power_sub;
    rclcpp::TimerBase::SharedPtr diagnostics_timer;
    rclcpp::TimerBase::SharedPtr joint_states_timer;
    rclcpp::TimerBase::SharedPtr dynamic_timer;
};

/**
 * @brief Entry point, initialises ROS and spins the node
*/
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoMonitor>());
    rclcpp::shutdown();
    return 0;
}
