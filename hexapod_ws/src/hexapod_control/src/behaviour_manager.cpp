#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "hexapod_behaviour/battery_monitor.hpp"

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using gait_core::LogLevel;
using hexapod_behaviour::BatteryLevel;
using hexapod_behaviour::BatteryMonitor;
using hexapod_behaviour::BatteryParams;
using hexapod_behaviour::BatteryStatus;
using sensor_msgs::msg::BatteryState;
using sensor_msgs::msg::JoyFeedback;
using std_srvs::srv::SetBool;
using std_srvs::srv::Trigger;

/**
 * @brief High level manager for the robot's behaviour
 * @note The decisions are gait_core's BatteryMonitor, shared with the ESP32 and Servo2040 firmware.
 *       This node feeds it the daughter boards' power monitors (power/<location>) and carries out
 *       what it decides: a warning buzzes the controller, and critical sits the robot down and turns
 *       the servo power off (the gait controller's power_down service; its sit service with
 *       power_off false) so the servos stop drawing power. The state is published on /diagnostics
 *       for the panel.
*/
class BehaviourManager : public rclcpp::Node {
public:
    /**
     * @brief Declare the node's params, subscribe to each power monitor and start the check timer
    */
    BehaviourManager() : Node("behaviour_manager"), battery(1) {
        locations = declare_parameter("power_locations", std::vector<std::string>{"left", "right"});
        BatteryParams params;
        params.warning_voltage = declare_parameter("warning_voltage", 10.5);
        params.critical_voltage = declare_parameter("critical_voltage", 9.9);
        params.hysteresis = declare_parameter("hysteresis", 0.2);
        params.filter_time = declare_parameter("filter_time", 5.0);
        params.stale_timeout = declare_parameter("stale_timeout", 2.0);
        params.auto_sit = declare_parameter("auto_sit", true);
        params.power_off = declare_parameter("power_off", true);
        rumble = declare_parameter("rumble", true);
        rumble_intensity = declare_parameter("rumble_intensity", 1.0);
        warning_pulses = declare_parameter("warning_pulses", 2);
        critical_pulses = declare_parameter("critical_pulses", 4);
        pulse_period = declare_parameter("pulse_period", 1.5);
        double rate = declare_parameter("rate", 2.0);

        if (locations.size() > static_cast<size_t>(BatteryMonitor::MAX_MONITORS)) {
            RCLCPP_WARN(get_logger(), "Only the first %d power_locations are watched", BatteryMonitor::MAX_MONITORS);
            locations.resize(BatteryMonitor::MAX_MONITORS);
        }
        battery = BatteryMonitor(static_cast<int>(locations.size()), params);
        battery.log = [this](LogLevel level, const char* message) {
            switch (level) {
                case LogLevel::Debug: RCLCPP_DEBUG(get_logger(), "%s", message); break;
                case LogLevel::Info: RCLCPP_INFO(get_logger(), "%s", message); break;
                case LogLevel::Warn: RCLCPP_WARN(get_logger(), "%s", message); break;
                case LogLevel::Error: RCLCPP_ERROR(get_logger(), "%s", message); break;
            }
        };

        for (size_t i = 0; i < locations.size(); ++i) {
            const int monitor = static_cast<int>(i);
            power_subs.push_back(create_subscription<BatteryState>("power/" + locations[i], 10,
                [this, monitor](const BatteryState::SharedPtr msg) {
                    battery.onReading(monitor, msg->voltage, msg->current, now().nanoseconds());
                }));
        }

        diagnostics_pub = create_publisher<DiagnosticArray>("/diagnostics", 10);
        feedback_pub = create_publisher<JoyFeedback>("joy/set_feedback", 10);
        sit_client = create_client<SetBool>("sit");
        power_down_client = create_client<Trigger>("power_down");

        timer = create_timer(std::chrono::duration<double>(1.0 / rate), std::bind(&BehaviourManager::update, this));
        RCLCPP_INFO(get_logger(), "Behaviour manager started: battery warning below %.2f V, critical below %.2f V",
            params.warning_voltage, params.critical_voltage);
    }

private:
    /**
     * @brief Let the battery monitor decide, carry out what it says and publish the diagnostics
    */
    void update() {
        const BatteryStatus status = battery.update(now().nanoseconds());
        if (rumble && status.worsened) {
            buzz(status.level == BatteryLevel::Critical ? critical_pulses : warning_pulses);
        }
        if (status.sit) {
            if (status.powerOff) {
                requestPowerDown();
            } else {
                requestSit();
            }
        }
        publishDiagnostics(status);
    }

    /**
     * @brief Ask the gait controller to sit down, retrying on the next update if it isn't up yet
    */
    void requestSit() {
        if (!sit_client->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Waiting for the sit service to sit down");
            return;
        }
        battery.sitRequested();
        auto request = std::make_shared<SetBool::Request>();
        request->data = true;
        sit_client->async_send_request(request, [this](rclcpp::Client<SetBool>::SharedFuture future) {
            const auto response = future.get();
            if (!response->success) {
                RCLCPP_ERROR(get_logger(), "Gait controller refused to sit: %s", response->message.c_str());
            }
        });
    }

    /**
     * @brief Ask the gait controller to sit and turn the servos off, retrying on the next update if it isn't up yet
    */
    void requestPowerDown() {
        if (!power_down_client->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Waiting for the power_down service to sit down");
            return;
        }
        battery.sitRequested();
        power_down_client->async_send_request(std::make_shared<Trigger::Request>(),
            [this](rclcpp::Client<Trigger>::SharedFuture future) {
                const auto response = future.get();
                if (!response->success) {
                    RCLCPP_ERROR(get_logger(), "Gait controller refused to power down: %s", response->message.c_str());
                }
            });
    }

    /**
     * @brief Rumble the controller in a train of pulses, replacing any train still playing
     * @param[in] pulses How many pulses to play
     * @note The joy node plays each rumble message for a fixed second, so a pulse_period over 1 s
     *       leaves a gap between pulses. Wall time, so it buzzes at the same pace in the sim.
    */
    void buzz(int pulses) {
        pulses_left = pulses;
        pulse();
        pulse_timer = create_wall_timer(std::chrono::duration<double>(pulse_period), [this]() { pulse(); });
    }

    /**
     * @brief Play one rumble pulse, stopping the pulse timer after the last
    */
    void pulse() {
        if (pulses_left <= 0) {
            if (pulse_timer) {
                pulse_timer->cancel();
            }
            return;
        }
        pulses_left--;
        JoyFeedback feedback;
        feedback.type = JoyFeedback::TYPE_RUMBLE;
        feedback.id = 0;
        feedback.intensity = static_cast<float>(std::clamp(rumble_intensity, 0.0, 1.0));
        feedback_pub->publish(feedback);
    }

    /**
     * @brief Publish the battery state on /diagnostics
     * @param[in] status The battery monitor's latest status
    */
    void publishDiagnostics(const BatteryStatus& status) {
        const int64_t stamp = now().nanoseconds();
        std::vector<std::string> stale;
        for (size_t i = 0; i < locations.size(); ++i) {
            if (battery.stale(static_cast<int>(i), stamp)) {
                stale.push_back(locations[i]);
            }
        }

        DiagnosticStatus diagnostic;
        diagnostic.name = "behaviour_manager: battery";
        diagnostic.hardware_id = "battery";
        if (stale.size() == locations.size()) {
            diagnostic.level = DiagnosticStatus::STALE;
            diagnostic.message = "No power readings";
        } else {
            switch (status.level) {
                case BatteryLevel::Ok:
                    diagnostic.level = DiagnosticStatus::OK;
                    diagnostic.message = "OK";
                    break;
                case BatteryLevel::Warning:
                    diagnostic.level = DiagnosticStatus::WARN;
                    diagnostic.message = "Low";
                    break;
                case BatteryLevel::Critical:
                    diagnostic.level = DiagnosticStatus::ERROR;
                    diagnostic.message = "Critical";
                    break;
            }
            if (!stale.empty()) {
                diagnostic.level = std::max(diagnostic.level, DiagnosticStatus::WARN);
                diagnostic.message += ", no readings from " + join(stale);
            }
        }

        auto add = [&diagnostic](const std::string& key, const std::string& value) {
            diagnostic_msgs::msg::KeyValue pair;
            pair.key = key;
            pair.value = value;
            diagnostic.values.push_back(pair);
        };
        auto volts = [](double v) { return std::isfinite(v) ? format("%.2f", v) : std::string("n/a"); };
        add("voltage", volts(status.hasData ? status.voltage : std::numeric_limits<double>::quiet_NaN()));
        for (size_t i = 0; i < locations.size(); ++i) {
            const auto& reading = battery.reading(static_cast<int>(i));
            add(locations[i] + " voltage", reading.received ? volts(reading.voltage) : "n/a");
            add(locations[i] + " current", reading.received ? format("%.2f", std::abs(reading.current)) : "n/a");
        }
        add("warning voltage", volts(battery.params().warning_voltage));
        add("critical voltage", volts(battery.params().critical_voltage));

        DiagnosticArray array;
        array.header.stamp = now();
        array.status.push_back(diagnostic);
        diagnostics_pub->publish(array);
    }

    /**
     * @brief Format a number as a string
     * @param[in] fmt printf format with one double
     * @param[in] value The number
     * @return The formatted string
    */
    static std::string format(const char* fmt, double value) {
        char buffer[32];
        std::snprintf(buffer, sizeof(buffer), fmt, value);
        return buffer;
    }

    /**
     * @brief Join names with commas
     * @param[in] names The names to join
     * @return "a, b"
    */
    static std::string join(const std::vector<std::string>& names) {
        std::string out;
        for (const auto& name : names) {
            out += (out.empty() ? "" : ", ") + name;
        }
        return out;
    }

    BatteryMonitor battery;
    std::vector<std::string> locations;   // power/<location>, one per monitor, in monitor order
    std::vector<rclcpp::Subscription<BatteryState>::SharedPtr> power_subs;
    rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub;
    rclcpp::Publisher<JoyFeedback>::SharedPtr feedback_pub;
    rclcpp::Client<SetBool>::SharedPtr sit_client;
    rclcpp::Client<Trigger>::SharedPtr power_down_client;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr pulse_timer;
    int pulses_left = 0;        // rumble pulses still to play
    bool rumble;                // buzz the controller when the level gets worse
    double rumble_intensity;    // 0 to 1
    int warning_pulses;         // pulses when the battery goes low
    int critical_pulses;        // pulses when it goes critical
    double pulse_period;        // s from one pulse's start to the next
};

/**
 * @brief Entry point, initialises ROS and spins the node
 * @param[in] argc Number of command line arguments
 * @param[in] argv Command line arguments
 * @return Process exit code
*/
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BehaviourManager>());
    rclcpp::shutdown();
    return 0;
}
