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

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using sensor_msgs::msg::BatteryState;
using sensor_msgs::msg::JoyFeedback;
using std_srvs::srv::SetBool;

/**
 * @brief Battery level, worst last
*/
enum class BatteryLevel { OK, WARNING, CRITICAL };

/**
 * @brief One daughter board's power monitor, as read from its BatteryState topic
*/
struct PowerReading {
    std::string location;
    double voltage = 0.0;  // V, filtered
    double current = 0.0;  // A, latest (negative while discharging)
    rclcpp::Time stamp;    // when the latest reading arrived
    bool received = false;
    rclcpp::Subscription<BatteryState>::SharedPtr sub;
};

/**
 * @brief High level manager for the robot's behaviour
 * @note Watches the battery through the daughter boards' power monitors (power/<location>). The
 *       lowest board's voltage, filtered over a few seconds so walking doesn't trip it, sets the
 *       level: a warning buzzes the controller, and critical sits the robot down (the gait
 *       controller's sit service) so the servos stop drawing power. It never stands the robot back
 *       up; standing it up while critical is the operator's override. The state is published on
 *       /diagnostics for the panel.
*/
class BehaviourManager : public rclcpp::Node {
public:
    /**
     * @brief Declare the node's params, subscribe to each power monitor and start the check timer
    */
    BehaviourManager() : Node("behaviour_manager") {
        auto locations = declare_parameter("power_locations", std::vector<std::string>{"left", "right"});
        warning_voltage = declare_parameter("warning_voltage", 10.5);
        critical_voltage = declare_parameter("critical_voltage", 9.9);
        hysteresis = declare_parameter("hysteresis", 0.2);
        filter_time = declare_parameter("filter_time", 5.0);
        stale_timeout = declare_parameter("stale_timeout", 2.0);
        auto_sit = declare_parameter("auto_sit", true);
        rumble = declare_parameter("rumble", true);
        rumble_intensity = declare_parameter("rumble_intensity", 1.0);
        warning_pulses = declare_parameter("warning_pulses", 2);
        critical_pulses = declare_parameter("critical_pulses", 4);
        pulse_period = declare_parameter("pulse_period", 1.5);
        double rate = declare_parameter("rate", 2.0);

        for (const auto& location : locations) {
            auto reading = std::make_unique<PowerReading>();
            reading->location = location;
            PowerReading* raw = reading.get();
            reading->sub = create_subscription<BatteryState>("power/" + location, 10,
                [this, raw](const BatteryState::SharedPtr msg) { powerCallback(*raw, *msg); });
            readings.push_back(std::move(reading));
        }

        diagnostics_pub = create_publisher<DiagnosticArray>("/diagnostics", 10);
        feedback_pub = create_publisher<JoyFeedback>("joy/set_feedback", 10);
        sit_client = create_client<SetBool>("sit");

        timer = create_timer(std::chrono::duration<double>(1.0 / rate), std::bind(&BehaviourManager::update, this));
        RCLCPP_INFO(get_logger(), "Behaviour manager started: battery warning below %.2f V, critical below %.2f V",
            warning_voltage, critical_voltage);
    }

private:
    /**
     * @brief Filter a board's voltage with a low-pass of time constant filter_time
     * @param[in,out] reading The board the message came from
     * @param[in] msg The board's latest power reading
    */
    void powerCallback(PowerReading& reading, const BatteryState& msg) {
        const rclcpp::Time stamp = now();
        if (!std::isfinite(msg.voltage)) {
            return;
        }
        if (!reading.received) {
            reading.voltage = msg.voltage;
        } else {
            const double dt = std::max(0.0, (stamp - reading.stamp).seconds());
            const double alpha = filter_time > 0.0 ? 1.0 - std::exp(-dt / filter_time) : 1.0;
            reading.voltage += alpha * (msg.voltage - reading.voltage);
        }
        reading.current = msg.current;
        reading.stamp = stamp;
        reading.received = true;
    }

    /**
     * @brief Work out the battery level, act on a change and publish the diagnostics
    */
    void update() {
        const rclcpp::Time stamp = now();
        double voltage = std::numeric_limits<double>::infinity();
        std::vector<std::string> stale;
        for (const auto& reading : readings) {
            if (!reading->received || (stamp - reading->stamp).seconds() > stale_timeout) {
                stale.push_back(reading->location);
            } else {
                voltage = std::min(voltage, reading->voltage);
            }
        }
        const bool has_data = std::isfinite(voltage);

        // Without data the level holds: a dropped topic is no reason to sit down or to clear a warning
        if (has_data) {
            const BatteryLevel previous = level;
            level = levelFor(voltage);
            if (level != previous) {
                levelChanged(previous, voltage);
            }
        }
        if (level == BatteryLevel::CRITICAL && sit_pending) {
            requestSit();
        }
        publishDiagnostics(has_data ? voltage : std::numeric_limits<double>::quiet_NaN(), stale);
    }

    /**
     * @brief The battery level for a voltage, with hysteresis so it doesn't flicker at a threshold
     * @param[in] voltage The lowest board's filtered voltage (V)
     * @return The new level; recovering needs hysteresis above the threshold that was crossed
    */
    BatteryLevel levelFor(double voltage) const {
        if (voltage < critical_voltage ||
            (level == BatteryLevel::CRITICAL && voltage < critical_voltage + hysteresis)) {
            return BatteryLevel::CRITICAL;
        }
        if (voltage < warning_voltage ||
            (level != BatteryLevel::OK && voltage < warning_voltage + hysteresis)) {
            return BatteryLevel::WARNING;
        }
        return BatteryLevel::OK;
    }

    /**
     * @brief Log a change of battery level, buzz the controller if it got worse, and sit at critical
     * @param[in] previous The level before the change
     * @param[in] voltage The voltage that caused it (V)
    */
    void levelChanged(BatteryLevel previous, double voltage) {
        switch (level) {
            case BatteryLevel::OK:
                RCLCPP_INFO(get_logger(), "Battery OK (%.2f V)", voltage);
                break;
            case BatteryLevel::WARNING:
                if (previous == BatteryLevel::OK) {
                    RCLCPP_WARN(get_logger(), "Battery low (%.2f V)", voltage);
                } else {
                    RCLCPP_WARN(get_logger(), "Battery back above critical (%.2f V), still low", voltage);
                }
                break;
            case BatteryLevel::CRITICAL:
                RCLCPP_ERROR(get_logger(), "Battery critical (%.2f V)%s", voltage, auto_sit ? ", sitting down" : "");
                sit_pending = auto_sit;
                break;
        }
        if (level != BatteryLevel::CRITICAL) {
            sit_pending = false;
        }
        if (rumble && level > previous) {
            buzz(level == BatteryLevel::CRITICAL ? critical_pulses : warning_pulses);
        }
    }

    /**
     * @brief Ask the gait controller to sit down, retrying on the next update if it isn't up yet
    */
    void requestSit() {
        if (!sit_client->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Waiting for the sit service to sit down");
            return;
        }
        sit_pending = false;
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
     * @param[in] voltage The lowest board's filtered voltage (V), NaN without data
     * @param[in] stale The boards with no recent reading
    */
    void publishDiagnostics(double voltage, const std::vector<std::string>& stale) {
        DiagnosticStatus status;
        status.name = "behaviour_manager: battery";
        status.hardware_id = "battery";
        if (stale.size() == readings.size()) {
            status.level = DiagnosticStatus::STALE;
            status.message = "No power readings";
        } else {
            switch (level) {
                case BatteryLevel::OK:
                    status.level = DiagnosticStatus::OK;
                    status.message = "OK";
                    break;
                case BatteryLevel::WARNING:
                    status.level = DiagnosticStatus::WARN;
                    status.message = "Low";
                    break;
                case BatteryLevel::CRITICAL:
                    status.level = DiagnosticStatus::ERROR;
                    status.message = "Critical";
                    break;
            }
            if (!stale.empty()) {
                status.level = std::max(status.level, DiagnosticStatus::WARN);
                status.message += ", no readings from " + join(stale);
            }
        }

        auto add = [&status](const std::string& key, const std::string& value) {
            diagnostic_msgs::msg::KeyValue pair;
            pair.key = key;
            pair.value = value;
            status.values.push_back(pair);
        };
        auto volts = [](double v) { return std::isfinite(v) ? format("%.2f", v) : std::string("n/a"); };
        add("voltage", volts(voltage));
        for (const auto& reading : readings) {
            add(reading->location + " voltage", reading->received ? volts(reading->voltage) : "n/a");
            add(reading->location + " current", reading->received ? format("%.2f", std::abs(reading->current)) : "n/a");
        }
        add("warning voltage", volts(warning_voltage));
        add("critical voltage", volts(critical_voltage));

        DiagnosticArray array;
        array.header.stamp = now();
        array.status.push_back(status);
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

    std::vector<std::unique_ptr<PowerReading>> readings;
    rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub;
    rclcpp::Publisher<JoyFeedback>::SharedPtr feedback_pub;
    rclcpp::Client<SetBool>::SharedPtr sit_client;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr pulse_timer;
    int pulses_left = 0;        // rumble pulses still to play
    BatteryLevel level = BatteryLevel::OK;
    bool sit_pending = false;   // critical, and the sit request hasn't gone out yet
    double warning_voltage;     // V, below this the battery is low
    double critical_voltage;    // V, below this the robot sits down
    double hysteresis;          // V, how far above a threshold the voltage must recover to clear it
    double filter_time;         // s, time constant of the voltage filter
    double stale_timeout;       // s, a board with no reading for this long is ignored
    bool auto_sit;              // sit down at critical
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
