#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include "hexapod_hardware_interface/ina226.hpp"

using hexapod_hardware_interface::Ina226;
using sensor_msgs::msg::BatteryState;

/**
 * @brief One INA226 power monitor and its publisher
*/
struct PowerMonitor {
    std::string location;
    uint8_t address;
    double shunt_resistance;
    Ina226 device;
    bool initialised = false;
    bool warned = false;  // logged that the monitor is missing
    rclcpp::Publisher<BatteryState>::SharedPtr pub;
};

/**
 * @brief Driver node for the INA226 power monitors on the servo daughter boards
 * @note Publishes sensor_msgs/BatteryState on power/<location> for each board.
 *       Current is negative while the servos draw power, as BatteryState specifies.
*/
class Ina226Node : public rclcpp::Node {
public:
    /**
     * @brief Declare the node's params, create a publisher per board and start the read timer
    */
    Ina226Node() : Node("ina226") {
        i2c_bus = declare_parameter("i2c_bus", "/dev/i2c-1");
        auto addresses = declare_parameter("addresses", std::vector<int64_t>{0x40, 0x41});
        auto locations = declare_parameter("locations", std::vector<std::string>{"left", "right"});
        auto shunts = declare_parameter("shunt_resistances", std::vector<double>{0.002, 0.002});
        full_voltage = declare_parameter("battery_full_voltage", 12.6);
        empty_voltage = declare_parameter("battery_empty_voltage", 9.0);
        double rate = declare_parameter("rate", 10.0);

        if (addresses.size() != locations.size() || addresses.size() != shunts.size()) {
            throw std::invalid_argument("addresses, locations and shunt_resistances must be the same length");
        }
        for (size_t i = 0; i < addresses.size(); ++i) {
            auto monitor = std::make_unique<PowerMonitor>();
            monitor->location = locations[i];
            monitor->address = static_cast<uint8_t>(addresses[i]);
            monitor->shunt_resistance = shunts[i];
            monitor->pub = create_publisher<BatteryState>("power/" + locations[i], 10);
            monitors.push_back(std::move(monitor));
        }

        timer = create_wall_timer(std::chrono::duration<double>(1.0 / rate), std::bind(&Ina226Node::update, this));
    }

private:
    /**
     * @brief Check a monitor's manufacturer ID and configure it
     * @param[in,out] monitor The monitor to configure
     * @return true if the INA226 was found and configured, false otherwise
    */
    bool initialise(PowerMonitor& monitor) {
        if (!monitor.device.initialise(i2c_bus, monitor.address)) {
            if (!monitor.warned) {
                RCLCPP_WARN(get_logger(), "INA226 (%s) not found at 0x%02X, retrying", monitor.location.c_str(), monitor.address);
                monitor.warned = true;
            }
            return false;
        }
        monitor.warned = false;
        RCLCPP_INFO(get_logger(), "INA226 (%s) ready at 0x%02X", monitor.location.c_str(), monitor.address);
        return true;
    }

    /**
     * @brief Read every monitor and publish its voltage and current
     * @note A monitor that fails to read is reinitialised on the next update
    */
    void update() {
        for (auto& monitor : monitors) {
            if (!monitor->initialised) {
                monitor->initialised = initialise(*monitor);
                if (!monitor->initialised) {
                    continue;
                }
            }

            double voltage = 0.0;
            double current = 0.0;
            if (!monitor->device.read(monitor->shunt_resistance, voltage, current)) {
                RCLCPP_WARN(get_logger(), "INA226 (%s) read failed", monitor->location.c_str());
                monitor->initialised = false;
                continue;
            }

            const double nan = std::numeric_limits<double>::quiet_NaN();

            BatteryState msg;
            msg.header.stamp = now();
            msg.voltage = static_cast<float>(voltage);
            msg.current = static_cast<float>(-current);
            msg.charge = nan;
            msg.capacity = nan;
            msg.design_capacity = nan;
            msg.percentage = static_cast<float>(
                std::clamp((voltage - empty_voltage) / (full_voltage - empty_voltage), 0.0, 1.0));
            msg.temperature = nan;
            msg.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
            msg.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
            msg.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
            msg.present = true;
            msg.location = monitor->location;
            monitor->pub->publish(msg);
        }
    }

    std::vector<std::unique_ptr<PowerMonitor>> monitors;
    rclcpp::TimerBase::SharedPtr timer;
    std::string i2c_bus;
    double full_voltage;
    double empty_voltage;
};

/**
 * @brief Entry point, initialises ROS and spins the node
 * @param[in] argc Number of command line arguments
 * @param[in] argv Command line arguments
 * @return Process exit code
*/
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ina226Node>());
    rclcpp::shutdown();
    return 0;
}
