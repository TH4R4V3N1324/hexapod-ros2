#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

using geometry_msgs::msg::Twist;
using sensor_msgs::msg::BatteryState;

/**
 * @brief Stand-in for the daughter boards' INA226 power monitors in simulation
 * @note Publishes sensor_msgs/BatteryState on power/<location> like ina226_node, so the panel and
 *       behaviour manager work the same in the sim. The battery voltage is a parameter, which can be
 *       set while running (ros2 param set /sim_power_monitor voltage 9.8) or left to drain at
 *       drain_rate. Each board draws idle_current, or walking_current while /cmd_vel is non-zero,
 *       and the voltage sags by the total current times internal_resistance.
*/
class SimPowerMonitor : public rclcpp::Node {
public:
    /**
     * @brief Declare the node's params, create a publisher per board and start the publish timer
    */
    SimPowerMonitor() : Node("sim_power_monitor") {
        auto locations = declare_parameter("locations", std::vector<std::string>{"left", "right"});
        declare_parameter("voltage", 12.4);
        declare_parameter("drain_rate", 0.0);
        idle_current = declare_parameter("idle_current", 0.8);
        walking_current = declare_parameter("walking_current", 3.0);
        internal_resistance = declare_parameter("internal_resistance", 0.05);
        full_voltage = declare_parameter("battery_full_voltage", 12.6);
        empty_voltage = declare_parameter("battery_empty_voltage", 9.0);
        double rate = declare_parameter("rate", 10.0);

        for (const auto& location : locations) {
            pubs.emplace_back(location, create_publisher<BatteryState>("power/" + location, 10));
        }
        cmd_vel_sub = create_subscription<Twist>("cmd_vel", 10, [this](const Twist::SharedPtr msg) {
            walking = msg->linear.x != 0.0 || msg->linear.y != 0.0 || msg->angular.z != 0.0;
        });

        period = 1.0 / rate;
        timer = create_timer(std::chrono::duration<double>(period), std::bind(&SimPowerMonitor::update, this));
    }

private:
    /**
     * @brief Drain the battery and publish each board's reading
    */
    void update() {
        double voltage = get_parameter("voltage").as_double();
        const double drain_rate = get_parameter("drain_rate").as_double();
        if (drain_rate > 0.0) {
            voltage = std::max(0.0, voltage - drain_rate * period);
            set_parameter(rclcpp::Parameter("voltage", voltage));
        }

        const double board_current = walking ? walking_current : idle_current;
        const double loaded = voltage - board_current * pubs.size() * internal_resistance;
        const double nan = std::numeric_limits<double>::quiet_NaN();

        for (const auto& [location, pub] : pubs) {
            BatteryState msg;
            msg.header.stamp = now();
            msg.voltage = static_cast<float>(loaded);
            msg.current = static_cast<float>(-board_current);
            msg.charge = nan;
            msg.capacity = nan;
            msg.design_capacity = nan;
            msg.percentage = static_cast<float>(
                std::clamp((loaded - empty_voltage) / (full_voltage - empty_voltage), 0.0, 1.0));
            msg.temperature = nan;
            msg.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
            msg.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
            msg.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
            msg.present = true;
            msg.location = location;
            pub->publish(msg);
        }
    }

    std::vector<std::pair<std::string, rclcpp::Publisher<BatteryState>::SharedPtr>> pubs;
    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub;
    rclcpp::TimerBase::SharedPtr timer;
    bool walking = false;
    double period;               // s between readings
    double idle_current;         // A per board standing still
    double walking_current;      // A per board while walking
    double internal_resistance;  // ohms, battery and wiring
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
    rclcpp::spin(std::make_shared<SimPowerMonitor>());
    rclcpp::shutdown();
    return 0;
}
