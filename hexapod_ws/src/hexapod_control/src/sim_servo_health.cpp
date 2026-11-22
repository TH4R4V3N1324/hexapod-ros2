#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <control_msgs/msg/interface_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

using control_msgs::msg::DynamicJointState;
using control_msgs::msg::InterfaceValue;
using sensor_msgs::msg::BatteryState;
using sensor_msgs::msg::JointState;

namespace {

constexpr uint8_t ERROR_OVERHEAT = 1 << 2;  // ST3215 status error bit
constexpr double OVERHEAT_TEMPERATURE = 70.0;  // °C, the servos' default max_temperature

}  // namespace

/**
 * @brief Stand-in for the ST3215 servos' health readings in simulation
 * @note Gazebo's joints have no temperature or supply voltage, so this models them, and publishes a
 *       control_msgs/DynamicJointState with the same interfaces ST3215System gives each joint on the
 *       robot (position, velocity, effort from /joint_states, plus temperature, voltage, current and
 *       servo_error). The simulation launch points servo_monitor at it, so /diagnostics and the
 *       Foxglove panel can be tried without the robot.
 *       - current: idle_current plus stall_current in proportion to |effort| / stall_torque
 *       - temperature: a first-order thermal model, heated by current² × winding_resistance through
 *         thermal_resistance to ambient_temperature, with time constant thermal_time_constant.
 *         time_scale speeds it up, e.g. 30 to see a long walk's heating in a minute.
 *       - voltage: the side's /power/<location> voltage (sim_power_monitor), less current ×
 *         wire_resistance. Legs 1-3 are on the right board, 4-6 on the left.
 *       - servo_error: the overheat bit at 70 °C
 *       Faults can be injected while running, for testing warnings and errors:
 *         ros2 param set /sim_servo_health hot_joints "['leg2_femur_joint']"   (+hot_offset °C)
 *         ros2 param set /sim_servo_health silent_joints "['leg4_coxa_joint']" (stops answering)
 *       While /servo_power is false every servo stops answering, like the real ones.
*/
class SimServoHealth : public rclcpp::Node {
public:
    SimServoHealth() : Node("sim_servo_health") {
        ambient = declare_parameter("ambient_temperature", 25.0);
        thermal_resistance = declare_parameter("thermal_resistance", 9.0);       // K/W
        thermal_time_constant = declare_parameter("thermal_time_constant", 300.0);  // s
        winding_resistance = declare_parameter("winding_resistance", 2.0);       // Ω
        idle_current = declare_parameter("idle_current", 0.1);                   // A
        stall_current = declare_parameter("stall_current", 2.7);                 // A at stall_torque
        stall_torque = declare_parameter("stall_torque", 2.94);                  // N·m
        wire_resistance = declare_parameter("wire_resistance", 0.1);             // Ω
        declare_parameter("time_scale", 1.0);
        declare_parameter("hot_joints", std::vector<std::string>{""});
        declare_parameter("hot_offset", 20.0);                                   // °C
        declare_parameter("silent_joints", std::vector<std::string>{""});
        const double rate = declare_parameter("rate", 50.0);

        pub = create_publisher<DynamicJointState>("sim/dynamic_joint_states", 10);
        joint_states_sub = create_subscription<JointState>("joint_states", 10,
            [this](JointState::SharedPtr msg) { latest = msg; });
        for (const char* side : {"left", "right"}) {
            const std::string location = side;
            power_subs.push_back(create_subscription<BatteryState>("power/" + location, 10,
                [this, location](BatteryState::SharedPtr msg) { board_voltage[location] = msg->voltage; }));
        }
        servo_power_sub = create_subscription<std_msgs::msg::Bool>("servo_power", rclcpp::QoS(1).transient_local(),
            [this](std_msgs::msg::Bool::SharedPtr msg) { servo_power = msg->data; });

        period = 1.0 / rate;
        timer = create_timer(std::chrono::duration<double>(period), [this] { update(); });
    }

private:
    static std::set<std::string> nameSet(const std::vector<std::string>& names) {
        std::set<std::string> set;
        for (const auto& name : names) {
            if (!name.empty()) {
                set.insert(name);
            }
        }
        return set;
    }

    /**
     * @brief Advance each servo's temperature and publish the readings
    */
    void update() {
        if (!latest) {
            return;
        }
        const double dt = period * get_parameter("time_scale").as_double();
        const auto hot = nameSet(get_parameter("hot_joints").as_string_array());
        const double hot_offset = get_parameter("hot_offset").as_double();
        const auto silent = nameSet(get_parameter("silent_joints").as_string_array());
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double capacity = thermal_time_constant / thermal_resistance;  // J/K

        const auto& js = *latest;
        DynamicJointState msg;
        msg.header.stamp = now();
        for (size_t i = 0; i < js.name.size(); ++i) {
            const std::string& joint = js.name[i];
            const double position = i < js.position.size() ? js.position[i] : nan;
            const double velocity = i < js.velocity.size() ? js.velocity[i] : nan;
            const double effort = i < js.effort.size() ? js.effort[i] : 0.0;

            const double current = idle_current + stall_current * std::min(std::abs(effort) / stall_torque, 1.5);
            // The windings keep heating (and cooling) whether or not anyone is listening
            double& temperature = temperatures.try_emplace(joint, ambient).first->second;
            const double heating = current * current * winding_resistance;
            temperature += dt * (heating - (temperature - ambient) / thermal_resistance) / capacity;

            InterfaceValue iv;
            iv.interface_names = {"position", "velocity", "effort", "temperature", "voltage", "current", "servo_error"};
            if (!servo_power || silent.count(joint) > 0) {
                iv.values = {nan, nan, nan, nan, nan, nan, nan};
            } else {
                const double reported = temperature + (hot.count(joint) > 0 ? hot_offset : 0.0);
                // Leg 1-3 are on the right board, 4-6 on the left
                const bool right = joint.size() > 3 && joint[3] >= '1' && joint[3] <= '3';
                const auto board = board_voltage.find(right ? "right" : "left");
                const double supply = board != board_voltage.end() ? board->second : 12.4;
                iv.values = {
                    position, velocity, effort,
                    std::round(reported),  // the servo reports whole degrees
                    std::round((supply - current * wire_resistance) * 10.0) / 10.0,  // and 0.1 V steps
                    current,
                    reported >= OVERHEAT_TEMPERATURE ? static_cast<double>(ERROR_OVERHEAT) : 0.0,
                };
            }
            msg.joint_names.push_back(joint);
            msg.interface_values.push_back(iv);
        }
        pub->publish(msg);
    }

    double ambient, thermal_resistance, thermal_time_constant, winding_resistance;
    double idle_current, stall_current, stall_torque, wire_resistance;
    double period;
    bool servo_power = true;
    std::map<std::string, double> temperatures;
    std::map<std::string, double> board_voltage;
    JointState::SharedPtr latest;

    rclcpp::Publisher<DynamicJointState>::SharedPtr pub;
    rclcpp::Subscription<JointState>::SharedPtr joint_states_sub;
    std::vector<rclcpp::Subscription<BatteryState>::SharedPtr> power_subs;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servo_power_sub;
    rclcpp::TimerBase::SharedPtr timer;
};

/**
 * @brief Entry point, initialises ROS and spins the node
*/
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimServoHealth>());
    rclcpp::shutdown();
    return 0;
}
