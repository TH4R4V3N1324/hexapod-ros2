#include "hexapod_hardware_interface/st3215_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace hexapod_hardware_interface {

using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

namespace {

/**
 * @brief Look up a ros2_control param, falling back to a default
 * @param[in] params The params parsed from the URDF
 * @param[in] key The param name
 * @param[in] fallback The value to use if the param is missing
 * @return The param value
*/
std::string param(
    const std::unordered_map<std::string, std::string>& params,
    const std::string& key,
    const std::string& fallback
) {
    auto it = params.find(key);
    return (it != params.end()) ? it->second : fallback;
}

}  // namespace

/**
 * @brief Parse the hardware and joint params from the URDF
 * @param[in] params The hardware component params, including the parsed URDF info
 * @return SUCCESS if every joint is configured correctly, ERROR otherwise
*/
CallbackReturn ST3215System::on_init(const hardware_interface::HardwareComponentInterfaceParams& params) {
    if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    try {
        const auto& hw = info_.hardware_parameters;
        serial_port = param(hw, "serial_port", "/dev/ttyAMA0");
        baud_rate = std::stoi(param(hw, "baud_rate", "1000000"));
        echo = param(hw, "echo", "false") == "true";
        max_torque = std::stod(param(hw, "max_torque", "2.94"));
        acceleration = static_cast<uint8_t>(std::stoi(param(hw, "acceleration", "0")));
        speed = static_cast<uint16_t>(std::stoi(param(hw, "speed", "0")));

        for (const auto& joint : info_.joints) {
            if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != HW_IF_POSITION) {
                RCLCPP_ERROR(get_logger(), "Joint '%s' must have a single position command interface", joint.name.c_str());
                return CallbackReturn::ERROR;
            }
            for (const char* name : {HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_EFFORT}) {
                bool found = std::any_of(joint.state_interfaces.begin(), joint.state_interfaces.end(),
                    [name](const auto& interface) { return interface.name == name; });
                if (!found) {
                    RCLCPP_ERROR(get_logger(), "Joint '%s' is missing the %s state interface", joint.name.c_str(), name);
                    return CallbackReturn::ERROR;
                }
            }
            if (joint.parameters.find("servo_id") == joint.parameters.end()) {
                RCLCPP_ERROR(get_logger(), "Joint '%s' is missing the servo_id param", joint.name.c_str());
                return CallbackReturn::ERROR;
            }

            ST3215Joint servo;
            servo.name = joint.name;
            servo.id = static_cast<uint8_t>(std::stoi(joint.parameters.at("servo_id")));
            servo.direction = std::stod(param(joint.parameters, "direction", "1")) < 0 ? -1.0 : 1.0;
            servo.offset = std::stod(param(joint.parameters, "offset", "0"));
            joints.push_back(servo);
            ids.push_back(servo.id);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Invalid ST3215 param: %s", e.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

/**
 * @brief Open the serial bus and check which servos respond
 * @param[in] previous_state The lifecycle state before configuring
 * @return SUCCESS if the port opened, ERROR otherwise
 * @note Missing servos are logged but do not stop the hardware from configuring
*/
CallbackReturn ST3215System::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    if (!bus.open(serial_port, baud_rate, echo)) {
        RCLCPP_ERROR(get_logger(), "Failed to open %s at %d baud", serial_port.c_str(), baud_rate);
        return CallbackReturn::ERROR;
    }

    int found = 0;
    for (const auto& joint : joints) {
        if (bus.ping(joint.id)) {
            found++;
        } else {
            RCLCPP_WARN(get_logger(), "Servo %d (%s) did not respond", joint.id, joint.name.c_str());
        }
    }
    RCLCPP_INFO(get_logger(), "Opened %s, %d/%zu servos responding", serial_port.c_str(), found, joints.size());
    return CallbackReturn::SUCCESS;
}

/**
 * @brief Hold every servo at its current position and enable torque
 * @param[in] previous_state The lifecycle state before activating
 * @return SUCCESS once torque is enabled, ERROR if the bus write fails
 * @note Goal positions are set to the current positions first, so the legs do not jump
*/
CallbackReturn ST3215System::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    std::vector<std::optional<ST3215State>> states;
    bus.readStates(ids, states);

    std::vector<uint8_t> hold_ids;
    std::vector<int> hold_positions;
    for (size_t i = 0; i < joints.size(); ++i) {
        const auto& joint = joints[i];
        if (!states[i]) {
            RCLCPP_WARN(get_logger(), "No state from servo %d (%s), leaving torque off", joint.id, joint.name.c_str());
            set_command(joint.name + "/" + HW_IF_POSITION, std::numeric_limits<double>::quiet_NaN());
            continue;
        }
        double position = stepsToRadians(joint, states[i]->position);
        set_state(joint.name + "/" + HW_IF_POSITION, position);
        set_command(joint.name + "/" + HW_IF_POSITION, position);
        hold_ids.push_back(joint.id);
        hold_positions.push_back(states[i]->position);
    }

    if (!hold_ids.empty()) {
        if (!bus.writeGoalPositions(hold_ids, hold_positions, acceleration, speed) ||
            !bus.setTorqueEnabled(hold_ids, true)) {
            RCLCPP_ERROR(get_logger(), "Failed to enable servo torque");
            return CallbackReturn::ERROR;
        }
    }
    RCLCPP_INFO(get_logger(), "Torque enabled on %zu servos", hold_ids.size());
    return CallbackReturn::SUCCESS;
}

/**
 * @brief Disable torque on every servo
 * @param[in] previous_state The lifecycle state before deactivating
 * @return SUCCESS
 * @note The legs go limp, so the robot will drop if it is standing
*/
CallbackReturn ST3215System::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    bus.setTorqueEnabled(ids, false);
    RCLCPP_INFO(get_logger(), "Torque disabled");
    return CallbackReturn::SUCCESS;
}

/**
 * @brief Close the serial bus
 * @param[in] previous_state The lifecycle state before cleaning up
 * @return SUCCESS
*/
CallbackReturn ST3215System::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
    bus.close();
    return CallbackReturn::SUCCESS;
}

/**
 * @brief Read position, velocity and load from every servo
 * @param[in] time The current time
 * @param[in] period The time since the last read
 * @return OK, missing replies keep the joint's previous state
*/
return_type ST3215System::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    std::vector<std::optional<ST3215State>> states;
    bus.readStates(ids, states);

    for (size_t i = 0; i < joints.size(); ++i) {
        const auto& joint = joints[i];
        if (!states[i]) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No reply from servo %d (%s)", joint.id, joint.name.c_str());
            continue;
        }
        if (states[i]->error != 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Servo %d (%s) reports error 0x%02X", joint.id, joint.name.c_str(), states[i]->error);
        }
        set_state(joint.name + "/" + HW_IF_POSITION, stepsToRadians(joint, states[i]->position));
        set_state(joint.name + "/" + HW_IF_VELOCITY, joint.direction * states[i]->speed * 2.0 * M_PI / ST3215_STEPS_PER_REV);
        set_state(joint.name + "/" + HW_IF_EFFORT, joint.direction * states[i]->load / 1000.0 * max_torque);
    }
    return return_type::OK;
}

/**
 * @brief Send the commanded positions to every servo in one sync write
 * @param[in] time The current time
 * @param[in] period The time since the last write
 * @return OK if the packet was sent, ERROR otherwise
 * @note Joints with no command yet (NaN) are skipped
*/
return_type ST3215System::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    std::vector<uint8_t> write_ids;
    std::vector<int> positions;
    for (const auto& joint : joints) {
        double command = get_command<double>(joint.name + "/" + HW_IF_POSITION);
        if (!std::isfinite(command)) {
            continue;
        }
        write_ids.push_back(joint.id);
        positions.push_back(radiansToSteps(joint, command));
    }
    if (write_ids.empty()) {
        return return_type::OK;
    }
    return bus.writeGoalPositions(write_ids, positions, acceleration, speed) ? return_type::OK : return_type::ERROR;
}

/**
 * @brief Convert a servo position to a joint angle
 * @param[in] joint The joint's servo mapping
 * @param[in] steps The servo position in steps
 * @return The joint angle in radians
*/
double ST3215System::stepsToRadians(const ST3215Joint& joint, int steps) const {
    return joint.direction * (steps - ST3215_CENTER_POSITION) * 2.0 * M_PI / ST3215_STEPS_PER_REV + joint.offset;
}

/**
 * @brief Convert a joint angle to a servo position
 * @param[in] joint The joint's servo mapping
 * @param[in] radians The joint angle in radians
 * @return The servo position in steps, clamped to 0-4095
*/
int ST3215System::radiansToSteps(const ST3215Joint& joint, double radians) const {
    double steps = joint.direction * (radians - joint.offset) * ST3215_STEPS_PER_REV / (2.0 * M_PI) + ST3215_CENTER_POSITION;
    return std::clamp(static_cast<int>(std::lround(steps)), 0, ST3215_STEPS_PER_REV - 1);
}

}  // namespace hexapod_hardware_interface

PLUGINLIB_EXPORT_CLASS(hexapod_hardware_interface::ST3215System, hardware_interface::SystemInterface)
