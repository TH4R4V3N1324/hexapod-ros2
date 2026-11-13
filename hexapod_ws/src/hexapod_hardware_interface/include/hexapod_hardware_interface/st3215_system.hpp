#ifndef HEXAPOD_HARDWARE_INTERFACE__ST3215_SYSTEM_HPP_
#define HEXAPOD_HARDWARE_INTERFACE__ST3215_SYSTEM_HPP_

#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "hexapod_hardware_interface/st3215_bus.hpp"

namespace hexapod_hardware_interface {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

/**
 * @brief Per-joint servo mapping, parsed from the joint's ros2_control params
*/
struct ST3215Joint {
    std::string name;
    uint8_t id;        // servo ID on the bus
    double direction;  // 1 or -1, flips the servo's rotation to match the joint
    double offset;     // joint angle (rad) when the servo is at its centre position

    // Whether it's answering. Unpowered servos (SERVO_EN low, E-stop open) don't; their state reads NaN.
    bool answering = false;
    int missed = 0;            // replies missed in a row
    // A servo that starts answering again holds where it is, ignoring the command it had before,
    // until a new one arrives (the gait controller sends nothing while the servos are off)
    bool holding = false;
    double held_command = 0.0;
};

/**
 * @brief ros2_control system for the 18 leg joints, driven by Waveshare ST3215 bus servos
 * @note Exposes position commands and position, velocity and effort states.
 *       Effort is the servo's load reading scaled by the max_torque hardware param.
 *       A servo that stops answering (its power is off: SERVO_EN low or the E-stop open) reads
 *       NaN, which the gait controller takes as missing. When it answers again it is held where it
 *       is with torque on until a new command arrives, so it never jumps to a stale command.
*/
class ST3215System : public hardware_interface::SystemInterface {
public:
    CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    double stepsToRadians(const ST3215Joint& joint, int steps) const;
    int radiansToSteps(const ST3215Joint& joint, double radians) const;
    void setStateUnknown(const ST3215Joint& joint);

    ST3215Bus bus;
    std::vector<ST3215Joint> joints;
    std::vector<uint8_t> ids;
    size_t next_probe = 0;   // which silent servo to try next: one a cycle, so unpowered servos don't stall the loop

    std::string serial_port;
    int baud_rate = 1000000;
    bool echo = false;
    double max_torque = 2.94;   // N·m, ST3215 stall torque at 12 V
    uint8_t acceleration = 0;
    uint16_t speed = 0;
};

}  // namespace hexapod_hardware_interface

#endif  // HEXAPOD_HARDWARE_INTERFACE__ST3215_SYSTEM_HPP_
