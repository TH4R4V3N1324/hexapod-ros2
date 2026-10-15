#ifndef HEXAPOD_HARDWARE_INTERFACE__ISM330DHCX_HPP_
#define HEXAPOD_HARDWARE_INTERFACE__ISM330DHCX_HPP_

#include <cstdint>
#include <string>

#include "hexapod_hardware_interface/i2c_device.hpp"

namespace hexapod_hardware_interface {

/**
 * @brief One accelerometer and gyroscope reading, in the sensor's axes
*/
struct ImuSample {
    double accel[3];  // m/s²
    double gyro[3];   // rad/s
};

/**
 * @brief I2C driver for the ISM330DHCX IMU (accelerometer + gyroscope)
 * @note Also accepts the older ISM330DLC, which shares the register layout and
 *       which Adafruit sold on the same style of board.
*/
class Ism330dhcx {
public:
    bool initialise(
        const std::string& bus,
        uint8_t address,
        int accel_range_g,
        int gyro_range_dps,
        double rate,
        std::string& error
    );
    bool read(ImuSample& sample);

    const char* chipName() const { return chip_name_; }
    double outputRate() const { return output_rate_; }

private:
    I2CDevice device_;
    const char* chip_name_ = "ISM330DHCX";
    double output_rate_ = 0.0;
    double accel_scale_ = 0.0;
    double gyro_scale_ = 0.0;
};

}  // namespace hexapod_hardware_interface

#endif  // HEXAPOD_HARDWARE_INTERFACE__ISM330DHCX_HPP_
