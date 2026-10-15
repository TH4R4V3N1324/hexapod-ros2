#ifndef HEXAPOD_HARDWARE_INTERFACE__INA226_HPP_
#define HEXAPOD_HARDWARE_INTERFACE__INA226_HPP_

#include <cstdint>
#include <string>

#include "hexapod_hardware_interface/i2c_device.hpp"

namespace hexapod_hardware_interface {

/**
 * @brief I2C driver for an INA226 power monitor
*/
class Ina226 {
public:
    bool initialise(const std::string& bus, uint8_t address);
    bool read(double shunt_resistance, double& voltage, double& current);
    bool readDieId(uint16_t& id);

private:
    I2CDevice device_;
};

}  // namespace hexapod_hardware_interface

#endif  // HEXAPOD_HARDWARE_INTERFACE__INA226_HPP_
