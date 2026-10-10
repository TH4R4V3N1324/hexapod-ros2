#ifndef HEXAPOD_HARDWARE_INTERFACE__I2C_DEVICE_HPP_
#define HEXAPOD_HARDWARE_INTERFACE__I2C_DEVICE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace hexapod_hardware_interface {

/**
 * @brief Register access to one device on a Linux I2C bus (/dev/i2c-N)
*/
class I2CDevice {
public:
    ~I2CDevice();

    bool open(const std::string& bus, uint8_t address);
    void close();
    bool isOpen() const { return fd_ >= 0; }

    bool readRegisters(uint8_t reg, uint8_t* data, size_t length);
    bool writeRegister(uint8_t reg, const uint8_t* data, size_t length);
    bool readRegister8(uint8_t reg, uint8_t& value);
    bool writeRegister8(uint8_t reg, uint8_t value);
    bool readRegister16BE(uint8_t reg, uint16_t& value);
    bool writeRegister16BE(uint8_t reg, uint16_t value);

private:
    int fd_ = -1;
    uint8_t address_ = 0;
};

}  // namespace hexapod_hardware_interface

#endif  // HEXAPOD_HARDWARE_INTERFACE__I2C_DEVICE_HPP_
