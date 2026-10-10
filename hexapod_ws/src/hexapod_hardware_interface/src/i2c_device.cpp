#include "hexapod_hardware_interface/i2c_device.hpp"

#include <algorithm>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

namespace hexapod_hardware_interface {

/**
 * @brief Close the bus when the device is destroyed
*/
I2CDevice::~I2CDevice() {
    close();
}

/**
 * @brief Open the I2C bus for a device
 * @param[in] bus The bus device, e.g. /dev/i2c-1
 * @param[in] address The 7-bit device address
 * @return true if the bus was opened, false otherwise
*/
bool I2CDevice::open(const std::string& bus, uint8_t address) {
    close();
    fd_ = ::open(bus.c_str(), O_RDWR);
    address_ = address;
    return fd_ >= 0;
}

/**
 * @brief Close the bus if it is open
*/
void I2CDevice::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

/**
 * @brief Read consecutive registers in one combined write-then-read transaction
 * @param[in] reg The first register to read
 * @param[out] data Where to store the register values
 * @param[in] length The number of bytes to read
 * @return true if the transfer succeeded, false otherwise
*/
bool I2CDevice::readRegisters(uint8_t reg, uint8_t* data, size_t length) {
    i2c_msg msgs[2] = {
        {address_, 0, 1, &reg},
        {address_, I2C_M_RD, static_cast<uint16_t>(length), data},
    };
    i2c_rdwr_ioctl_data transfer{msgs, 2};
    return fd_ >= 0 && ioctl(fd_, I2C_RDWR, &transfer) == 2;
}

/**
 * @brief Write consecutive registers
 * @param[in] reg The first register to write
 * @param[in] data The register values
 * @param[in] length The number of bytes to write
 * @return true if the transfer succeeded, false otherwise
*/
bool I2CDevice::writeRegister(uint8_t reg, const uint8_t* data, size_t length) {
    std::vector<uint8_t> buffer(length + 1);
    buffer[0] = reg;
    std::copy(data, data + length, buffer.begin() + 1);
    i2c_msg msg{address_, 0, static_cast<uint16_t>(buffer.size()), buffer.data()};
    i2c_rdwr_ioctl_data transfer{&msg, 1};
    return fd_ >= 0 && ioctl(fd_, I2C_RDWR, &transfer) == 1;
}

/**
 * @brief Read an 8-bit register
 * @param[in] reg The register
 * @param[out] value The register value
 * @return true if the transfer succeeded, false otherwise
*/
bool I2CDevice::readRegister8(uint8_t reg, uint8_t& value) {
    return readRegisters(reg, &value, 1);
}

/**
 * @brief Write an 8-bit register
 * @param[in] reg The register
 * @param[in] value The value to write
 * @return true if the transfer succeeded, false otherwise
*/
bool I2CDevice::writeRegister8(uint8_t reg, uint8_t value) {
    return writeRegister(reg, &value, 1);
}

/**
 * @brief Read a big-endian 16-bit register
 * @param[in] reg The register
 * @param[out] value The register value
 * @return true if the transfer succeeded, false otherwise
*/
bool I2CDevice::readRegister16BE(uint8_t reg, uint16_t& value) {
    uint8_t bytes[2];
    if (!readRegisters(reg, bytes, 2)) {
        return false;
    }
    value = static_cast<uint16_t>((bytes[0] << 8) | bytes[1]);
    return true;
}

/**
 * @brief Write a big-endian 16-bit register
 * @param[in] reg The register
 * @param[in] value The value to write
 * @return true if the transfer succeeded, false otherwise
*/
bool I2CDevice::writeRegister16BE(uint8_t reg, uint16_t value) {
    uint8_t bytes[2] = {static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
    return writeRegister(reg, bytes, 2);
}

}  // namespace hexapod_hardware_interface
