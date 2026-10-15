#include "hexapod_hardware_interface/ina226.hpp"

namespace hexapod_hardware_interface {

namespace {

// INA226 registers (16-bit, big-endian)
constexpr uint8_t INA226_CONFIG = 0x00;
constexpr uint8_t INA226_SHUNT_VOLTAGE = 0x01;
constexpr uint8_t INA226_BUS_VOLTAGE = 0x02;
constexpr uint8_t INA226_MANUFACTURER_ID = 0xFE;
constexpr uint8_t INA226_DIE_ID = 0xFF;
constexpr uint16_t INA226_MANUFACTURER_ID_VALUE = 0x5449;  // "TI"

// 16-sample averaging, 1.1 ms conversions, continuous shunt and bus
constexpr uint16_t INA226_CONFIG_VALUE = 0x4527;
constexpr double SHUNT_VOLTAGE_LSB = 2.5e-6;  // V
constexpr double BUS_VOLTAGE_LSB = 1.25e-3;   // V

}  // namespace

/**
 * @brief Open the bus, check the manufacturer ID and configure the INA226
 * @param[in] bus The I2C bus device, e.g. /dev/i2c-1
 * @param[in] address The INA226's I2C address
 * @return true if the INA226 was found and configured, false otherwise
*/
bool Ina226::initialise(const std::string& bus, uint8_t address) {
    uint16_t id = 0;
    return device_.open(bus, address) &&
        device_.readRegister16BE(INA226_MANUFACTURER_ID, id) &&
        id == INA226_MANUFACTURER_ID_VALUE &&
        device_.writeRegister16BE(INA226_CONFIG, INA226_CONFIG_VALUE);
}

/**
 * @brief Read the bus voltage and shunt current
 * @param[in] shunt_resistance The shunt resistor, in ohms
 * @param[out] voltage The bus voltage (IN- to GND), in volts
 * @param[out] current The current from IN+ to IN-, in amps (positive while the load draws power)
 * @return true if the read succeeded, false otherwise
*/
bool Ina226::read(double shunt_resistance, double& voltage, double& current) {
    uint16_t shunt_raw = 0;
    uint16_t bus_raw = 0;
    if (!device_.readRegister16BE(INA226_SHUNT_VOLTAGE, shunt_raw) ||
        !device_.readRegister16BE(INA226_BUS_VOLTAGE, bus_raw)) {
        return false;
    }
    voltage = bus_raw * BUS_VOLTAGE_LSB;
    current = static_cast<int16_t>(shunt_raw) * SHUNT_VOLTAGE_LSB / shunt_resistance;
    return true;
}

/**
 * @brief Read the die ID register
 * @param[out] id The die ID (0x2260 on a genuine INA226)
 * @return true if the read succeeded, false otherwise
*/
bool Ina226::readDieId(uint16_t& id) {
    return device_.readRegister16BE(INA226_DIE_ID, id);
}

}  // namespace hexapod_hardware_interface
