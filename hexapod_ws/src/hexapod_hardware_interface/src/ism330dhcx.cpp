#include "hexapod_hardware_interface/ism330dhcx.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

namespace hexapod_hardware_interface {

namespace {

// ISM330DHCX registers (the older ISM330DLC shares this layout)
constexpr uint8_t WHO_AM_I = 0x0F;
constexpr uint8_t CTRL1_XL = 0x10;
constexpr uint8_t CTRL2_G = 0x11;
constexpr uint8_t CTRL3_C = 0x12;
constexpr uint8_t OUTX_L_G = 0x22;  // gyro x, y, z then accel x, y, z, 12 bytes

constexpr uint8_t ISM330DHCX_ID = 0x6B;
constexpr uint8_t ISM330DLC_ID = 0x6A;

constexpr uint8_t CTRL3_SW_RESET = 0x01;
constexpr uint8_t CTRL3_BDU_IF_INC = 0x44;  // block data update, auto-increment register address
constexpr double GRAVITY = 9.80665;

}  // namespace

/**
 * @brief Open the bus, reset the sensor and configure its ranges and output rate
 * @param[in] bus The I2C bus device, e.g. /dev/i2c-1
 * @param[in] address The sensor's I2C address (0x6A, or 0x6B with the address jumper bridged)
 * @param[in] accel_range_g Accelerometer full scale: 2, 4, 8 or 16 g
 * @param[in] gyro_range_dps Gyroscope full scale: 125, 250, 500, 1000, 2000 or 4000 °/s
 * @param[in] rate How often the sensor will be read, in Hz
 * @param[out] error Why initialisation failed, if it did
 * @return true if the sensor was found and configured, false otherwise
 * @note The output data rate is set to the slowest step at or above rate, so every read
 *       gets a fresh, anti-alias filtered sample
*/
bool Ism330dhcx::initialise(
    const std::string& bus,
    uint8_t address,
    int accel_range_g,
    int gyro_range_dps,
    double rate,
    std::string& error
) {
    char text[96];
    if (!device_.open(bus, address)) {
        error = "Failed to open " + bus;
        return false;
    }

    // FS_XL bits [3:2] of CTRL1_XL, mg/LSB from the datasheet
    uint8_t accel_range_code;
    double accel_mg_per_lsb;
    switch (accel_range_g) {
        case 2: accel_range_code = 0x00; accel_mg_per_lsb = 0.061; break;
        case 4: accel_range_code = 0x08; accel_mg_per_lsb = 0.122; break;
        case 8: accel_range_code = 0x0C; accel_mg_per_lsb = 0.244; break;
        case 16: accel_range_code = 0x04; accel_mg_per_lsb = 0.488; break;
        default:
            error = "accel_range_g must be 2, 4, 8 or 16";
            return false;
    }
    // FS_G bits [3:0] of CTRL2_G, mdps/LSB from the datasheet
    uint8_t gyro_range_code;
    double gyro_mdps_per_lsb;
    switch (gyro_range_dps) {
        case 125: gyro_range_code = 0x02; gyro_mdps_per_lsb = 4.375; break;
        case 250: gyro_range_code = 0x00; gyro_mdps_per_lsb = 8.75; break;
        case 500: gyro_range_code = 0x04; gyro_mdps_per_lsb = 17.5; break;
        case 1000: gyro_range_code = 0x08; gyro_mdps_per_lsb = 35.0; break;
        case 2000: gyro_range_code = 0x0C; gyro_mdps_per_lsb = 70.0; break;
        case 4000: gyro_range_code = 0x01; gyro_mdps_per_lsb = 140.0; break;
        default:
            error = "gyro_range_dps must be 125, 250, 500, 1000, 2000 or 4000";
            return false;
    }
    // ODR bits [7:4]: 1 = 12.5 Hz, doubling up to 8 = 1666 Hz
    uint8_t odr_code = 1;
    double odr_hz = 12.5;
    while (odr_hz < rate && odr_code < 8) {
        odr_code++;
        odr_hz *= 2.0;
    }

    device_.writeRegister8(CTRL3_C, CTRL3_SW_RESET);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    uint8_t id = 0;
    if (!device_.readRegister8(WHO_AM_I, id) || (id != ISM330DHCX_ID && id != ISM330DLC_ID)) {
        snprintf(text, sizeof(text), "ISM330DHCX not found at 0x%02X (WHO_AM_I 0x%02X)", address, id);
        error = text;
        return false;
    }
    chip_name_ = id == ISM330DHCX_ID ? "ISM330DHCX" : "ISM330DLC";
    if (id == ISM330DLC_ID && gyro_range_dps == 4000) {
        error = "The ISM330DLC has no ±4000 °/s range";
        return false;
    }

    bool ok = device_.writeRegister8(CTRL3_C, CTRL3_BDU_IF_INC);
    ok = ok && device_.writeRegister8(CTRL1_XL, static_cast<uint8_t>(odr_code << 4) | accel_range_code);
    ok = ok && device_.writeRegister8(CTRL2_G, static_cast<uint8_t>(odr_code << 4) | gyro_range_code);
    if (!ok) {
        error = std::string("Failed to configure ") + chip_name_;
        return false;
    }

    output_rate_ = odr_hz;
    accel_scale_ = accel_mg_per_lsb / 1000.0 * GRAVITY;
    gyro_scale_ = gyro_mdps_per_lsb / 1000.0 * M_PI / 180.0;
    return true;
}

/**
 * @brief Read the latest accelerometer and gyroscope sample
 * @param[out] sample The reading in m/s² and rad/s
 * @return true if the read succeeded, false otherwise
*/
bool Ism330dhcx::read(ImuSample& sample) {
    uint8_t raw[12];
    if (!device_.readRegisters(OUTX_L_G, raw, 12)) {
        return false;
    }
    for (int i = 0; i < 3; ++i) {
        sample.gyro[i] = static_cast<int16_t>(raw[2 * i] | (raw[2 * i + 1] << 8)) * gyro_scale_;
        sample.accel[i] = static_cast<int16_t>(raw[6 + 2 * i] | (raw[7 + 2 * i] << 8)) * accel_scale_;
    }
    return true;
}

}  // namespace hexapod_hardware_interface
