#ifndef HEXAPOD_HARDWARE_INTERFACE__ST3215_BUS_HPP_
#define HEXAPOD_HARDWARE_INTERFACE__ST3215_BUS_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace hexapod_hardware_interface {

// Feetech STS protocol instructions
static constexpr uint8_t ST3215_INST_PING = 0x01;
static constexpr uint8_t ST3215_INST_READ = 0x02;
static constexpr uint8_t ST3215_INST_WRITE = 0x03;
static constexpr uint8_t ST3215_INST_SYNC_READ = 0x82;
static constexpr uint8_t ST3215_INST_SYNC_WRITE = 0x83;
static constexpr uint8_t ST3215_BROADCAST_ID = 0xFE;

// ST3215 control table (multi-byte values are little-endian)
static constexpr uint8_t ST3215_REG_ID = 5;                    // EEPROM, unlock with ST3215_REG_LOCK first
static constexpr uint8_t ST3215_REG_MIN_ANGLE_LIMIT = 9;
static constexpr uint8_t ST3215_REG_MAX_ANGLE_LIMIT = 11;
static constexpr uint8_t ST3215_REG_TORQUE_ENABLE = 40;
static constexpr uint8_t ST3215_REG_ACCELERATION = 41;
static constexpr uint8_t ST3215_REG_GOAL_POSITION = 42;
static constexpr uint8_t ST3215_REG_LOCK = 55;                 // 0 = EEPROM writable, 1 = locked
static constexpr uint8_t ST3215_REG_PRESENT_POSITION = 56;
static constexpr uint8_t ST3215_REG_PRESENT_VOLTAGE = 62;      // 0.1 V
static constexpr uint8_t ST3215_REG_PRESENT_TEMPERATURE = 63;  // °C
static constexpr uint8_t ST3215_REG_MOVING = 66;

static constexpr int ST3215_STEPS_PER_REV = 4096;
static constexpr int ST3215_CENTER_POSITION = 2048;

/**
 * @brief Raw feedback read from one servo
*/
struct ST3215State {
    int position;   // steps, 0-4095
    int speed;      // steps/s, signed
    int load;       // 0.1% of max torque, signed
    uint8_t error;  // status packet error bits (0 = no error)
};

/**
 * @brief Serial bus driver for Waveshare ST3215 servos (Feetech STS protocol)
 * @note The bus is half-duplex. Set echo if the adapter loops transmitted bytes back to RX.
*/
class ST3215Bus {
public:
    ~ST3215Bus();

    bool open(const std::string& port, int baud_rate, bool echo = false);
    void close();
    bool isOpen() const { return fd_ >= 0; }

    bool ping(uint8_t id);
    bool setTorqueEnabled(const std::vector<uint8_t>& ids, bool enabled);
    bool writeGoalPositions(
        const std::vector<uint8_t>& ids,
        const std::vector<int>& positions,
        uint8_t acceleration,
        uint16_t speed
    );
    void readStates(const std::vector<uint8_t>& ids, std::vector<std::optional<ST3215State>>& states);
    bool readRegisters(uint8_t id, uint8_t address, uint8_t length, std::vector<uint8_t>& data, uint8_t& error);
    bool writeRegisters(uint8_t id, uint8_t address, const std::vector<uint8_t>& data);

    static std::vector<uint8_t> buildPacket(uint8_t id, uint8_t instruction, const std::vector<uint8_t>& params);
    static int decodeSigned(uint16_t value, int sign_bit);

private:
    bool syncWrite(uint8_t address, const std::vector<uint8_t>& ids, const std::vector<std::vector<uint8_t>>& data);
    bool transmit(const std::vector<uint8_t>& packet);
    bool readStatus(uint8_t id, std::vector<uint8_t>& params, uint8_t& error, int timeout_ms);
    bool readBytes(uint8_t* buffer, size_t length, int timeout_ms);

    int fd_ = -1;
    bool echo_ = false;
};

}  // namespace hexapod_hardware_interface

#endif  // HEXAPOD_HARDWARE_INTERFACE__ST3215_BUS_HPP_
