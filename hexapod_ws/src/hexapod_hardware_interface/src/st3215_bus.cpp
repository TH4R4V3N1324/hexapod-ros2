#include "hexapod_hardware_interface/st3215_bus.hpp"

#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace hexapod_hardware_interface {

namespace {

constexpr int STATUS_TIMEOUT_MS = 5;
constexpr uint8_t STATE_LENGTH = 6;  // position, speed, load (2 bytes each)

/**
 * @brief Convert a baud rate to its termios speed constant
 * @param[in] baud_rate The baud rate in bits/s
 * @return The termios speed constant, or B0 if the rate is not supported
*/
speed_t toSpeed(int baud_rate) {
    switch (baud_rate) {
        case 1000000: return B1000000;
        case 500000: return B500000;
        case 115200: return B115200;
        case 57600: return B57600;
        case 38400: return B38400;
        default: return B0;
    }
}

/**
 * @brief Calculate the STS checksum of a packet
 * @param[in] packet The packet, starting with the two 0xFF header bytes
 * @return The inverted low byte of the sum of every byte after the header
*/
uint8_t checksum(const std::vector<uint8_t>& packet) {
    uint32_t sum = 0;
    for (size_t i = 2; i < packet.size(); ++i) {
        sum += packet[i];
    }
    return static_cast<uint8_t>(~sum);
}

}  // namespace

/**
 * @brief Close the serial port when the bus is destroyed
*/
ST3215Bus::~ST3215Bus() {
    close();
}

/**
 * @brief Open the serial port and configure it for raw 8N1 communication
 * @param[in] port The serial device, e.g. /dev/ttyAMA0
 * @param[in] baud_rate The bus baud rate (ST3215 default is 1000000)
 * @param[in] echo Whether the adapter echoes transmitted bytes back to RX
 * @return true if the port was opened and configured, false otherwise
*/
bool ST3215Bus::open(const std::string& port, int baud_rate, bool echo) {
    close();
    speed_t speed = toSpeed(baud_rate);
    if (speed == B0) {
        return false;
    }

    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        close();
        return false;
    }
    cfmakeraw(&tty);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(CSTOPB | CRTSCTS);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        close();
        return false;
    }
    tcflush(fd_, TCIOFLUSH);

    echo_ = echo;
    return true;
}

/**
 * @brief Close the serial port if it is open
*/
void ST3215Bus::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

/**
 * @brief Check whether a servo responds on the bus
 * @param[in] id The servo ID
 * @return true if the servo replied, false otherwise
*/
bool ST3215Bus::ping(uint8_t id) {
    if (!transmit(buildPacket(id, ST3215_INST_PING, {}))) {
        return false;
    }
    std::vector<uint8_t> params;
    uint8_t error = 0;
    return readStatus(id, params, error, STATUS_TIMEOUT_MS);
}

/**
 * @brief Enable or disable torque on a set of servos
 * @param[in] ids The servo IDs
 * @param[in] enabled true to enable torque, false to let the servos go limp
 * @return true if the packet was sent, false otherwise
*/
bool ST3215Bus::setTorqueEnabled(const std::vector<uint8_t>& ids, bool enabled) {
    std::vector<std::vector<uint8_t>> data(ids.size(), {static_cast<uint8_t>(enabled ? 1 : 0)});
    return syncWrite(ST3215_REG_TORQUE_ENABLE, ids, data);
}

/**
 * @brief Send goal positions to a set of servos in a single sync write packet
 * @param[in] ids The servo IDs
 * @param[in] positions The goal position of each servo in steps (0-4095)
 * @param[in] acceleration Acceleration limit (0 = no limit)
 * @param[in] speed Speed limit in steps/s (0 = no limit)
 * @return true if the packet was sent, false otherwise
*/
bool ST3215Bus::writeGoalPositions(
    const std::vector<uint8_t>& ids,
    const std::vector<int>& positions,
    uint8_t acceleration,
    uint16_t speed
) {
    std::vector<std::vector<uint8_t>> data;
    data.reserve(ids.size());
    for (int position : positions) {
        uint16_t pos = static_cast<uint16_t>(position);
        // [acceleration][goal position][goal time = 0][goal speed]
        data.push_back({
            acceleration,
            static_cast<uint8_t>(pos & 0xFF), static_cast<uint8_t>(pos >> 8),
            0, 0,
            static_cast<uint8_t>(speed & 0xFF), static_cast<uint8_t>(speed >> 8)
        });
    }
    return syncWrite(ST3215_REG_ACCELERATION, ids, data);
}

/**
 * @brief Read position, speed and load from a set of servos with one sync read
 * @param[in] ids The servo IDs
 * @param[out] states The state of each servo, or nullopt if it did not reply
*/
void ST3215Bus::readStates(const std::vector<uint8_t>& ids, std::vector<std::optional<ST3215State>>& states) {
    states.assign(ids.size(), std::nullopt);

    std::vector<uint8_t> params = {ST3215_REG_PRESENT_POSITION, STATE_LENGTH};
    params.insert(params.end(), ids.begin(), ids.end());
    if (!transmit(buildPacket(ST3215_BROADCAST_ID, ST3215_INST_SYNC_READ, params))) {
        return;
    }

    // Each servo replies with its own status packet, in the order requested
    for (size_t i = 0; i < ids.size(); ++i) {
        std::vector<uint8_t> data;
        uint8_t error = 0;
        if (!readStatus(ids[i], data, error, STATUS_TIMEOUT_MS) || data.size() < STATE_LENGTH) {
            continue;
        }
        states[i] = ST3215State{
            decodeSigned(static_cast<uint16_t>(data[0] | (data[1] << 8)), 15),
            decodeSigned(static_cast<uint16_t>(data[2] | (data[3] << 8)), 15),
            decodeSigned(static_cast<uint16_t>(data[4] | (data[5] << 8)), 10),
            error
        };
    }
}

/**
 * @brief Read a block of registers from one servo
 * @param[in] id The servo ID
 * @param[in] address The first register to read
 * @param[in] length The number of bytes to read
 * @param[out] data The register values
 * @param[out] error The reply's error bits (0 = no error)
 * @return true if the servo replied with the requested bytes, false otherwise
*/
bool ST3215Bus::readRegisters(uint8_t id, uint8_t address, uint8_t length, std::vector<uint8_t>& data, uint8_t& error) {
    if (!transmit(buildPacket(id, ST3215_INST_READ, {address, length}))) {
        return false;
    }
    return readStatus(id, data, error, STATUS_TIMEOUT_MS) && data.size() == length;
}

/**
 * @brief Write a block of registers on one servo, or on every servo with the broadcast ID
 * @param[in] id The servo ID, or ST3215_BROADCAST_ID
 * @param[in] address The first register to write
 * @param[in] data The bytes to write
 * @return true if the write was sent and, unless broadcast, acknowledged, false otherwise
*/
bool ST3215Bus::writeRegisters(uint8_t id, uint8_t address, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> params = {address};
    params.insert(params.end(), data.begin(), data.end());
    if (!transmit(buildPacket(id, ST3215_INST_WRITE, params))) {
        return false;
    }
    if (id == ST3215_BROADCAST_ID) {
        return true;
    }
    std::vector<uint8_t> reply;
    uint8_t error = 0;
    return readStatus(id, reply, error, STATUS_TIMEOUT_MS);
}

/**
 * @brief Build an STS instruction packet
 * @param[in] id The servo ID (0xFE to broadcast)
 * @param[in] instruction The instruction code
 * @param[in] params The instruction parameters
 * @return The packet bytes, including header and checksum
*/
std::vector<uint8_t> ST3215Bus::buildPacket(uint8_t id, uint8_t instruction, const std::vector<uint8_t>& params) {
    std::vector<uint8_t> packet = {0xFF, 0xFF, id, static_cast<uint8_t>(params.size() + 2), instruction};
    packet.insert(packet.end(), params.begin(), params.end());
    packet.push_back(checksum(packet));
    return packet;
}

/**
 * @brief Decode a sign-magnitude register value
 * @param[in] value The raw register value
 * @param[in] sign_bit The bit that holds the sign (15 for position/speed, 10 for load)
 * @return The signed value
*/
int ST3215Bus::decodeSigned(uint16_t value, int sign_bit) {
    const uint16_t mask = static_cast<uint16_t>(1u << sign_bit);
    return (value & mask) ? -static_cast<int>(value & ~mask) : static_cast<int>(value);
}

/**
 * @brief Write the same register block to several servos with one broadcast packet
 * @param[in] address The first register to write
 * @param[in] ids The servo IDs
 * @param[in] data The bytes to write to each servo, all the same length
 * @return true if the packet was sent, false otherwise
 * @note Sync writes are broadcast, so the servos do not reply
*/
bool ST3215Bus::syncWrite(uint8_t address, const std::vector<uint8_t>& ids, const std::vector<std::vector<uint8_t>>& data) {
    if (ids.empty() || ids.size() != data.size()) {
        return false;
    }
    std::vector<uint8_t> params = {address, static_cast<uint8_t>(data[0].size())};
    for (size_t i = 0; i < ids.size(); ++i) {
        params.push_back(ids[i]);
        params.insert(params.end(), data[i].begin(), data[i].end());
    }
    return transmit(buildPacket(ST3215_BROADCAST_ID, ST3215_INST_SYNC_WRITE, params));
}

/**
 * @brief Discard stale input and send a packet
 * @param[in] packet The packet bytes
 * @return true if every byte was written, false otherwise
 * @note If echo is enabled, the echoed bytes are read back and discarded
*/
bool ST3215Bus::transmit(const std::vector<uint8_t>& packet) {
    if (fd_ < 0) {
        return false;
    }
    tcflush(fd_, TCIFLUSH);

    size_t written = 0;
    while (written < packet.size()) {
        ssize_t n = ::write(fd_, packet.data() + written, packet.size() - written);
        if (n < 0) {
            if (errno == EAGAIN) {
                pollfd pfd{fd_, POLLOUT, 0};
                poll(&pfd, 1, STATUS_TIMEOUT_MS);
                continue;
            }
            return false;
        }
        written += static_cast<size_t>(n);
    }
    tcdrain(fd_);

    if (echo_) {
        std::vector<uint8_t> echoed(packet.size());
        return readBytes(echoed.data(), echoed.size(), STATUS_TIMEOUT_MS);
    }
    return true;
}

/**
 * @brief Read a status packet from a servo
 * @param[in] id The servo ID the reply should come from
 * @param[out] params The reply parameters (register data)
 * @param[out] error The reply error bits
 * @param[in] timeout_ms How long to wait for the reply
 * @return true if a valid reply from id was received, false otherwise
*/
bool ST3215Bus::readStatus(uint8_t id, std::vector<uint8_t>& params, uint8_t& error, int timeout_ms) {
    // Find the 0xFF 0xFF header
    uint8_t byte = 0;
    int header_count = 0;
    while (header_count < 2) {
        if (!readBytes(&byte, 1, timeout_ms)) {
            return false;
        }
        header_count = (byte == 0xFF) ? header_count + 1 : 0;
    }

    // Skip any extra 0xFF bytes, the first non-0xFF byte is the ID
    uint8_t reply_id = 0xFF;
    while (reply_id == 0xFF) {
        if (!readBytes(&reply_id, 1, timeout_ms)) {
            return false;
        }
    }

    uint8_t length = 0;
    if (!readBytes(&length, 1, timeout_ms) || length < 2) {
        return false;
    }

    // [error][params...][checksum]
    std::vector<uint8_t> body(length);
    if (!readBytes(body.data(), body.size(), timeout_ms)) {
        return false;
    }

    std::vector<uint8_t> packet = {0xFF, 0xFF, reply_id, length};
    packet.insert(packet.end(), body.begin(), body.end() - 1);
    if (checksum(packet) != body.back() || reply_id != id) {
        return false;
    }

    error = body[0];
    params.assign(body.begin() + 1, body.end() - 1);
    return true;
}

/**
 * @brief Read an exact number of bytes from the serial port
 * @param[out] buffer Where to store the bytes
 * @param[in] length The number of bytes to read
 * @param[in] timeout_ms How long to wait in total
 * @return true if all bytes arrived before the timeout, false otherwise
*/
bool ST3215Bus::readBytes(uint8_t* buffer, size_t length, int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    size_t received = 0;
    while (received < length) {
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count();
        if (remaining <= 0) {
            return false;
        }
        pollfd pfd{fd_, POLLIN, 0};
        if (poll(&pfd, 1, static_cast<int>(remaining)) <= 0) {
            return false;
        }
        ssize_t n = ::read(fd_, buffer + received, length - received);
        if (n < 0 && errno != EAGAIN) {
            return false;
        }
        if (n > 0) {
            received += static_cast<size_t>(n);
        }
    }
    return true;
}

}  // namespace hexapod_hardware_interface
