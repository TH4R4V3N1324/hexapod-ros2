#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <glob.h>
#include <map>
#include <optional>
#include <poll.h>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <vector>

#include "hexapod_hardware_interface/st3215_bus.hpp"
#include "test_common.hpp"

using namespace hardware_tests;
using namespace hexapod_hardware_interface;

static constexpr uint8_t ACCELERATION = 20;  // 8.7 °/s² per unit
static constexpr int ARRIVED_TOLERANCE = 10;  // steps, about 0.9°
static constexpr double CURRENT_MA_PER_UNIT = 6.5;
static constexpr int MAX_WHEEL_SPEED = 3400;  // steps/s

static const char* USAGE = R"(Test, configure and calibrate the ST3215 bus servos.

Usage: test_servos <command> [args] [--port /dev/ttyAMA0] [--baud 1000000] [--echo] [--debug]

Finding servos:
  ports                     List the serial ports (the GPIO UART, USB adapters).
  scan [--all] [--all-bauds] Ping the 18 leg servos (IDs 11-13, 21-23 ... 61-63) and show their state.
                            --all pings every ID (0-253), to find servos with other IDs. --all-bauds also
                            tries every baud rate this tool can use (1000000, 500000, 115200, 57600, 38400).
  ping <id>                 Check one servo replies.
  info <id>                 Show every register (ID, baud, limits, offset, mode, gains, live state...).
  regs                      List the register names read and write take.
  read <id> <reg> [length]  Read a register by name, or <length> bytes (default 1) from an address.
  write <id> <reg> <value>  Write a register by name (EEPROM registers are unlocked and locked again).
  write <id> <addr> <byte>...  Write raw bytes from an address (EEPROM stays locked unless you unlock it).

Configuring (saved in the servo's EEPROM):
  set-id <old> <new>        Change a servo's ID. New servos come as ID 1, so connect them one at a time:
                            two servos with the same ID garble each other's replies.
  set-baud <id> <rate> --yes  Change a servo's baud rate (1000000 500000 115200 57600 38400). The robot
                            runs at 1000000; the servo stops answering at any other rate until you use --baud.
  mode <id> <0-3>           0 position (what the robot needs), 1 wheel, 2 PWM, 3 step. 1-3 need --yes.
  reset <id> --yes          Factory reset: ID 1, 1000000 baud, no offset or limits.

Calibrating (angles are degrees from the servo's centre, as in move):
  centre <id>               Turn torque off, move the joint by hand to the pose that should read 0 (every
                            joint at 0 is the leg laid out flat), then press Enter: that becomes the centre.
  offset <id> [steps]       Show the position offset centre stored, or set it (±2047 steps) to fine-tune.
  limits <id>               Show the angle limits.
  limits <id> min [degrees] Set the lower limit. Without degrees, torque goes off: move the joint by hand to
  limits <id> max [degrees] the limit and press Enter. max sets the upper limit the same way.
  limits <id> <min> <max>   Set both limits, in degrees.
  limits <id> clear         Remove the limits (the full 0-4095 steps).
Centre a servo before setting its limits: they count from the centre, so centring moves them too.

Moving and testing:
  torque <id> on|off        Hold the joint where it is, or let it go limp.
  limp [id ...]             Turn torque off (default: every servo on the bus). The legs will drop!
  pos [id ...]              Show where each servo is (default: every leg servo).
  move <id> <degrees>       Move to an angle from the servo's centre (0 = centre, 2048 steps), then hold it.
                            --steps takes the target in steps (0-4095) instead.
  wiggle <id> [degrees]     Move +degrees, back, -degrees, back (default 10), then hold. Shows which joint
                            the ID drives and which way positive is.
  sweep <id> [cycles] [<min> <max>]  Move back and forth (default 3 cycles between the servo's limits,
                            or ±30° without any), then show the largest error, move time, load and current.
  wheel <id> <°/s> [seconds] --yes  Spin continuously in wheel mode (default 3 s), then go back to position
                            mode. Only for a servo that isn't fitted to a leg: a joint would hit its stops.
  monitor [id ...]          Show position, load, current, voltage and temperature live (default: every leg
                            servo). --rate <Hz> (default 5). Doesn't change torque; with torque off, move a
                            joint by hand to see its direction.
  selftest <id>             Ping, 50 reads, voltage, temperature, error bits, and a ±10° move and back.

--speed <°/s> sets how fast move, wiggle and sweep go (default 30). Everything that moves leaves torque on so
the joint holds its position; run 'limp' to release it. --debug prints every packet sent and byte received.
)";

/**
 * @brief A servo's feedback registers
*/
struct ServoStatus {
    int position;     // steps
    int speed;        // steps/s
    int load;         // 0.1% of max torque
    double voltage;   // V
    int temperature;  // °C
    double current;   // mA
    bool moving;
    uint8_t error;
};

/**
 * @brief One entry of the ST3215 control table
*/
struct Register {
    const char* name;
    uint8_t address;
    uint8_t size;      // bytes
    bool eeprom;       // kept through a power cycle; needs the lock register cleared to write
    int sign_bit;      // sign-magnitude sign bit, or -1 if unsigned
};

static const std::vector<Register> REGISTERS = {
    {"firmware_major", 0, 1, true, -1},
    {"firmware_minor", 1, 1, true, -1},
    {"servo_major", 3, 1, true, -1},
    {"servo_minor", 4, 1, true, -1},
    {"id", 5, 1, true, -1},
    {"baud", 6, 1, true, -1},
    {"return_delay", 7, 1, true, -1},
    {"status_level", 8, 1, true, -1},
    {"min_angle", 9, 2, true, -1},
    {"max_angle", 11, 2, true, -1},
    {"max_temperature", 13, 1, true, -1},
    {"max_voltage", 14, 1, true, -1},
    {"min_voltage", 15, 1, true, -1},
    {"max_torque", 16, 2, true, -1},
    {"phase", 18, 1, true, -1},
    {"unload_condition", 19, 1, true, -1},
    {"led_alarm", 20, 1, true, -1},
    {"p_gain", 21, 1, true, -1},
    {"d_gain", 22, 1, true, -1},
    {"i_gain", 23, 1, true, -1},
    {"min_start_force", 24, 2, true, -1},
    {"cw_deadband", 26, 1, true, -1},
    {"ccw_deadband", 27, 1, true, -1},
    {"protect_current", 28, 2, true, -1},
    {"angle_resolution", 30, 1, true, -1},
    {"offset", 31, 2, true, 11},
    {"mode", 33, 1, true, -1},
    {"protect_torque", 34, 1, true, -1},
    {"protect_time", 35, 1, true, -1},
    {"overload_torque", 36, 1, true, -1},
    {"torque_enable", 40, 1, false, -1},
    {"acceleration", 41, 1, false, -1},
    {"goal_position", 42, 2, false, 15},
    {"goal_time", 44, 2, false, -1},
    {"goal_speed", 46, 2, false, 15},
    {"torque_limit", 48, 2, false, -1},
    {"lock", 55, 1, false, -1},
    {"present_position", 56, 2, false, 15},
    {"present_speed", 58, 2, false, 15},
    {"present_load", 60, 2, false, 10},
    {"present_voltage", 62, 1, false, -1},
    {"present_temperature", 63, 1, false, -1},
    {"status", 65, 1, false, -1},
    {"moving", 66, 1, false, -1},
    {"present_current", 69, 2, false, 15},
};

// Baud rate register codes, for the rates the bus driver can open
static const std::map<int, uint8_t> BAUD_CODES = {
    {1000000, 0}, {500000, 1}, {115200, 4}, {57600, 6}, {38400, 7},
};
static const std::map<uint8_t, int> BAUD_FROM_CODE = {
    {0, 1000000}, {1, 500000}, {2, 250000}, {3, 128000}, {4, 115200}, {5, 76800}, {6, 57600}, {7, 38400},
};
static const char* MODE_NAMES[] = {"position", "wheel", "PWM", "step"};

/**
 * @brief The 18 leg servo IDs, leg * 10 + joint
 * @return IDs 11-13, 21-23 ... 61-63
*/
std::vector<uint8_t> legServoIds() {
    std::vector<uint8_t> ids;
    for (int leg = 1; leg <= 6; ++leg) {
        for (int joint = 1; joint <= 3; ++joint) {
            ids.push_back(static_cast<uint8_t>(leg * 10 + joint));
        }
    }
    return ids;
}

/**
 * @brief The joint a servo ID drives
 * @param[in] id The servo ID
 * @return e.g. "leg2_femur", or "-" if the ID isn't a leg servo
*/
std::string jointName(int id) {
    static const char* joints[] = {"coxa", "femur", "tibia"};
    int leg = id / 10;
    int joint = id % 10;
    if (leg < 1 || leg > 6 || joint < 1 || joint > 3) {
        return "-";
    }
    return format("leg%d_%s", leg, joints[joint - 1]);
}

/**
 * @brief Describe a status packet's error bits
 * @param[in] error The error bits
 * @return The names of the set bits, or "ok"
*/
std::string errorText(uint8_t error) {
    if (error == 0) {
        return "ok";
    }
    static const char* names[] = {"voltage", "sensor", "overheat", "overcurrent", "angle", "overload", "bit6", "bit7"};
    std::string text;
    for (int bit = 0; bit < 8; ++bit) {
        if (error & (1 << bit)) {
            text += (text.empty() ? "" : ",") + std::string(names[bit]);
        }
    }
    return "ERROR " + text;
}

double stepsToDegrees(int steps) {
    return (steps - ST3215_CENTER_POSITION) * 360.0 / ST3215_STEPS_PER_REV;
}

int degreesToSteps(double degrees) {
    int steps = ST3215_CENTER_POSITION + static_cast<int>(std::lround(degrees * ST3215_STEPS_PER_REV / 360.0));
    return std::clamp(steps, 0, ST3215_STEPS_PER_REV - 1);
}

/**
 * @brief Encode a value as sign-magnitude, the inverse of ST3215Bus::decodeSigned
*/
uint16_t encodeSigned(int value, int sign_bit) {
    return static_cast<uint16_t>(value < 0 ? (-value) | (1 << sign_bit) : value);
}

std::vector<uint8_t> littleEndian(uint16_t value) {
    return {static_cast<uint8_t>(value & 0xFF), static_cast<uint8_t>(value >> 8)};
}

uint8_t parseId(const std::string& text) {
    int id = std::stoi(text, nullptr, 0);
    if (id < 0 || id >= ST3215_BROADCAST_ID) {
        throw std::out_of_range("id");
    }
    return static_cast<uint8_t>(id);
}

/**
 * @brief Read position, speed, load, voltage, temperature, error bits, moving and current in one read
 * @param[in] bus The servo bus
 * @param[in] id The servo ID
 * @return The servo's status, or nullopt if it didn't reply
*/
std::optional<ServoStatus> readStatus(ST3215Bus& bus, uint8_t id) {
    std::vector<uint8_t> data;
    uint8_t error = 0;
    if (!bus.readRegisters(id, ST3215_REG_PRESENT_POSITION, 15, data, error)) {
        return std::nullopt;
    }
    auto word = [&data](size_t i) { return static_cast<uint16_t>(data[i] | (data[i + 1] << 8)); };
    return ServoStatus{
        ST3215Bus::decodeSigned(word(0), 15),
        ST3215Bus::decodeSigned(word(2), 15),
        ST3215Bus::decodeSigned(word(4), 10),
        data[6] * 0.1,
        data[7],
        ST3215Bus::decodeSigned(word(13), 15) * CURRENT_MA_PER_UNIT,
        data[10] != 0,
        error
    };
}

void printStatusHeader() {
    printf("  ID  joint         position          speed     load   current  voltage  temp  status\n");
}

void printStatus(int id, const ServoStatus& status) {
    printf("  %2d  %-12s  %4d (%+7.1f°)  %+5d/s  %+5.1f%%  %5.0f mA  %5.1f V  %3d°C  %s\n",
        id, jointName(id).c_str(), status.position, stepsToDegrees(status.position), status.speed,
        status.load / 10.0, status.current, status.voltage, status.temperature, errorText(status.error).c_str());
}

/**
 * @brief Find a register by name
 * @return The register, or nullptr if there's none by that name
*/
const Register* findRegister(const std::string& name) {
    for (const auto& reg : REGISTERS) {
        if (name == reg.name) {
            return &reg;
        }
    }
    return nullptr;
}

/**
 * @brief Read one register and decode it
 * @return The value, or nullopt if the servo didn't reply
*/
std::optional<int> readRegister(ST3215Bus& bus, uint8_t id, const Register& reg) {
    std::vector<uint8_t> data;
    uint8_t error = 0;
    if (!bus.readRegisters(id, reg.address, reg.size, data, error)) {
        return std::nullopt;
    }
    int value = reg.size == 1 ? data[0] : data[0] | (data[1] << 8);
    return reg.sign_bit >= 0 ? ST3215Bus::decodeSigned(static_cast<uint16_t>(value), reg.sign_bit) : value;
}

/**
 * @brief Write bytes to EEPROM registers: unlock, write, lock again
 * @return true if the servo acknowledged the write
*/
bool writeEeprom(ST3215Bus& bus, uint8_t id, uint8_t address, const std::vector<uint8_t>& data) {
    if (!bus.writeRegisters(id, ST3215_REG_LOCK, {0})) {
        return false;
    }
    const bool written = bus.writeRegisters(id, address, data);
    bus.writeRegisters(id, ST3215_REG_LOCK, {1});
    return written;
}

/**
 * @brief Encode and write one register, unlocking the EEPROM if it's an EEPROM register
 * @return true if the servo acknowledged the write
*/
bool writeRegister(ST3215Bus& bus, uint8_t id, const Register& reg, int value) {
    const uint16_t raw = reg.sign_bit >= 0 ? encodeSigned(value, reg.sign_bit) : static_cast<uint16_t>(value);
    std::vector<uint8_t> data = littleEndian(raw);
    data.resize(reg.size);
    return reg.eeprom ? writeEeprom(bus, id, reg.address, data) : bus.writeRegisters(id, reg.address, data);
}

/**
 * @brief Describe a register's value in its units, e.g. "(12.1 V)", or "" if it has none
*/
std::string describeValue(const Register& reg, int value) {
    const std::string name = reg.name;
    if (name == "baud") {
        auto it = BAUD_FROM_CODE.find(static_cast<uint8_t>(value));
        return it != BAUD_FROM_CODE.end() ? format("(%d baud)", it->second) : "";
    }
    if (name == "mode" && value >= 0 && value <= 3) {
        return format("(%s)", MODE_NAMES[value]);
    }
    if (name == "present_voltage" || name == "max_voltage" || name == "min_voltage") {
        return format("(%.1f V)", value / 10.0);
    }
    if (name == "status") {
        return format("(%s)", errorText(static_cast<uint8_t>(value)).c_str());
    }
    if (name == "present_position" || name == "goal_position" || name == "min_angle" || name == "max_angle") {
        return format("(%+.1f°)", stepsToDegrees(value));
    }
    if (name == "offset") {
        return format("(%+.1f°)", value * 360.0 / ST3215_STEPS_PER_REV);
    }
    if (name == "present_current") {
        return format("(%.0f mA)", value * CURRENT_MA_PER_UNIT);
    }
    return "";
}

/**
 * @brief Turn torque off and show a joint's position live until Enter
 * @param[in] what Where to move the joint to, e.g. "the pose that should read 0"
 * @return The position when Enter was pressed, or nullopt on Ctrl+C or if the servo didn't reply
*/
std::optional<int> positionByHand(ST3215Bus& bus, uint8_t id, const std::string& what) {
    if (!bus.setTorqueEnabled({id}, false) || !readStatus(bus, id)) {
        printf("Servo %d didn't reply. Run 'scan'.\n", id);
        return std::nullopt;
    }
    printf("Torque off on servo %d (%s). Move it by hand to %s, then press Enter (Ctrl+C cancels).\n",
        id, jointName(id).c_str(), what.c_str());
    std::optional<int> position;
    while (!stop_requested) {
        auto status = readStatus(bus, id);
        if (status) {
            position = status->position;
            printf("\r  at %+7.1f° (%4d steps)   ", stepsToDegrees(status->position), status->position);
        } else {
            printf("\r  no reply                 ");
        }
        fflush(stdout);
        pollfd input{STDIN_FILENO, POLLIN, 0};
        if (poll(&input, 1, 100) > 0) {
            char line[256];
            printf("\n");
            if (!fgets(line, sizeof(line), stdin) || !position) {
                break;
            }
            return position;
        }
    }
    printf("\nCancelled\n");
    return std::nullopt;
}

/**
 * @brief Ping servos and print the state of each one that replies
*/
int scan(ST3215Bus& bus, bool all) {
    const auto expected = legServoIds();
    std::vector<uint8_t> ids = expected;
    if (all) {
        ids.clear();
        for (int id = 0; id < ST3215_BROADCAST_ID; ++id) {
            ids.push_back(static_cast<uint8_t>(id));
        }
    }

    printf("Pinging %zu IDs...\n\n", ids.size());
    printStatusHeader();
    std::vector<int> missing, unexpected;
    int faults = 0;
    for (uint8_t id : ids) {
        bool is_expected = std::find(expected.begin(), expected.end(), id) != expected.end();
        if (!bus.ping(id)) {
            if (is_expected) {
                missing.push_back(id);
            }
            continue;
        }
        if (!is_expected) {
            unexpected.push_back(id);
        }
        auto status = readStatus(bus, id);
        if (status) {
            printStatus(id, *status);
            faults += status->error != 0;
        } else {
            printf("  %2d  %-12s  replied to ping but not to a read\n", id, jointName(id).c_str());
        }
    }

    Report report;
    printf("\n");
    int found = static_cast<int>(expected.size() - missing.size());
    if (missing.empty()) {
        report.pass(format("All %zu leg servos replied", expected.size()));
    } else {
        std::string list;
        for (int id : missing) {
            list += format(" %d (%s)", id, jointName(id).c_str());
        }
        report.fail(format("%d/%zu leg servos replied. Missing:%s", found, expected.size(), list.c_str()));
        if (found == 0) {
            report.info("Nothing replied: is the servo power on? Is the port right? Try --echo if the bus "
                "adapter echoes, or --all-bauds in case the servos are at another baud rate.");
        }
    }
    for (int id : unexpected) {
        report.warn(format("Servo with ID %d isn't a leg servo%s: give it a leg ID with 'set-id %d <new>'",
            id, id == 1 ? " (factory default)" : "", id));
    }
    if (faults > 0) {
        report.warn(format("%d servo%s reported an error", faults, faults == 1 ? "" : "s"));
    }
    if (!all) {
        report.info("Run 'scan --all' to find servos with other IDs.");
    }
    return report.finish();
}

/**
 * @brief Ping every ID at every baud rate the bus can open, to find servos set to another rate
*/
int scanAllBauds(ST3215Bus& bus, const std::string& port, bool echo, bool all) {
    std::vector<uint8_t> ids = legServoIds();
    if (all) {
        ids.clear();
        for (int id = 0; id < ST3215_BROADCAST_ID; ++id) {
            ids.push_back(static_cast<uint8_t>(id));
        }
    }
    int found = 0;
    for (auto it = BAUD_CODES.rbegin(); it != BAUD_CODES.rend() && !stop_requested; ++it) {
        if (!bus.open(port, it->first, echo)) {
            printf("  %7d baud: couldn't open %s\n", it->first, port.c_str());
            continue;
        }
        printf("  %7d baud: pinging %zu IDs...", it->first, ids.size());
        fflush(stdout);
        std::string list;
        for (uint8_t id : ids) {
            if (bus.ping(id)) {
                list += format(" %d", id);
                ++found;
            }
        }
        printf(" %s\n", list.empty() ? "none" : ("found" + list).c_str());
    }
    printf("\n%d servo%s found. Use --baud <rate> to talk to a servo at another rate, and 'set-baud <id> 1000000 "
        "--yes' to bring it back to the robot's rate.\n", found, found == 1 ? "" : "s");
    return found > 0 ? 0 : 1;
}

/**
 * @brief List the serial ports a bus adapter could be on
*/
int ports() {
    int count = 0;
    for (const char* pattern : {"/dev/ttyAMA*", "/dev/ttyACM*", "/dev/ttyUSB*", "/dev/serial/by-id/*"}) {
        glob_t matches{};
        if (glob(pattern, 0, nullptr, &matches) == 0) {
            for (size_t i = 0; i < matches.gl_pathc; ++i) {
                printf("  %s\n", matches.gl_pathv[i]);
                ++count;
            }
        }
        globfree(&matches);
    }
    if (count == 0) {
        printf("No serial ports found\n");
    }
    printf("\n/dev/ttyAMA0 is the GPIO UART (pins 8 and 10), ttyACM/ttyUSB are USB adapters.\n");
    return count > 0 ? 0 : 1;
}

int ping(ST3215Bus& bus, uint8_t id) {
    if (!bus.ping(id)) {
        printf("Servo %d: no reply\n", id);
        return 1;
    }
    auto status = readStatus(bus, id);
    printf("Servo %d (%s): replied, %s\n", id, jointName(id).c_str(),
        status ? errorText(status->error).c_str() : "but not to a read");
    return 0;
}

/**
 * @brief Print every register in the control table
*/
int info(ST3215Bus& bus, uint8_t id) {
    if (!bus.ping(id)) {
        printf("Servo %d: no reply\n", id);
        return 1;
    }
    printf("Servo %d (%s)\n\n", id, jointName(id).c_str());
    for (const auto& reg : REGISTERS) {
        auto value = readRegister(bus, id, reg);
        if (value) {
            printf("  [%s] %3d  %-20s %6d  %s\n", reg.eeprom ? "E" : "R", reg.address, reg.name, *value,
                describeValue(reg, *value).c_str());
        } else {
            printf("  [%s] %3d  %-20s  no reply\n", reg.eeprom ? "E" : "R", reg.address, reg.name);
        }
    }
    printf("\nE: EEPROM, kept through a power cycle. R: RAM.\n");
    return 0;
}

int regs() {
    for (const auto& reg : REGISTERS) {
        printf("  %3d  %s  %dB  %s%s\n", reg.address, reg.eeprom ? "EEPROM" : "RAM   ", reg.size, reg.name,
            reg.sign_bit >= 0 ? "  (signed)" : "");
    }
    return 0;
}

/**
 * @brief Read a register by name, or raw bytes from an address
*/
int readCommand(ST3215Bus& bus, uint8_t id, const std::string& what, int length) {
    if (const Register* reg = findRegister(what)) {
        auto value = readRegister(bus, id, *reg);
        if (!value) {
            printf("Servo %d: no reply\n", id);
            return 1;
        }
        printf("%s = %d  %s\n", reg->name, *value, describeValue(*reg, *value).c_str());
        return 0;
    }
    const int address = std::stoi(what, nullptr, 0);
    std::vector<uint8_t> data;
    uint8_t error = 0;
    if (!bus.readRegisters(id, static_cast<uint8_t>(address), static_cast<uint8_t>(length), data, error)) {
        printf("Servo %d: no reply\n", id);
        return 1;
    }
    printf("[%d..%d] =", address, address + length - 1);
    for (uint8_t byte : data) {
        printf(" %d", byte);
    }
    printf("  (hex");
    for (uint8_t byte : data) {
        printf(" %02x", byte);
    }
    printf(")\n");
    return 0;
}

/**
 * @brief Write a register by name (handling the EEPROM lock), or raw bytes from an address
*/
int writeCommand(ST3215Bus& bus, uint8_t id, const std::string& what, const std::vector<std::string>& values) {
    if (const Register* reg = findRegister(what)) {
        if (values.size() != 1) {
            printf("Give one value for %s\n", reg->name);
            return 1;
        }
        if (!writeRegister(bus, id, *reg, std::stoi(values[0], nullptr, 0))) {
            printf("Servo %d didn't acknowledge the write\n", id);
            return 1;
        }
        auto value = readRegister(bus, id, *reg);
        printf("%s = %s  %s\n", reg->name, value ? std::to_string(*value).c_str() : "(no reply to the read back)",
            value ? describeValue(*reg, *value).c_str() : "");
        return 0;
    }
    std::vector<uint8_t> data;
    for (const auto& value : values) {
        data.push_back(static_cast<uint8_t>(std::stoi(value, nullptr, 0)));
    }
    const int address = std::stoi(what, nullptr, 0);
    if (data.empty() || !bus.writeRegisters(id, static_cast<uint8_t>(address), data)) {
        printf("Servo %d didn't acknowledge the write\n", id);
        return 1;
    }
    printf("Wrote %zu byte%s at %d\n", data.size(), data.size() == 1 ? "" : "s", address);
    return 0;
}

/**
 * @brief Redraw every servo's state until Ctrl+C
*/
int monitor(ST3215Bus& bus, std::vector<uint8_t> ids, double rate) {
    if (ids.empty()) {
        for (uint8_t id : legServoIds()) {
            if (bus.ping(id)) {
                ids.push_back(id);
            }
        }
        if (ids.empty()) {
            printf("No leg servos replied. Run 'scan --all'.\n");
            return 1;
        }
    }
    const bool redraw = isatty(STDOUT_FILENO);
    const auto period = std::chrono::duration<double>(1.0 / std::max(rate, 0.1));
    while (!stop_requested) {
        if (redraw) {
            printf("\033[H\033[J");
        }
        printf("Servo monitor, Ctrl+C to stop\n\n");
        printStatusHeader();
        for (uint8_t id : ids) {
            auto status = readStatus(bus, id);
            if (status) {
                printStatus(id, *status);
            } else {
                printf("  %2d  %-12s  no reply\n", id, jointName(id).c_str());
            }
        }
        if (!redraw) {
            printf("\n");
        }
        fflush(stdout);
        std::this_thread::sleep_for(period);
    }
    return 0;
}

/**
 * @brief Show each servo's position once
*/
int positions(ST3215Bus& bus, std::vector<uint8_t> ids) {
    const bool all = ids.empty();
    if (all) {
        ids = legServoIds();
    }
    int replied = 0;
    for (uint8_t id : ids) {
        auto status = readStatus(bus, id);
        if (status) {
            printf("  %2d  %-12s  %4d steps  %+7.1f°\n", id, jointName(id).c_str(), status->position,
                stepsToDegrees(status->position));
            ++replied;
        } else if (!all) {
            printf("  %2d  %-12s  no reply\n", id, jointName(id).c_str());
        }
    }
    if (replied == 0) {
        printf("No servos replied. Run 'scan --all'.\n");
    }
    return replied > 0 ? 0 : 1;
}

/**
 * @brief How a move went
*/
struct MoveResult {
    ServoStatus status;  // where it ended up
    double seconds;
    int peak_load;       // 0.1% of max torque
    double peak_current; // mA
    int peak_temperature;
};

/**
 * @brief Move one servo to a goal position at a limited speed and wait until it gets there
 * @return How the move went once it arrived (or timed out), or nullopt if it stopped replying
*/
std::optional<MoveResult> moveTo(ST3215Bus& bus, uint8_t id, int goal, uint16_t speed) {
    auto status = readStatus(bus, id);
    if (!status) {
        return std::nullopt;
    }
    printf("  -> %+.1f°  ", stepsToDegrees(goal));
    fflush(stdout);
    const auto start = std::chrono::steady_clock::now();
    bus.writeGoalPositions({id}, {goal}, ACCELERATION, speed);

    const double expected_time = std::abs(goal - status->position) / static_cast<double>(speed);
    const auto deadline = start + std::chrono::duration<double>(expected_time + 2.0);
    MoveResult result{*status, 0.0, 0, 0.0, status->temperature};
    while (!stop_requested && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        status = readStatus(bus, id);
        if (!status) {
            printf("no reply\n");
            return std::nullopt;
        }
        result.peak_load = std::max(result.peak_load, std::abs(status->load));
        result.peak_current = std::max(result.peak_current, std::abs(status->current));
        result.peak_temperature = std::max(result.peak_temperature, status->temperature);
        if (std::abs(status->position - goal) <= ARRIVED_TOLERANCE && status->speed == 0) {
            break;
        }
    }
    result.status = *status;
    result.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    int error_steps = status->position - goal;
    printf("reached %+.1f° (%s by %+.1f°) in %.2f s, peak load %.1f%%, %s\n",
        stepsToDegrees(status->position),
        std::abs(error_steps) <= ARRIVED_TOLERANCE ? "ok" : "MISSED",
        error_steps * 360.0 / ST3215_STEPS_PER_REV, result.seconds, result.peak_load / 10.0,
        errorText(status->error).c_str());
    return result;
}

/**
 * @brief Turn torque on with the goal set to the current position, so the joint doesn't jump
*/
std::optional<ServoStatus> holdCurrentPosition(ST3215Bus& bus, uint8_t id, uint16_t speed) {
    auto status = readStatus(bus, id);
    if (!status) {
        printf("Servo %d didn't reply. Run 'scan'.\n", id);
        return std::nullopt;
    }
    bus.writeGoalPositions({id}, {status->position}, ACCELERATION, speed);
    bus.setTorqueEnabled({id}, true);
    printf("Servo %d (%s) at %+.1f°, torque on\n", id, jointName(id).c_str(), stepsToDegrees(status->position));
    return status;
}

/**
 * @brief Move a servo through a list of goal positions, stopping early on Ctrl+C
*/
int moveThrough(ST3215Bus& bus, uint8_t id, const std::vector<int>& goals, uint16_t speed) {
    for (int goal : goals) {
        if (stop_requested) {
            printf("Stopped\n");
            break;
        }
        if (!moveTo(bus, id, goal, speed)) {
            return 1;
        }
    }
    printf("Torque is still on. Run 'test_servos limp %d' to release it.\n", id);
    return 0;
}

int move(ST3215Bus& bus, uint8_t id, int goal, uint16_t speed) {
    if (!holdCurrentPosition(bus, id, speed)) {
        return 1;
    }
    auto low = readRegister(bus, id, *findRegister("min_angle"));
    auto high = readRegister(bus, id, *findRegister("max_angle"));
    if (low && high && (goal < *low || goal > *high)) {
        printf("  %+.1f° is outside its limits (%+.1f° to %+.1f°): it will stop at the limit\n",
            stepsToDegrees(goal), stepsToDegrees(*low), stepsToDegrees(*high));
    }
    return moveThrough(bus, id, {goal}, speed);
}

int wiggle(ST3215Bus& bus, uint8_t id, double degrees, uint16_t speed) {
    auto start = holdCurrentPosition(bus, id, speed);
    if (!start) {
        return 1;
    }
    int offset = static_cast<int>(std::lround(degrees * ST3215_STEPS_PER_REV / 360.0));
    int home = start->position;
    return moveThrough(bus, id, {home + offset, home, home - offset, home}, speed);
}

int torque(ST3215Bus& bus, uint8_t id, bool on, uint16_t speed) {
    if (on) {
        return holdCurrentPosition(bus, id, speed) ? 0 : 1;
    }
    if (!bus.setTorqueEnabled({id}, false) || !readStatus(bus, id)) {
        printf("Servo %d didn't reply. Run 'scan'.\n", id);
        return 1;
    }
    printf("Servo %d (%s): torque off\n", id, jointName(id).c_str());
    return 0;
}

int limp(ST3215Bus& bus, const std::vector<uint8_t>& ids) {
    if (ids.empty()) {
        bus.writeRegisters(ST3215_BROADCAST_ID, ST3215_REG_TORQUE_ENABLE, {0});
        printf("Torque off on every servo\n");
        return 0;
    }
    bus.setTorqueEnabled(ids, false);
    printf("Torque off on %zu servo%s\n", ids.size(), ids.size() == 1 ? "" : "s");
    return 0;
}

/**
 * @brief Move back and forth between two positions and summarise how well it tracked
*/
int sweep(ST3215Bus& bus, uint8_t id, int cycles, std::optional<std::pair<int, int>> range, uint16_t speed) {
    if (!range) {
        auto low = readRegister(bus, id, *findRegister("min_angle"));
        auto high = readRegister(bus, id, *findRegister("max_angle"));
        if (!low || !high) {
            printf("Servo %d didn't reply. Run 'scan'.\n", id);
            return 1;
        }
        const bool limited = *low < *high && !(*low == 0 && *high == ST3215_STEPS_PER_REV - 1);
        range = limited ? std::make_pair(*low, *high) : std::make_pair(degreesToSteps(-30.0), degreesToSteps(30.0));
    }
    if (!holdCurrentPosition(bus, id, speed)) {
        return 1;
    }
    printf("Sweeping %+.1f° to %+.1f°, %d cycle%s (Ctrl+C stops)\n", stepsToDegrees(range->first),
        stepsToDegrees(range->second), cycles, cycles == 1 ? "" : "s");
    Stats errors, seconds;
    int peak_load = 0, peak_temperature = 0;
    double peak_current = 0.0;
    for (int cycle = 0; cycle < cycles && !stop_requested; ++cycle) {
        for (int goal : {range->first, range->second}) {
            if (stop_requested) {
                break;
            }
            auto result = moveTo(bus, id, goal, speed);
            if (!result) {
                return 1;
            }
            errors.add(std::abs(result->status.position - goal) * 360.0 / ST3215_STEPS_PER_REV);
            seconds.add(result->seconds);
            peak_load = std::max(peak_load, result->peak_load);
            peak_current = std::max(peak_current, result->peak_current);
            peak_temperature = std::max(peak_temperature, result->peak_temperature);
        }
    }
    if (errors.count() > 0) {
        printf("\nLargest error %.1f°, average move %.2f s, peak load %.1f%%, peak current %.0f mA, peak %d°C\n",
            errors.max(), seconds.mean(), peak_load / 10.0, peak_current, peak_temperature);
    }
    printf("Torque is still on. Run 'test_servos limp %d' to release it.\n", id);
    return 0;
}

/**
 * @brief Spin in wheel mode for a while, then stop and go back to position mode
*/
int wheel(ST3215Bus& bus, uint8_t id, double degrees_per_second, double seconds) {
    const Register& mode = *findRegister("mode");
    const Register& goal_speed = *findRegister("goal_speed");
    const int speed = std::clamp(static_cast<int>(std::lround(degrees_per_second * ST3215_STEPS_PER_REV / 360.0)),
        -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
    if (!writeRegister(bus, id, mode, 1)) {
        printf("Servo %d didn't reply. Run 'scan'.\n", id);
        return 1;
    }
    bus.setTorqueEnabled({id}, true);
    writeRegister(bus, id, goal_speed, speed);
    printf("Spinning servo %d at %+.0f°/s for %.1f s (Ctrl+C stops)\n", id, speed * 360.0 / ST3215_STEPS_PER_REV, seconds);
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (!stop_requested && std::chrono::steady_clock::now() < end) {
        auto status = readStatus(bus, id);
        if (status) {
            printf("\r  speed %+6.0f°/s  load %+5.1f%%  current %5.0f mA   ", status->speed * 360.0 / ST3215_STEPS_PER_REV,
                status->load / 10.0, status->current);
            fflush(stdout);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    writeRegister(bus, id, goal_speed, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // Back to position mode, holding wherever it stopped
    bus.setTorqueEnabled({id}, false);
    const bool restored = writeRegister(bus, id, mode, 0);
    printf("\nStopped, %s, torque off\n", restored ? "back in position mode" : "BUT STILL IN WHEEL MODE (run 'mode <id> 0')");
    return restored ? 0 : 1;
}

/**
 * @brief Quick health check: replies, reads, voltage, temperature, error bits and a small move
*/
int selftest(ST3215Bus& bus, uint8_t id, uint16_t speed) {
    Report report;
    if (!bus.ping(id)) {
        report.fail(format("Servo %d didn't reply to a ping", id));
        return report.finish();
    }
    report.pass(format("Servo %d (%s) replied", id, jointName(id).c_str()));

    int good = 0;
    for (int i = 0; i < 50; ++i) {
        good += readStatus(bus, id).has_value();
    }
    if (good == 50) {
        report.pass("50/50 reads");
    } else {
        report.warn(format("%d/50 reads: the bus is dropping replies (wiring, --echo, or another program on the port)", good));
    }

    auto status = readStatus(bus, id);
    if (!status) {
        report.fail("No reply to a read");
        return report.finish();
    }
    if (status->voltage >= 9.0 && status->voltage <= 12.6) {
        report.pass(format("Supply %.1f V", status->voltage));
    } else {
        report.warn(format("Supply %.1f V, outside 9-12.6 V", status->voltage));
    }
    if (status->temperature < 60) {
        report.pass(format("Temperature %d°C", status->temperature));
    } else {
        report.warn(format("Temperature %d°C", status->temperature));
    }
    if (status->error == 0) {
        report.pass("No error bits");
    } else {
        report.warn(errorText(status->error));
    }

    if (!holdCurrentPosition(bus, id, speed)) {
        report.fail("Couldn't turn torque on");
        return report.finish();
    }
    const int home = status->position;
    const int offset = degreesToSteps(10.0) - ST3215_CENTER_POSITION;
    double worst = 0.0;
    for (int goal : {home + offset, home - offset, home}) {
        auto result = moveTo(bus, id, goal, speed);
        if (!result) {
            report.fail("Stopped replying while moving");
            return report.finish();
        }
        worst = std::max(worst, std::abs(result->status.position - goal) * 360.0 / ST3215_STEPS_PER_REV);
    }
    if (worst <= ARRIVED_TOLERANCE * 360.0 / ST3215_STEPS_PER_REV) {
        report.pass(format("±10° move, largest error %.1f°", worst));
    } else {
        report.warn(format("±10° move, largest error %.1f°: blocked, or the speed is too high", worst));
    }
    printf("Torque is still on. Run 'test_servos limp %d' to release it.\n", id);
    return report.finish();
}

int setId(ST3215Bus& bus, int old_id, int new_id) {
    if (new_id < 0 || new_id >= ST3215_BROADCAST_ID || new_id == old_id) {
        printf("The new ID must be 0-253 and different from the old one\n");
        return 1;
    }
    if (!bus.ping(static_cast<uint8_t>(old_id))) {
        printf("No servo replied on ID %d. Run 'scan --all'.\n", old_id);
        return 1;
    }
    if (bus.ping(static_cast<uint8_t>(new_id))) {
        printf("A servo already uses ID %d\n", new_id);
        return 1;
    }
    if (jointName(new_id) == "-") {
        printf("Note: %d isn't a leg servo ID (leg * 10 + joint, e.g. 11-13 ... 61-63)\n", new_id);
    }

    const auto old_u8 = static_cast<uint8_t>(old_id);
    const auto new_u8 = static_cast<uint8_t>(new_id);
    if (!bus.writeRegisters(old_u8, ST3215_REG_LOCK, {0})) {
        printf("Servo %d didn't acknowledge the EEPROM unlock\n", old_id);
        return 1;
    }
    // The acknowledgement may come back from either ID, so check with a ping instead
    bus.writeRegisters(old_u8, ST3215_REG_ID, {new_u8});
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    bus.writeRegisters(new_u8, ST3215_REG_LOCK, {1});
    if (!bus.ping(new_u8)) {
        printf("The servo didn't reply on its new ID %d. Run 'scan --all'.\n", new_id);
        return 1;
    }
    printf("Servo %d is now ID %d (%s)\n", old_id, new_id, jointName(new_id).c_str());
    return 0;
}

/**
 * @brief Change a servo's baud rate and check it answers at the new one
*/
int setBaud(ST3215Bus& bus, uint8_t id, int rate, const std::string& port, bool echo) {
    auto code = BAUD_CODES.find(rate);
    if (code == BAUD_CODES.end()) {
        printf("Use one of 1000000 500000 115200 57600 38400\n");
        return 1;
    }
    if (!bus.ping(id)) {
        printf("Servo %d: no reply\n", id);
        return 1;
    }
    // The acknowledgement may come back at either rate, so check with a ping at the new one
    bus.writeRegisters(id, ST3215_REG_LOCK, {0});
    bus.writeRegisters(id, findRegister("baud")->address, {code->second});
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (!bus.open(port, rate, echo) || !bus.ping(id)) {
        printf("Servo %d didn't reply at %d baud. Run 'scan --all --all-bauds'.\n", id, rate);
        return 1;
    }
    bus.writeRegisters(id, ST3215_REG_LOCK, {1});
    printf("Servo %d is now at %d baud%s\n", id, rate,
        rate == 1000000 ? "" : format(": talk to it with --baud %d", rate).c_str());
    return 0;
}

int mode(ST3215Bus& bus, uint8_t id, int value) {
    if (value < 0 || value > 3) {
        printf("Mode must be 0 (position), 1 (wheel), 2 (PWM) or 3 (step)\n");
        return 1;
    }
    if (!writeRegister(bus, id, *findRegister("mode"), value)) {
        printf("Servo %d didn't acknowledge the write\n", id);
        return 1;
    }
    printf("Servo %d is in %s mode%s\n", id, MODE_NAMES[value],
        value == 0 ? "" : ": the robot needs position mode (0)");
    return 0;
}

int reset(ST3215Bus& bus, uint8_t id) {
    if (!bus.factoryReset(id)) {
        printf("Servo %d didn't acknowledge the reset\n", id);
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    printf("Servo %d reset to its factory settings: it's now ID 1 at 1000000 baud. Give it its ID again with "
        "'set-id 1 %d' (on its own on the bus).\n", id, id);
    return 0;
}

/**
 * @brief Print a servo's position offset and angle limits
*/
void printCalibration(ST3215Bus& bus, uint8_t id) {
    auto offset = readRegister(bus, id, *findRegister("offset"));
    auto low = readRegister(bus, id, *findRegister("min_angle"));
    auto high = readRegister(bus, id, *findRegister("max_angle"));
    if (!offset || !low || !high) {
        printf("Servo %d: no reply\n", id);
        return;
    }
    printf("Servo %d (%s): offset %+d steps (%+.1f°), ", id, jointName(id).c_str(), *offset,
        *offset * 360.0 / ST3215_STEPS_PER_REV);
    if (*low == 0 && *high == ST3215_STEPS_PER_REV - 1) {
        printf("no limits\n");
    } else {
        printf("limits %+.1f° to %+.1f° (%d to %d steps)\n", stepsToDegrees(*low), stepsToDegrees(*high), *low, *high);
    }
}

/**
 * @brief Make where the joint is now read as the servo's centre
*/
int centre(ST3215Bus& bus, uint8_t id) {
    auto low = readRegister(bus, id, *findRegister("min_angle"));
    auto high = readRegister(bus, id, *findRegister("max_angle"));
    if (!low || !high) {
        printf("Servo %d didn't reply. Run 'scan'.\n", id);
        return 1;
    }
    if (!positionByHand(bus, id, "the pose that should read 0")) {
        return 1;
    }
    // Unlocked around it so the new offset is kept through a power cycle
    bus.writeRegisters(id, ST3215_REG_LOCK, {0});
    const bool centred = bus.writeRegisters(id, ST3215_REG_TORQUE_ENABLE, {ST3215_TORQUE_SET_CENTRE});
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    bus.writeRegisters(id, ST3215_REG_LOCK, {1});
    bus.setTorqueEnabled({id}, false);
    auto status = readStatus(bus, id);
    if (!centred || !status) {
        printf("Servo %d didn't acknowledge centring\n", id);
        return 1;
    }
    printf("Centred: it now reads %+.1f° (%d steps). Torque is off.\n", stepsToDegrees(status->position),
        status->position);
    printCalibration(bus, id);
    if (!(*low == 0 && *high == ST3215_STEPS_PER_REV - 1)) {
        printf("Its limits were set before centring, so they've moved with it: set them again with "
            "'limits %d min' and 'limits %d max'.\n", id, id);
    }
    return 0;
}

int offset(ST3215Bus& bus, uint8_t id, std::optional<int> steps) {
    if (steps) {
        if (std::abs(*steps) > 2047) {
            printf("The offset must be within ±2047 steps\n");
            return 1;
        }
        if (!writeRegister(bus, id, *findRegister("offset"), *steps)) {
            printf("Servo %d didn't acknowledge the write\n", id);
            return 1;
        }
    }
    printCalibration(bus, id);
    return 0;
}

/**
 * @brief Show or set the angle limits
 * @param[in] args The arguments after the ID: none, "clear", "min [degrees]", "max [degrees]" or "<min> <max>"
*/
int limits(ST3215Bus& bus, uint8_t id, const std::vector<std::string>& args) {
    auto low = readRegister(bus, id, *findRegister("min_angle"));
    auto high = readRegister(bus, id, *findRegister("max_angle"));
    if (!low || !high) {
        printf("Servo %d didn't reply. Run 'scan'.\n", id);
        return 1;
    }
    if (args.empty()) {
        printCalibration(bus, id);
        return 0;
    }
    if (args[0] == "clear" && args.size() == 1) {
        low = 0;
        high = ST3215_STEPS_PER_REV - 1;
    } else if ((args[0] == "min" || args[0] == "max") && args.size() <= 2) {
        const bool is_min = args[0] == "min";
        std::optional<int> steps = args.size() == 2 ? std::optional<int>(degreesToSteps(std::stod(args[1])))
            : positionByHand(bus, id, is_min ? "its lower limit" : "its upper limit");
        if (!steps) {
            return 1;
        }
        (is_min ? low : high) = *steps;
    } else if (args.size() == 2) {
        low = degreesToSteps(std::stod(args[0]));
        high = degreesToSteps(std::stod(args[1]));
    } else {
        printf("Usage: limits <id> [min [degrees] | max [degrees] | <min> <max> | clear]\n");
        return 1;
    }
    if (*low >= *high) {
        printf("The lower limit (%+.1f°) must be below the upper one (%+.1f°)\n", stepsToDegrees(*low),
            stepsToDegrees(*high));
        return 1;
    }
    std::vector<uint8_t> data = littleEndian(static_cast<uint16_t>(*low));
    const auto high_bytes = littleEndian(static_cast<uint16_t>(*high));
    data.insert(data.end(), high_bytes.begin(), high_bytes.end());
    if (!writeEeprom(bus, id, ST3215_REG_MIN_ANGLE_LIMIT, data)) {
        printf("Servo %d didn't acknowledge the write\n", id);
        return 1;
    }
    printCalibration(bus, id);
    return 0;
}

/**
 * @brief Parse servo IDs from positional arguments
*/
std::vector<uint8_t> parseIds(const std::vector<std::string>& args, size_t first) {
    std::vector<uint8_t> ids;
    for (size_t i = first; i < args.size(); ++i) {
        ids.push_back(parseId(args[i]));
    }
    return ids;
}

/**
 * @brief Open the servo bus and run a command
 * @param[in] argc Number of command line arguments
 * @param[in] argv Command line arguments
 * @return 0 if the command succeeded, 1 otherwise
*/
int main(int argc, char** argv) {
    Args args(argc, argv, {"all", "all-bauds", "echo", "debug", "yes", "steps"});
    const auto& pos = args.positional();
    if (args.has("help") || pos.empty()) {
        printf("%s", USAGE);
        return pos.empty() && !args.has("help") ? 1 : 0;
    }
    const std::string& command = pos[0];
    if (command == "ports") {
        return ports();
    }
    if (command == "regs") {
        return regs();
    }

    const std::string port = args.get("port", "/dev/ttyAMA0");
    const int baud = args.getInt("baud", 1000000);
    const bool echo = args.has("echo");
    const double speed_dps = args.getDouble("speed", 30.0);
    const auto speed = static_cast<uint16_t>(std::clamp(speed_dps * ST3215_STEPS_PER_REV / 360.0, 1.0, 3000.0));
    catchCtrlC();

    ST3215Bus bus;
    if (!bus.open(port, baud, echo)) {
        printf("Failed to open %s at %d baud\n", port.c_str(), baud);
        printf("Is the robot running with hardware:=st3215? It holds the port; stop it or use hardware:=mock.\n");
        return 1;
    }
    bus.setDebug(args.has("debug"));

    // Commands that can leave a servo hard to talk to, or move a fitted joint into its stops
    auto confirmed = [&args](const char* why) {
        if (!args.has("yes")) {
            printf("%s Add --yes to go ahead.\n", why);
        }
        return args.has("yes");
    };

    try {
        if (command == "scan" && pos.size() == 1) {
            return args.has("all-bauds") ? scanAllBauds(bus, port, echo, args.has("all")) : scan(bus, args.has("all"));
        } else if (command == "ping" && pos.size() == 2) {
            return ping(bus, parseId(pos[1]));
        } else if (command == "info" && pos.size() == 2) {
            return info(bus, parseId(pos[1]));
        } else if (command == "read" && (pos.size() == 3 || pos.size() == 4)) {
            return readCommand(bus, parseId(pos[1]), pos[2], pos.size() == 4 ? std::stoi(pos[3], nullptr, 0) : 1);
        } else if (command == "write" && pos.size() >= 4) {
            return writeCommand(bus, parseId(pos[1]), pos[2], {pos.begin() + 3, pos.end()});
        } else if (command == "set-id" && pos.size() == 3) {
            return setId(bus, std::stoi(pos[1]), std::stoi(pos[2]));
        } else if (command == "set-baud" && pos.size() == 3) {
            return confirmed("The servo will stop answering at the robot's 1000000 baud unless that's the new rate.")
                ? setBaud(bus, parseId(pos[1]), std::stoi(pos[2]), port, echo) : 1;
        } else if (command == "mode" && pos.size() == 3) {
            const int value = std::stoi(pos[2]);
            return value == 0 || confirmed("Only position mode (0) moves to a goal: in the others a fitted joint can "
                "run into its stops.") ? mode(bus, parseId(pos[1]), value) : 1;
        } else if (command == "reset" && pos.size() == 2) {
            return confirmed("This resets the servo's ID to 1 and clears its offset and limits.")
                ? reset(bus, parseId(pos[1])) : 1;
        } else if ((command == "centre" || command == "center") && pos.size() == 2) {
            return centre(bus, parseId(pos[1]));
        } else if (command == "offset" && (pos.size() == 2 || pos.size() == 3)) {
            return offset(bus, parseId(pos[1]), pos.size() == 3 ? std::optional<int>(std::stoi(pos[2])) : std::nullopt);
        } else if (command == "limits" && pos.size() >= 2 && pos.size() <= 4) {
            return limits(bus, parseId(pos[1]), {pos.begin() + 2, pos.end()});
        } else if (command == "torque" && pos.size() == 3 && (pos[2] == "on" || pos[2] == "off")) {
            return torque(bus, parseId(pos[1]), pos[2] == "on", speed);
        } else if (command == "limp") {
            return limp(bus, parseIds(pos, 1));
        } else if (command == "pos") {
            return positions(bus, parseIds(pos, 1));
        } else if (command == "move" && pos.size() == 3) {
            const int goal = args.has("steps") ? std::clamp(std::stoi(pos[2]), 0, ST3215_STEPS_PER_REV - 1)
                : degreesToSteps(std::stod(pos[2]));
            return move(bus, parseId(pos[1]), goal, speed);
        } else if (command == "wiggle" && (pos.size() == 2 || pos.size() == 3)) {
            return wiggle(bus, parseId(pos[1]), pos.size() == 3 ? std::stod(pos[2]) : 10.0, speed);
        } else if (command == "sweep" && (pos.size() == 2 || pos.size() == 3 || pos.size() == 5)) {
            std::optional<std::pair<int, int>> range;
            if (pos.size() == 5) {
                range = std::make_pair(degreesToSteps(std::stod(pos[3])), degreesToSteps(std::stod(pos[4])));
            }
            return sweep(bus, parseId(pos[1]), pos.size() >= 3 ? std::stoi(pos[2]) : 3, range, speed);
        } else if (command == "wheel" && (pos.size() == 3 || pos.size() == 4)) {
            return confirmed("Wheel mode spins continuously: only for a servo that isn't fitted to a leg.")
                ? wheel(bus, parseId(pos[1]), std::stod(pos[2]), pos.size() == 4 ? std::stod(pos[3]) : 3.0) : 1;
        } else if (command == "monitor") {
            return monitor(bus, parseIds(pos, 1), args.getDouble("rate", 5.0));
        } else if (command == "selftest" && pos.size() == 2) {
            return selftest(bus, parseId(pos[1]), speed);
        }
    } catch (const std::exception&) {
        printf("Invalid number in the arguments\n\n");
    }
    printf("%s", USAGE);
    return 1;
}
