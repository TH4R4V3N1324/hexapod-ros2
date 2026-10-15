#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <optional>
#include <thread>
#include <unistd.h>
#include <vector>

#include "hexapod_hardware_interface/st3215_bus.hpp"
#include "test_common.hpp"

using namespace hardware_tests;
using namespace hexapod_hardware_interface;

static constexpr uint8_t ACCELERATION = 20;  // 8.7 °/s² per unit
static constexpr int ARRIVED_TOLERANCE = 10;  // steps, about 0.9°

static const char* USAGE = R"(Test the ST3215 bus servos.

Usage: test_servos <command> [args] [--port /dev/ttyAMA0] [--baud 1000000] [--echo]

Commands:
  scan [--all]              Ping the 18 leg servos (IDs 11-13, 21-23 ... 61-63) and show their state.
                            --all pings every ID (0-253), to find servos with other IDs.
  monitor [id ...]          Show position, load, voltage and temperature live (default: every leg servo).
                            Doesn't change torque; with torque off, move a joint by hand to see its direction.
  move <id> <degrees>       Move to an angle from the servo's centre (0 = centre, 2048 steps), then hold it.
                            --speed <°/s> (default 30)
  wiggle <id> [degrees]     Move +degrees, back, -degrees, back (default 10), then hold. Shows which joint
                            the ID drives and which way positive is. --speed <°/s> (default 30)
  limp [id ...]             Turn torque off (default: every servo on the bus). The legs will drop!
  set-id <old> <new>        Change a servo's ID (saved in its EEPROM). New servos come as ID 1, so connect
                            them one at a time: two servos with the same ID garble each other's replies.

move and wiggle leave torque on so the joint holds its position; run 'limp' to release it.
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
    uint8_t error;
};

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
 * @brief Read position, speed, load, voltage and temperature in one read
 * @param[in] bus The servo bus
 * @param[in] id The servo ID
 * @return The servo's status, or nullopt if it didn't reply
*/
std::optional<ServoStatus> readStatus(ST3215Bus& bus, uint8_t id) {
    std::vector<uint8_t> data;
    uint8_t error = 0;
    if (!bus.readRegisters(id, ST3215_REG_PRESENT_POSITION, 8, data, error)) {
        return std::nullopt;
    }
    return ServoStatus{
        ST3215Bus::decodeSigned(static_cast<uint16_t>(data[0] | (data[1] << 8)), 15),
        ST3215Bus::decodeSigned(static_cast<uint16_t>(data[2] | (data[3] << 8)), 15),
        ST3215Bus::decodeSigned(static_cast<uint16_t>(data[4] | (data[5] << 8)), 10),
        data[6] * 0.1,
        data[7],
        error
    };
}

void printStatusHeader() {
    printf("  ID  joint         position          speed     load   voltage  temp  status\n");
}

void printStatus(int id, const ServoStatus& status) {
    printf("  %2d  %-12s  %4d (%+7.1f°)  %+5d/s  %+5.1f%%  %5.1f V  %3d°C  %s\n",
        id, jointName(id).c_str(), status.position, stepsToDegrees(status.position), status.speed,
        status.load / 10.0, status.voltage, status.temperature, errorText(status.error).c_str());
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
            report.info("Nothing replied: is the servo power on? Is the port right? Try --echo if the bus adapter echoes.");
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
 * @brief Redraw every servo's state five times a second until Ctrl+C
*/
int monitor(ST3215Bus& bus, std::vector<uint8_t> ids) {
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
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return 0;
}

/**
 * @brief Move one servo to a goal position at a limited speed and wait until it gets there
 * @return The status once it arrived (or timed out), or nullopt if it stopped replying
*/
std::optional<ServoStatus> moveTo(ST3215Bus& bus, uint8_t id, int goal, uint16_t speed) {
    auto status = readStatus(bus, id);
    if (!status) {
        return std::nullopt;
    }
    printf("  -> %+.1f°  ", stepsToDegrees(goal));
    fflush(stdout);
    bus.writeGoalPositions({id}, {goal}, ACCELERATION, speed);

    const double expected_time = std::abs(goal - status->position) / static_cast<double>(speed);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(expected_time + 2.0);
    int peak_load = 0;
    while (!stop_requested && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        status = readStatus(bus, id);
        if (!status) {
            printf("no reply\n");
            return std::nullopt;
        }
        peak_load = std::max(peak_load, std::abs(status->load));
        if (std::abs(status->position - goal) <= ARRIVED_TOLERANCE && status->speed == 0) {
            break;
        }
    }
    int error_steps = status->position - goal;
    printf("reached %+.1f° (%s by %+.1f°), peak load %.1f%%, %s\n",
        stepsToDegrees(status->position),
        std::abs(error_steps) <= ARRIVED_TOLERANCE ? "ok" : "MISSED",
        error_steps * 360.0 / ST3215_STEPS_PER_REV, peak_load / 10.0, errorText(status->error).c_str());
    return status;
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

int move(ST3215Bus& bus, uint8_t id, double degrees, uint16_t speed) {
    if (!holdCurrentPosition(bus, id, speed)) {
        return 1;
    }
    return moveThrough(bus, id, {degreesToSteps(degrees)}, speed);
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
 * @brief Parse servo IDs from positional arguments
*/
std::vector<uint8_t> parseIds(const std::vector<std::string>& args, size_t first) {
    std::vector<uint8_t> ids;
    for (size_t i = first; i < args.size(); ++i) {
        ids.push_back(static_cast<uint8_t>(std::stoi(args[i])));
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
    Args args(argc, argv, {"all", "echo"});
    const auto& pos = args.positional();
    if (args.has("help") || pos.empty()) {
        printf("%s", USAGE);
        return pos.empty() && !args.has("help") ? 1 : 0;
    }
    const std::string port = args.get("port", "/dev/ttyAMA0");
    const int baud = args.getInt("baud", 1000000);
    const double speed_dps = args.getDouble("speed", 30.0);
    const auto speed = static_cast<uint16_t>(std::clamp(speed_dps * ST3215_STEPS_PER_REV / 360.0, 1.0, 3000.0));
    catchCtrlC();

    ST3215Bus bus;
    if (!bus.open(port, baud, args.has("echo"))) {
        printf("Failed to open %s at %d baud\n", port.c_str(), baud);
        printf("Is the robot running with hardware:=st3215? It holds the port; stop it or use hardware:=mock.\n");
        return 1;
    }

    const std::string& command = pos[0];
    try {
        if (command == "scan") {
            return scan(bus, args.has("all"));
        } else if (command == "monitor") {
            return monitor(bus, parseIds(pos, 1));
        } else if (command == "move" && pos.size() == 3) {
            return move(bus, static_cast<uint8_t>(std::stoi(pos[1])), std::stod(pos[2]), speed);
        } else if (command == "wiggle" && (pos.size() == 2 || pos.size() == 3)) {
            return wiggle(bus, static_cast<uint8_t>(std::stoi(pos[1])), pos.size() == 3 ? std::stod(pos[2]) : 10.0, speed);
        } else if (command == "limp") {
            return limp(bus, parseIds(pos, 1));
        } else if (command == "set-id" && pos.size() == 3) {
            return setId(bus, std::stoi(pos[1]), std::stoi(pos[2]));
        }
    } catch (const std::exception&) {
        printf("Invalid number in the arguments\n\n");
    }
    printf("%s", USAGE);
    return 1;
}
