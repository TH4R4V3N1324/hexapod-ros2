#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <thread>
#include <vector>

#include "hexapod_hardware_interface/ina226.hpp"
#include "test_common.hpp"

using namespace hardware_tests;
using hexapod_hardware_interface::Ina226;

static constexpr double FULL_VOLTAGE = 12.6;   // 3S LiPo, 4.2 V/cell
static constexpr double EMPTY_VOLTAGE = 9.0;   // 3.0 V/cell
static constexpr uint16_t INA226_DIE_ID = 0x2260;

static const char* USAGE = R"(Test the INA226 power monitors on the two servo daughter boards.

Usage: test_power [--bus /dev/i2c-1] [--left 0x40] [--right 0x41] [--shunt 0.002] [--seconds 10]

Run 'test_servos wiggle <id>' in another terminal while this streams to see the current rise.
)";

/**
 * @brief One power monitor and its readings
*/
struct Monitor {
    const char* location;
    int address;
    Ina226 device;
    bool found = false;
    int failed_reads = 0;
    Stats voltage;
    Stats current;
};

/**
 * @brief Find both power monitors, stream voltage and current, then check the readings make sense
 * @param[in] argc Number of command line arguments
 * @param[in] argv Command line arguments
 * @return 0 if every check passed (warnings allowed), 1 otherwise
*/
int main(int argc, char** argv) {
    Args args(argc, argv);
    if (args.has("help")) {
        printf("%s", USAGE);
        return 0;
    }
    const std::string bus = args.get("bus", "/dev/i2c-1");
    const double shunt = args.getDouble("shunt", 0.002);
    const double seconds = args.getDouble("seconds", 10.0);
    catchCtrlC();

    std::vector<std::unique_ptr<Monitor>> monitors;
    for (auto [location, address] : {std::pair{"left", args.getInt("left", 0x40)}, {"right", args.getInt("right", 0x41)}}) {
        auto monitor = std::make_unique<Monitor>();
        monitor->location = location;
        monitor->address = address;
        monitors.push_back(std::move(monitor));
    }

    Report report;
    printf("Looking for the power monitors on %s\n", bus.c_str());
    for (auto& monitor : monitors) {
        monitor->found = monitor->device.initialise(bus, static_cast<uint8_t>(monitor->address));
        if (!monitor->found) {
            report.fail(format("INA226 (%s) not found at 0x%02X", monitor->location, monitor->address));
            continue;
        }
        uint16_t die_id = 0;
        monitor->device.readDieId(die_id);
        if (die_id == INA226_DIE_ID) {
            report.pass(format("INA226 (%s) found at 0x%02X", monitor->location, monitor->address));
        } else {
            report.warn(format("Chip at 0x%02X (%s) has TI's ID but die ID 0x%04X, not 0x%04X: not an INA226?",
                monitor->address, monitor->location, die_id, INA226_DIE_ID));
        }
    }
    bool any_found = false;
    for (auto& monitor : monitors) {
        any_found = any_found || monitor->found;
    }
    if (!any_found) {
        report.info("Run 'i2cdetect -y 1' on the Pi: 40 and 41 should appear.");
        report.info("The A0 jumper sets the address: open = 0x40 (left), bridged = 0x41 (right).");
        return report.finish();
    }

    printf("\nStreaming for %.0f s (current is positive while the servos draw power). Ctrl+C to stop early.\n\n", seconds);
    const auto start = std::chrono::steady_clock::now();
    auto next = start;
    for (int i = 0; !stop_requested && std::chrono::steady_clock::now() - start < std::chrono::duration<double>(seconds); ++i) {
        next += std::chrono::milliseconds(100);
        std::this_thread::sleep_until(next);

        std::string line;
        for (auto& monitor : monitors) {
            if (!monitor->found) {
                continue;
            }
            double voltage = 0.0;
            double current = 0.0;
            if (!monitor->device.read(shunt, voltage, current)) {
                monitor->failed_reads++;
                continue;
            }
            monitor->voltage.add(voltage);
            monitor->current.add(current);
            line += format("%-5s %6.2f V %+7.3f A %6.2f W    ", monitor->location, voltage, current, voltage * current);
        }
        if (i % 5 == 0) {
            printf("%s\n", line.c_str());
        }
    }

    printf("\nSummary:\n");
    std::vector<double> mean_voltages;
    for (auto& monitor : monitors) {
        if (!monitor->found) {
            continue;
        }
        const char* where = monitor->location;
        if (monitor->voltage.count() == 0) {
            report.fail(format("%s: no readings", where));
            continue;
        }
        if (monitor->failed_reads > 0) {
            report.warn(format("%s: %d reads failed", where, monitor->failed_reads));
        }
        report.info(format("%s: %.2f-%.2f V, current mean %+.3f A, peak %+.3f A",
            where, monitor->voltage.min(), monitor->voltage.max(), monitor->current.mean(), monitor->current.max()));

        double voltage = monitor->voltage.mean();
        mean_voltages.push_back(voltage);
        if (voltage < 1.0) {
            report.fail(format("%s: no voltage, is the battery connected and switched on?", where));
        } else if (voltage < EMPTY_VOLTAGE) {
            report.warn(format("%s: %.2f V is below %.1f V, the battery is flat (or this isn't the 3S LiPo)", where, voltage, EMPTY_VOLTAGE));
        } else if (voltage > FULL_VOLTAGE + 0.2) {
            report.warn(format("%s: %.2f V is above a full 3S LiPo (%.1f V), is this a bench supply?", where, voltage, FULL_VOLTAGE));
        } else {
            report.pass(format("%s: %.2f V, battery about %.0f%%", where, voltage,
                100.0 * (voltage - EMPTY_VOLTAGE) / (FULL_VOLTAGE - EMPTY_VOLTAGE)));
        }

        if (monitor->current.mean() < -0.05) {
            report.warn(format("%s: current reads negative, the shunt is wired backwards (IN+ must be on the battery side)", where));
        } else {
            report.pass(format("%s: current direction looks right", where));
        }
    }
    if (mean_voltages.size() == 2 && std::abs(mean_voltages[0] - mean_voltages[1]) > 0.3) {
        report.warn(format("The sides differ by %.2f V, but share a battery: check the power wiring",
            std::abs(mean_voltages[0] - mean_voltages[1])));
    }
    return report.finish();
}
