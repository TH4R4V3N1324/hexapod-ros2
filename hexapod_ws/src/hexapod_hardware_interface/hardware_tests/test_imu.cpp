#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#include "hexapod_hardware_interface/ism330dhcx.hpp"
#include "test_common.hpp"

using namespace hardware_tests;
using hexapod_hardware_interface::ImuSample;
using hexapod_hardware_interface::Ism330dhcx;

static constexpr double GRAVITY = 9.80665;
static constexpr double RAD_TO_DEG = 180.0 / M_PI;

static const char* USAGE = R"(Test the ISM330DHCX IMU: find it, stream readings, then check gravity and gyro bias.

Usage: test_imu [--bus /dev/i2c-1] [--address 0x6A] [--seconds 10]

Keep the robot still and level until the summary. To check the axes while it streams:
  raise the front   -> accel X goes positive
  raise the left    -> accel Y goes positive
  turn it anticlockwise (seen from above) -> gyro Z goes positive
)";

/**
 * @brief Find the IMU, stream readings, then report gravity direction, gravity magnitude and gyro bias
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
    const int address = args.getInt("address", 0x6A);
    const double seconds = args.getDouble("seconds", 10.0);
    const double rate = 100.0;
    catchCtrlC();

    printf("Looking for the IMU at 0x%02X on %s\n", address, bus.c_str());
    Ism330dhcx imu;
    std::string error;
    Report report;
    if (!imu.initialise(bus, static_cast<uint8_t>(address), 8, 1000, rate, error)) {
        report.fail(error);
        report.info("Run 'i2cdetect -y 1' on the Pi: 6a (or 6b with the address jumper bridged) should appear.");
        report.info("If nothing shows, check the wiring: 3V3, GND, SDA = GPIO2 (pin 3), SCL = GPIO3 (pin 5).");
        return report.finish();
    }
    printf("Found %s (accel ±8 g, gyro ±1000 °/s, %.1f Hz output rate)\n", imu.chipName(), imu.outputRate());
    printf("Streaming for %.0f s, keep the robot still and level. Ctrl+C to stop early.\n\n", seconds);

    Stats accel[3], gyro[3], magnitude;
    int failed_reads = 0;
    const auto period = std::chrono::duration<double>(1.0 / rate);
    const auto start = std::chrono::steady_clock::now();
    auto next = start;
    for (int i = 0; !stop_requested && std::chrono::steady_clock::now() - start < std::chrono::duration<double>(seconds); ++i) {
        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(next);

        ImuSample sample;
        if (!imu.read(sample)) {
            failed_reads++;
            continue;
        }
        double norm = std::sqrt(sample.accel[0] * sample.accel[0] + sample.accel[1] * sample.accel[1] +
            sample.accel[2] * sample.accel[2]);
        magnitude.add(norm);
        for (int axis = 0; axis < 3; ++axis) {
            accel[axis].add(sample.accel[axis]);
            gyro[axis].add(sample.gyro[axis] * RAD_TO_DEG);
        }
        if (i % 20 == 0) {
            printf("accel x %+6.2f y %+6.2f z %+6.2f |a| %5.2f m/s²   gyro x %+7.2f y %+7.2f z %+7.2f °/s\n",
                sample.accel[0], sample.accel[1], sample.accel[2], norm,
                sample.gyro[0] * RAD_TO_DEG, sample.gyro[1] * RAD_TO_DEG, sample.gyro[2] * RAD_TO_DEG);
        }
    }

    printf("\nSummary of %d samples:\n", magnitude.count());
    if (magnitude.count() == 0) {
        report.fail("No samples read");
        return report.finish();
    }
    if (failed_reads > 0) {
        report.warn(format("%d reads failed (loose I2C wiring or another program using the bus?)", failed_reads));
    } else {
        report.pass("No failed reads");
    }

    // The accelerometer reads +g on whichever axis points up
    const char* axis_names = "XYZ";
    int up_axis = 0;
    for (int axis = 1; axis < 3; ++axis) {
        if (std::abs(accel[axis].mean()) > std::abs(accel[up_axis].mean())) {
            up_axis = axis;
        }
    }
    char up_sign = accel[up_axis].mean() >= 0.0 ? '+' : '-';
    report.info(format("Mean accel: x %+.2f  y %+.2f  z %+.2f m/s²",
        accel[0].mean(), accel[1].mean(), accel[2].mean()));
    if (up_axis == 2 && up_sign == '+') {
        double tilt = std::acos(std::min(1.0, accel[2].mean() / magnitude.mean())) * RAD_TO_DEG;
        report.pass(format("+Z points up, as imu_link expects (tilted %.1f° from level)", tilt));
    } else {
        report.warn(format("%c%c points up, but imu_link expects +Z up (X forward, Y left)", up_sign, axis_names[up_axis]));
        report.info("If the robot was level, the IMU is mounted rotated: fix imu_joint's rpy in the URDF.");
    }

    double gravity_error = magnitude.mean() - GRAVITY;
    if (std::abs(gravity_error) < 0.5) {
        report.pass(format("Gravity reads %.2f m/s² (expected %.2f)", magnitude.mean(), GRAVITY));
    } else {
        report.warn(format("Gravity reads %.2f m/s², expected %.2f: was the robot moving?", magnitude.mean(), GRAVITY));
    }

    report.info(format("Gyro bias: x %+.2f  y %+.2f  z %+.2f °/s, noise (std dev) x %.2f  y %.2f  z %.2f °/s",
        gyro[0].mean(), gyro[1].mean(), gyro[2].mean(), gyro[0].stddev(), gyro[1].stddev(), gyro[2].stddev()));
    bool gyro_ok = true;
    for (int axis = 0; axis < 3; ++axis) {
        gyro_ok = gyro_ok && std::abs(gyro[axis].mean()) < 3.0 && gyro[axis].stddev() < 0.5;
    }
    if (gyro_ok) {
        report.pass("Gyro bias and noise are small");
    } else {
        report.warn("Gyro bias above 3 °/s or noise above 0.5 °/s: was the robot moving or vibrating?");
    }
    return report.finish();
}
