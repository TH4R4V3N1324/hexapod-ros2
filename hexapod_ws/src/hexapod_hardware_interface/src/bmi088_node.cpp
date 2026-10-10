#include <chrono>
#include <cmath>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "hexapod_hardware_interface/i2c_device.hpp"

using namespace std::chrono_literals;
using hexapod_hardware_interface::I2CDevice;
using sensor_msgs::msg::Imu;

// BMI088 accelerometer registers
static constexpr uint8_t ACC_CHIP_ID = 0x00;
static constexpr uint8_t ACC_DATA = 0x12;
static constexpr uint8_t ACC_CONF = 0x40;
static constexpr uint8_t ACC_RANGE = 0x41;
static constexpr uint8_t ACC_PWR_CONF = 0x7C;
static constexpr uint8_t ACC_PWR_CTRL = 0x7D;
static constexpr uint8_t ACC_SOFTRESET = 0x7E;
static constexpr uint8_t ACC_CHIP_ID_VALUE = 0x1E;

// BMI088 gyroscope registers
static constexpr uint8_t GYRO_CHIP_ID = 0x00;
static constexpr uint8_t GYRO_DATA = 0x02;
static constexpr uint8_t GYRO_RANGE = 0x0F;
static constexpr uint8_t GYRO_BANDWIDTH = 0x10;
static constexpr uint8_t GYRO_SOFTRESET = 0x14;
static constexpr uint8_t GYRO_CHIP_ID_VALUE = 0x0F;

static constexpr uint8_t SOFTRESET_CMD = 0xB6;
static constexpr double GRAVITY = 9.80665;

/**
 * @brief Driver node for the BMI088 IMU, publishes raw accelerometer and gyroscope data
 * @note Publishes sensor_msgs/Imu on imu/data_raw with no orientation (orientation_covariance[0] = -1).
 *       Run imu_filter_madgwick on it to get orientation on imu/data.
*/
class Bmi088Node : public rclcpp::Node {
public:
    /**
     * @brief Declare the node's params and start the read timer
    */
    Bmi088Node() : Node("bmi088") {
        i2c_bus = declare_parameter("i2c_bus", "/dev/i2c-1");
        accel_address = static_cast<uint8_t>(declare_parameter("accel_address", 0x19));
        gyro_address = static_cast<uint8_t>(declare_parameter("gyro_address", 0x69));
        frame_id = declare_parameter("frame_id", "imu_link");
        accel_range_g = static_cast<int>(declare_parameter("accel_range_g", 6));
        gyro_range_dps = static_cast<int>(declare_parameter("gyro_range_dps", 1000));
        // Defaults are datasheet noise density x sqrt(filter bandwidth)
        double accel_stddev = declare_parameter("linear_acceleration_stddev", 0.015);
        double gyro_stddev = declare_parameter("angular_velocity_stddev", 0.0017);
        double rate = declare_parameter("rate", 100.0);

        accel_variance = accel_stddev * accel_stddev;
        gyro_variance = gyro_stddev * gyro_stddev;

        imu_pub = create_publisher<Imu>("imu/data_raw", 10);
        timer = create_wall_timer(std::chrono::duration<double>(1.0 / rate), std::bind(&Bmi088Node::update, this));
    }

private:
    /**
     * @brief Reset and configure both sensors
     * @return true if both sensors were found and configured, false otherwise
    */
    bool initialise() {
        if (!accel.open(i2c_bus, accel_address) || !gyro.open(i2c_bus, gyro_address)) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Failed to open %s", i2c_bus.c_str());
            return false;
        }

        uint8_t accel_range_code;
        switch (accel_range_g) {
            case 3: accel_range_code = 0x00; break;
            case 6: accel_range_code = 0x01; break;
            case 12: accel_range_code = 0x02; break;
            case 24: accel_range_code = 0x03; break;
            default:
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "accel_range_g must be 3, 6, 12 or 24");
                return false;
        }
        uint8_t gyro_range_code;
        switch (gyro_range_dps) {
            case 2000: gyro_range_code = 0x00; break;
            case 1000: gyro_range_code = 0x01; break;
            case 500: gyro_range_code = 0x02; break;
            case 250: gyro_range_code = 0x03; break;
            case 125: gyro_range_code = 0x04; break;
            default:
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "gyro_range_dps must be 125, 250, 500, 1000 or 2000");
                return false;
        }

        // Accelerometer: reset, power on, 200 Hz ODR with normal filter (~80 Hz bandwidth)
        accel.writeRegister8(ACC_SOFTRESET, SOFTRESET_CMD);
        std::this_thread::sleep_for(2ms);
        uint8_t id = 0;
        if (!accel.readRegister8(ACC_CHIP_ID, id) || id != ACC_CHIP_ID_VALUE) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "BMI088 accelerometer not found at 0x%02X (chip ID 0x%02X)", accel_address, id);
            return false;
        }
        bool ok = accel.writeRegister8(ACC_PWR_CTRL, 0x04);
        std::this_thread::sleep_for(50ms);
        ok = ok && accel.writeRegister8(ACC_PWR_CONF, 0x00);
        std::this_thread::sleep_for(5ms);
        ok = ok && accel.writeRegister8(ACC_CONF, 0xA9);
        ok = ok && accel.writeRegister8(ACC_RANGE, accel_range_code);

        // Gyroscope: reset, 400 Hz ODR with 47 Hz filter bandwidth
        gyro.writeRegister8(GYRO_SOFTRESET, SOFTRESET_CMD);
        std::this_thread::sleep_for(30ms);
        if (!gyro.readRegister8(GYRO_CHIP_ID, id) || id != GYRO_CHIP_ID_VALUE) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "BMI088 gyroscope not found at 0x%02X (chip ID 0x%02X)", gyro_address, id);
            return false;
        }
        ok = ok && gyro.writeRegister8(GYRO_RANGE, gyro_range_code);
        ok = ok && gyro.writeRegister8(GYRO_BANDWIDTH, 0x03);

        if (!ok) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Failed to configure BMI088");
            return false;
        }
        RCLCPP_INFO(get_logger(), "BMI088 ready (accel ±%d g, gyro ±%d °/s)", accel_range_g, gyro_range_dps);
        return true;
    }

    /**
     * @brief Read both sensors and publish an Imu message
     * @note Retries initialisation once per second until the sensor is found,
     *       and again after any read failure
    */
    void update() {
        if (!initialised) {
            auto now = std::chrono::steady_clock::now();
            if (now < next_init_attempt) {
                return;
            }
            next_init_attempt = now + 1s;
            initialised = initialise();
            if (!initialised) {
                return;
            }
        }

        uint8_t accel_raw[6];
        uint8_t gyro_raw[6];
        if (!accel.readRegisters(ACC_DATA, accel_raw, 6) || !gyro.readRegisters(GYRO_DATA, gyro_raw, 6)) {
            RCLCPP_WARN(get_logger(), "BMI088 read failed, reinitialising");
            initialised = false;
            return;
        }

        auto toInt16 = [](const uint8_t* bytes) {
            return static_cast<int16_t>(bytes[0] | (bytes[1] << 8));
        };
        const double accel_scale = accel_range_g * GRAVITY / 32768.0;
        const double gyro_scale = gyro_range_dps * M_PI / 180.0 / 32768.0;

        Imu msg;
        msg.header.stamp = now();
        msg.header.frame_id = frame_id;
        msg.orientation_covariance[0] = -1.0;
        msg.linear_acceleration.x = toInt16(&accel_raw[0]) * accel_scale;
        msg.linear_acceleration.y = toInt16(&accel_raw[2]) * accel_scale;
        msg.linear_acceleration.z = toInt16(&accel_raw[4]) * accel_scale;
        msg.angular_velocity.x = toInt16(&gyro_raw[0]) * gyro_scale;
        msg.angular_velocity.y = toInt16(&gyro_raw[2]) * gyro_scale;
        msg.angular_velocity.z = toInt16(&gyro_raw[4]) * gyro_scale;
        for (int i : {0, 4, 8}) {
            msg.linear_acceleration_covariance[i] = accel_variance;
            msg.angular_velocity_covariance[i] = gyro_variance;
        }
        imu_pub->publish(msg);
    }

    rclcpp::Publisher<Imu>::SharedPtr imu_pub;
    rclcpp::TimerBase::SharedPtr timer;
    I2CDevice accel;
    I2CDevice gyro;

    std::string i2c_bus;
    std::string frame_id;
    uint8_t accel_address;
    uint8_t gyro_address;
    int accel_range_g;
    int gyro_range_dps;
    double accel_variance;
    double gyro_variance;

    bool initialised = false;
    std::chrono::steady_clock::time_point next_init_attempt{};
};

/**
 * @brief Entry point, initialises ROS and spins the node
 * @param[in] argc Number of command line arguments
 * @param[in] argv Command line arguments
 * @return Process exit code
*/
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Bmi088Node>());
    rclcpp::shutdown();
    return 0;
}
