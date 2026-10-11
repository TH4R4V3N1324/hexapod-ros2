#include <chrono>
#include <cmath>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "hexapod_hardware_interface/i2c_device.hpp"

using namespace std::chrono_literals;
using hexapod_hardware_interface::I2CDevice;
using sensor_msgs::msg::Imu;

// ISM330DHCX registers (the older ISM330DLC shares this layout)
static constexpr uint8_t WHO_AM_I = 0x0F;
static constexpr uint8_t CTRL1_XL = 0x10;
static constexpr uint8_t CTRL2_G = 0x11;
static constexpr uint8_t CTRL3_C = 0x12;
static constexpr uint8_t OUTX_L_G = 0x22;  // gyro x, y, z then accel x, y, z, 12 bytes

static constexpr uint8_t ISM330DHCX_ID = 0x6B;
static constexpr uint8_t ISM330DLC_ID = 0x6A;

static constexpr uint8_t CTRL3_SW_RESET = 0x01;
static constexpr uint8_t CTRL3_BDU_IF_INC = 0x44;  // block data update, auto-increment register address
static constexpr double GRAVITY = 9.80665;

/**
 * @brief Driver node for the ISM330DHCX IMU, publishes raw accelerometer and gyroscope data
 * @note Publishes sensor_msgs/Imu on imu/data_raw with no orientation (orientation_covariance[0] = -1).
 *       Run imu_filter_madgwick on it to get orientation on imu/data.
 *       Also accepts the older ISM330DLC, which Adafruit sold on the same style of board.
*/
class Ism330dhcxNode : public rclcpp::Node {
public:
    /**
     * @brief Declare the node's params and start the read timer
    */
    Ism330dhcxNode() : Node("ism330dhcx") {
        i2c_bus = declare_parameter("i2c_bus", "/dev/i2c-1");
        address = static_cast<uint8_t>(declare_parameter("address", 0x6A));
        frame_id = declare_parameter("frame_id", "imu_link");
        accel_range_g = static_cast<int>(declare_parameter("accel_range_g", 8));
        gyro_range_dps = static_cast<int>(declare_parameter("gyro_range_dps", 1000));
        // Defaults are datasheet noise density x sqrt(filter bandwidth) at a 104 Hz output rate
        double accel_stddev = declare_parameter("linear_acceleration_stddev", 0.006);
        double gyro_stddev = declare_parameter("angular_velocity_stddev", 0.0005);
        rate = declare_parameter("rate", 100.0);

        accel_variance = accel_stddev * accel_stddev;
        gyro_variance = gyro_stddev * gyro_stddev;

        imu_pub = create_publisher<Imu>("imu/data_raw", 10);
        timer = create_wall_timer(std::chrono::duration<double>(1.0 / rate), std::bind(&Ism330dhcxNode::update, this));
    }

private:
    /**
     * @brief Reset and configure the sensor
     * @return true if the sensor was found and configured, false otherwise
    */
    bool initialise() {
        if (!imu.open(i2c_bus, address)) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Failed to open %s", i2c_bus.c_str());
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
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "accel_range_g must be 2, 4, 8 or 16");
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
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "gyro_range_dps must be 125, 250, 500, 1000, 2000 or 4000");
                return false;
        }
        // Slowest output data rate at or above the read rate, so every read gets a fresh,
        // anti-alias filtered sample. ODR bits [7:4]: 1 = 12.5 Hz, doubling up to 8 = 1666 Hz.
        uint8_t odr_code = 1;
        double odr_hz = 12.5;
        while (odr_hz < rate && odr_code < 8) {
            odr_code++;
            odr_hz *= 2.0;
        }

        imu.writeRegister8(CTRL3_C, CTRL3_SW_RESET);
        std::this_thread::sleep_for(10ms);
        uint8_t id = 0;
        if (!imu.readRegister8(WHO_AM_I, id) || (id != ISM330DHCX_ID && id != ISM330DLC_ID)) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "ISM330DHCX not found at 0x%02X (WHO_AM_I 0x%02X)", address, id);
            return false;
        }
        const char* chip = id == ISM330DHCX_ID ? "ISM330DHCX" : "ISM330DLC";
        if (id == ISM330DLC_ID && gyro_range_dps == 4000) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "The ISM330DLC has no ±4000 °/s range");
            return false;
        }

        bool ok = imu.writeRegister8(CTRL3_C, CTRL3_BDU_IF_INC);
        ok = ok && imu.writeRegister8(CTRL1_XL, static_cast<uint8_t>(odr_code << 4) | accel_range_code);
        ok = ok && imu.writeRegister8(CTRL2_G, static_cast<uint8_t>(odr_code << 4) | gyro_range_code);
        if (!ok) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Failed to configure %s", chip);
            return false;
        }

        accel_scale = accel_mg_per_lsb / 1000.0 * GRAVITY;
        gyro_scale = gyro_mdps_per_lsb / 1000.0 * M_PI / 180.0;
        RCLCPP_INFO(get_logger(), "%s ready (accel ±%d g, gyro ±%d °/s, %.1f Hz output rate)",
            chip, accel_range_g, gyro_range_dps, odr_hz);
        return true;
    }

    /**
     * @brief Read the sensor and publish an Imu message
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

        uint8_t raw[12];
        if (!imu.readRegisters(OUTX_L_G, raw, 12)) {
            RCLCPP_WARN(get_logger(), "ISM330DHCX read failed, reinitialising");
            initialised = false;
            return;
        }

        auto toInt16 = [](const uint8_t* bytes) {
            return static_cast<int16_t>(bytes[0] | (bytes[1] << 8));
        };

        Imu msg;
        msg.header.stamp = now();
        msg.header.frame_id = frame_id;
        msg.orientation_covariance[0] = -1.0;
        msg.angular_velocity.x = toInt16(&raw[0]) * gyro_scale;
        msg.angular_velocity.y = toInt16(&raw[2]) * gyro_scale;
        msg.angular_velocity.z = toInt16(&raw[4]) * gyro_scale;
        msg.linear_acceleration.x = toInt16(&raw[6]) * accel_scale;
        msg.linear_acceleration.y = toInt16(&raw[8]) * accel_scale;
        msg.linear_acceleration.z = toInt16(&raw[10]) * accel_scale;
        for (int i : {0, 4, 8}) {
            msg.linear_acceleration_covariance[i] = accel_variance;
            msg.angular_velocity_covariance[i] = gyro_variance;
        }
        imu_pub->publish(msg);
    }

    rclcpp::Publisher<Imu>::SharedPtr imu_pub;
    rclcpp::TimerBase::SharedPtr timer;
    I2CDevice imu;

    std::string i2c_bus;
    std::string frame_id;
    uint8_t address;
    int accel_range_g;
    int gyro_range_dps;
    double rate;
    double accel_variance;
    double gyro_variance;
    double accel_scale = 0.0;
    double gyro_scale = 0.0;

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
    rclcpp::spin(std::make_shared<Ism330dhcxNode>());
    rclcpp::shutdown();
    return 0;
}
