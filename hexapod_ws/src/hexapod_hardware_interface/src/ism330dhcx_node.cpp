#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "hexapod_hardware_interface/ism330dhcx.hpp"

using namespace std::chrono_literals;
using hexapod_hardware_interface::ImuSample;
using hexapod_hardware_interface::Ism330dhcx;
using sensor_msgs::msg::Imu;

/**
 * @brief Driver node for the ISM330DHCX IMU, publishes raw accelerometer and gyroscope data
 * @note Publishes sensor_msgs/Imu on imu/data_raw with no orientation (orientation_covariance[0] = -1).
 *       Run imu_filter_madgwick on it to get orientation on imu/data.
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
        std::string error;
        if (!imu.initialise(i2c_bus, address, accel_range_g, gyro_range_dps, rate, error)) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "%s", error.c_str());
            return false;
        }
        RCLCPP_INFO(get_logger(), "%s ready (accel ±%d g, gyro ±%d °/s, %.1f Hz output rate)",
            imu.chipName(), accel_range_g, gyro_range_dps, imu.outputRate());
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

        ImuSample sample;
        if (!imu.read(sample)) {
            RCLCPP_WARN(get_logger(), "ISM330DHCX read failed, reinitialising");
            initialised = false;
            return;
        }

        Imu msg;
        msg.header.stamp = now();
        msg.header.frame_id = frame_id;
        msg.orientation_covariance[0] = -1.0;
        msg.angular_velocity.x = sample.gyro[0];
        msg.angular_velocity.y = sample.gyro[1];
        msg.angular_velocity.z = sample.gyro[2];
        msg.linear_acceleration.x = sample.accel[0];
        msg.linear_acceleration.y = sample.accel[1];
        msg.linear_acceleration.z = sample.accel[2];
        for (int i : {0, 4, 8}) {
            msg.linear_acceleration_covariance[i] = accel_variance;
            msg.angular_velocity_covariance[i] = gyro_variance;
        }
        imu_pub->publish(msg);
    }

    rclcpp::Publisher<Imu>::SharedPtr imu_pub;
    rclcpp::TimerBase::SharedPtr timer;
    Ism330dhcx imu;

    std::string i2c_bus;
    std::string frame_id;
    uint8_t address;
    int accel_range_g;
    int gyro_range_dps;
    double rate;
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
    rclcpp::spin(std::make_shared<Ism330dhcxNode>());
    rclcpp::shutdown();
    return 0;
}
