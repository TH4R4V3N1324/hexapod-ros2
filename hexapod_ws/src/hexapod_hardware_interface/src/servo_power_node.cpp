#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

/**
 * @brief Drives the leg hubs' SERVO_EN from the gait controller's servo_power topic
 * @note SERVO_EN switches the servos' power (as long as the E-stop loop is closed too). Every line
 *       in gpio_lines is set together in one request, so with one line per leg hub the two sides
 *       still switch as one. The lines start low and go low again when the node stops, so nothing
 *       powers the servos without a gait controller asking for it; the leg hubs' pull-downs hold
 *       them off while the Pi boots and if this node dies. Uses the GPIO character device directly
 *       (/dev/gpiochipN, the kernel's v2 uAPI), so it needs no GPIO library.
*/
class ServoPowerNode : public rclcpp::Node {
public:
    ServoPowerNode() : Node("servo_power") {
        // Pi 5 with kernel 6.6.45 or newer: the header pins are on gpiochip0 (gpiochip4 before that)
        const std::string chip = declare_parameter("gpio_chip", std::string("/dev/gpiochip0"));
        const auto lines = declare_parameter("gpio_lines", std::vector<int64_t>{17});
        if (lines.empty() || lines.size() > GPIO_V2_LINES_MAX) {
            throw std::runtime_error("gpio_lines must list 1 to 64 GPIO numbers");
        }

        const int chip_fd = ::open(chip.c_str(), O_RDWR | O_CLOEXEC);
        if (chip_fd < 0) {
            throw std::runtime_error("Can't open " + chip + ": " + std::strerror(errno));
        }
        gpio_v2_line_request request{};
        for (size_t i = 0; i < lines.size(); ++i) {
            request.offsets[i] = static_cast<uint32_t>(lines[i]);
        }
        request.num_lines = static_cast<uint32_t>(lines.size());
        std::strncpy(request.consumer, "servo_power", sizeof(request.consumer) - 1);
        all_lines = lines.size() == 64 ? ~uint64_t{0} : (uint64_t{1} << lines.size()) - 1;
        // Outputs, starting low (servos off)
        request.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
        request.config.num_attrs = 1;
        request.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
        request.config.attrs[0].attr.values = 0;
        request.config.attrs[0].mask = all_lines;
        const int result = ::ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &request);
        const int error = errno;
        ::close(chip_fd);
        if (result < 0) {
            throw std::runtime_error("Can't claim the SERVO_EN lines on " + chip + ": " + std::strerror(error));
        }
        line_fd = request.fd;

        // Latched, like the gait controller publishes it, so a restart picks up the current state
        servo_power_sub = create_subscription<std_msgs::msg::Bool>("servo_power", rclcpp::QoS(1).transient_local(),
            [this](const std_msgs::msg::Bool::SharedPtr msg) { set(msg->data); });
        RCLCPP_INFO(get_logger(), "SERVO_EN on %zu line(s) of %s, servos off until the gait controller asks",
            lines.size(), chip.c_str());
    }

    ~ServoPowerNode() override {
        if (line_fd >= 0) {
            set(false);
            ::close(line_fd);
        }
    }

private:
    /**
     * @brief Set every SERVO_EN line high (servos powered) or low (off)
    */
    void set(bool on) {
        gpio_v2_line_values values{};
        values.mask = all_lines;
        values.bits = on ? all_lines : 0;
        if (::ioctl(line_fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &values) < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to set SERVO_EN %s: %s", on ? "high" : "low", std::strerror(errno));
            return;
        }
        if (on != powered) {
            RCLCPP_INFO(get_logger(), "Servo power %s", on ? "on" : "off");
        }
        powered = on;
    }

    int line_fd = -1;
    uint64_t all_lines = 0;
    bool powered = false;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servo_power_sub;
};

/**
 * @brief Entry point, initialises ROS and spins the node
*/
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int status = 0;
    try {
        rclcpp::spin(std::make_shared<ServoPowerNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("servo_power"), "%s", e.what());
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}
