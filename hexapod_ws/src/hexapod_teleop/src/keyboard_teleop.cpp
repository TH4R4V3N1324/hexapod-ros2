#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hexapod_interfaces/srv/get_capabilities.hpp"
#include "hexapod_interfaces/msg/locomotion_option.hpp"
#include "hexapod_interfaces/srv/set_mode.hpp"
#include "hexapod_interfaces/srv/set_gait.hpp"

#include <iostream>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

using geometry_msgs::msg::Twist;
using hexapod_interfaces::srv::GetCapabilities;
using hexapod_interfaces::srv::SetMode;
using hexapod_interfaces::srv::SetGait;

class KeyboardTeleop : public rclcpp::Node {public: KeyboardTeleop() : Node("keyboard_teleop") {
  	RCLCPP_INFO(this->get_logger(), "Keyboard Teleop Node has been started.");
  	cmd_vel_pub = this->create_publisher<Twist>("cmd_vel", 10);
  	get_capabilities_client = this->create_client<GetCapabilities>("get_capabilities");
	set_mode_client = this->create_client<SetMode>("set_mode");
	set_gait_client = this->create_client<SetGait>("set_gait");

	fetchCapabilities();
}
private:
  	rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub;
  	rclcpp::Client<GetCapabilities>::SharedPtr get_capabilities_client;
	rclcpp::Client<SetMode>::SharedPtr set_mode_client;
	rclcpp::Client<SetGait>::SharedPtr set_gait_client;
	std::vector<hexapod_interfaces::msg::LocomotionOption> available_gaits;
	std::vector<hexapod_interfaces::msg::LocomotionOption> available_modes;
	size_t current_gait_index = 0;
	size_t current_mode_index = 0;
  	double linearX = 0;
  	double linearY = 0;
  	double angularZ = 0;
  	termios orig_termios_{};
  	bool termios_ready_ = false;
public:
  	void userInputLoop();
  	void enableRawMode();
  	void disableRawMode();
  	int readKeyWithTimeout(int timeout_ms);
  	void fetchCapabilities();
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<KeyboardTeleop>();
	node->userInputLoop();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

void KeyboardTeleop::fetchCapabilities() {
    if (!get_capabilities_client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_WARN(this->get_logger(), "GetCapabilities service not available");
        return;
    }

    auto request = std::make_shared<hexapod_interfaces::srv::GetCapabilities::Request>();
    auto result_future = get_capabilities_client->async_send_request(request);

    // Wait for the result (synchronously for simplicity)
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result_future.get();

		available_gaits = response->gaits;
		available_modes = response->modes;

		RCLCPP_INFO(this->get_logger(), "Received capabilities: %zu gaits, %zu modes",
					available_gaits.size(), available_modes.size());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call GetCapabilities service");
    }
}

void KeyboardTeleop::userInputLoop() {
	enableRawMode();
	RCLCPP_INFO(this->get_logger(), "\nControls:\n Linear:	[w/s/a/d]\n Angular:	[e/q]\n stop:		[x]\n Gait:		[g]\n Mode:		[m]\n Exit:		[c]\n");
	while (rclcpp::ok()) {
		int key = readKeyWithTimeout(50);
		if (key < 0) {
		continue;
		}

    Twist cmd_vel_msg;
    if (key == 'w') {
        linearX += 0.01;
        RCLCPP_INFO(this->get_logger(), "Increasing linear X: %.2f", linearX);
    } else if (key == 's') {
        linearX -= 0.01;
        RCLCPP_INFO(this->get_logger(), "Decreasing linear X: %.2f", linearX);
    } else if (key == 'a') {
        linearY += 0.01;
        RCLCPP_INFO(this->get_logger(), "Increasing linear Y: %.2f", linearY);
    } else if (key == 'd') {
        linearY -= 0.01;
        RCLCPP_INFO(this->get_logger(), "Decreasing linear Y: %.2f", linearY);
    } else if (key == 'e') {
        angularZ -= 0.1;
        RCLCPP_INFO(this->get_logger(), "Increasing angular Z: %.2f", angularZ);
    } else if (key == 'q') {
        angularZ += 0.1;
        RCLCPP_INFO(this->get_logger(), "Decreasing angular Z: %.2f", angularZ);
    } else if (key == 'x') {
        linearX = 0;
        linearY = 0;
        angularZ = 0;
        RCLCPP_INFO(this->get_logger(), "Stopping robot");
	} else if (key == 'g') {
		if (!available_gaits.empty()) {
			current_gait_index = (current_gait_index + 1) % available_gaits.size();
			const auto& gait = available_gaits[current_gait_index];
			auto request = std::make_shared<SetGait::Request>();
			request->gait = gait.id;
			auto result_future = set_gait_client->async_send_request(request);
			if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
				rclcpp::FutureReturnCode::SUCCESS)
			{
				auto response = result_future.get();
				if (response->success) {
					RCLCPP_INFO(this->get_logger(), "Selected gait: [%d] %s", gait.id, gait.name.c_str());
				} else {
					RCLCPP_WARN(this->get_logger(), "Failed to set gait");
				}
			} else {
				RCLCPP_ERROR(this->get_logger(), "Failed to call SetGait service");
			}
		}
	} else if (key == 'm') {
		if (!available_modes.empty()) {
			current_mode_index = (current_mode_index + 1) % available_modes.size();
			const auto& mode = available_modes[current_mode_index];
			auto request = std::make_shared<SetMode::Request>();
			request->mode = mode.id;
			auto result_future = set_mode_client->async_send_request(request);
			if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
				rclcpp::FutureReturnCode::SUCCESS)
			{
				auto response = result_future.get();
				if (response->success) {
					RCLCPP_INFO(this->get_logger(), "Selected mode: [%d] %s", mode.id, mode.name.c_str());
				} else {
					RCLCPP_WARN(this->get_logger(), "Failed to set mode");
				}
			} else {
				RCLCPP_ERROR(this->get_logger(), "Failed to call SetMode service");
			}
		}
    } else if (key == 'c') {
        RCLCPP_INFO(this->get_logger(), "Exiting...");
        break;
    } else {
        continue;
    }

    cmd_vel_msg.linear.x = linearX;
    cmd_vel_msg.linear.y = linearY;
    cmd_vel_msg.angular.z = angularZ;

    cmd_vel_pub->publish(cmd_vel_msg);
  	}

  	disableRawMode();
}

void KeyboardTeleop::enableRawMode() {
	if (termios_ready_) {
		return;
	}
	if (tcgetattr(STDIN_FILENO, &orig_termios_) == -1) {
		RCLCPP_ERROR(this->get_logger(), "Failed to get terminal attributes");
		return;
	}
	termios raw = orig_termios_;
	raw.c_lflag &= static_cast<unsigned>(~(ECHO | ICANON));
	raw.c_cc[VMIN] = 0;
	raw.c_cc[VTIME] = 0;
	if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == -1) {
		RCLCPP_ERROR(this->get_logger(), "Failed to set terminal raw mode");
		return;
	}
	termios_ready_ = true;
}

void KeyboardTeleop::disableRawMode() {
	if (!termios_ready_) {return;}
	tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
	termios_ready_ = false;
}

int KeyboardTeleop::readKeyWithTimeout(int timeout_ms) {
	fd_set read_fds;
	FD_ZERO(&read_fds);
	FD_SET(STDIN_FILENO, &read_fds);

	timeval timeout;
	timeout.tv_sec = timeout_ms / 1000;
	timeout.tv_usec = (timeout_ms % 1000) * 1000;

	int ready = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &timeout);
	if (ready <= 0) {
		return -1;
	}

	unsigned char c = 0;
	if (read(STDIN_FILENO, &c, 1) == 1) {
		return static_cast<int>(c);
	}
	return -1;
}