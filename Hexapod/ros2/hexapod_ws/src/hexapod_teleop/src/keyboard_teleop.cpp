#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <iostream>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

using geometry_msgs::msg::Twist;

class KeyboardTeleop : public rclcpp::Node {public: KeyboardTeleop() : Node("keyboard_teleop") {
    RCLCPP_INFO(this->get_logger(), "Keyboard Teleop Node has been started.");
    cmd_vel_pub = this->create_publisher<Twist>("cmd_vel", 10);
  }
private:
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub;
    double linearX = 0;
    double linearY = 0;
    double angularZ = 0;
    termios orig_termios_{};
    bool termios_ready_ = false;
public:
  void userInputLoop();
private:
  void enableRawMode();
  void disableRawMode();
  int readKeyWithTimeout(int timeout_ms);
  
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardTeleop>();
  node->userInputLoop();
  rclcpp::shutdown();
  return 0;
}

void KeyboardTeleop::userInputLoop() {
  enableRawMode();
  RCLCPP_INFO(this->get_logger(), "Controls: w/s/a/d for linear, e/q for angular, x to stop, c to exit");
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
        angularZ += 0.1;
        RCLCPP_INFO(this->get_logger(), "Increasing angular Z: %.2f", angularZ);
    } else if (key == 'q') {
        angularZ -= 0.1;
        RCLCPP_INFO(this->get_logger(), "Decreasing angular Z: %.2f", angularZ);
    } else if (key == 'x') {
        linearX = 0;
        linearY = 0;
        angularZ = 0;
        RCLCPP_INFO(this->get_logger(), "Stopping robot");
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
  if (!termios_ready_) {
    return;
  }
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