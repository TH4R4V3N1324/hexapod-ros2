#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <SDL2/SDL.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{
constexpr double kGravity = 9.80665;
}  // namespace

class ControllerImu : public rclcpp::Node
{
public:
  ControllerImu()
  : Node("controller_imu"),
    imu_topic_(declare_parameter("imu_topic", "controller/imu")),
    update_period_(declare_parameter("update_period", 0.01)),
    accelerometer_correction_(declare_parameter("accelerometer_correction", 0.02))
  {
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS());
    if (SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_SENSOR | SDL_INIT_EVENTS) != 0) {
      throw std::runtime_error(std::string("SDL initialization failed: ") + SDL_GetError());
    }
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(update_period_)),
      std::bind(&ControllerImu::update, this));
    RCLCPP_INFO(get_logger(), "Controller IMU node started; publishing %s", imu_topic_.c_str());
  }

  ~ControllerImu() override
  {
    closeController();
    SDL_Quit();
  }

private:
  void closeController()
  {
    if (controller_ != nullptr) {
      SDL_GameControllerClose(controller_);
      controller_ = nullptr;
    }
    initialized_ = false;
  }

  bool openController()
  {
    for (int index = 0; index < SDL_NumJoysticks(); ++index) {
      if (!SDL_IsGameController(index)) {
        continue;
      }
      SDL_GameController *candidate = SDL_GameControllerOpen(index);
      if (candidate == nullptr) {
        continue;
      }
      const bool has_accel = SDL_GameControllerHasSensor(candidate, SDL_SENSOR_ACCEL);
      const bool has_gyro = SDL_GameControllerHasSensor(candidate, SDL_SENSOR_GYRO);
      if (!has_accel || !has_gyro ||
          SDL_GameControllerSetSensorEnabled(candidate, SDL_SENSOR_ACCEL, SDL_TRUE) != 0 ||
          SDL_GameControllerSetSensorEnabled(candidate, SDL_SENSOR_GYRO, SDL_TRUE) != 0)
      {
        RCLCPP_INFO(
          get_logger(), "Controller %s has no usable accelerometer and gyro; skipping",
          SDL_GameControllerName(candidate) != nullptr ? SDL_GameControllerName(candidate) : "unknown");
        SDL_GameControllerClose(candidate);
        continue;
      }
      controller_ = candidate;
      RCLCPP_INFO(
        get_logger(), "Using controller IMU from %s",
        SDL_GameControllerName(controller_) != nullptr ? SDL_GameControllerName(controller_) : "unknown");
      return true;
    }
    return false;
  }

  void update()
  {
    SDL_GameControllerUpdate();
    if (controller_ == nullptr && !openController()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No controller with both accelerometer and gyro found");
      return;
    }

    float acceleration[3] = {};
    float angular_velocity[3] = {};
    if (SDL_GameControllerGetSensorData(
          controller_, SDL_SENSOR_ACCEL, acceleration, 3) != 0 ||
        SDL_GameControllerGetSensorData(
          controller_, SDL_SENSOR_GYRO, angular_velocity, 3) != 0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Could not read controller IMU: %s", SDL_GetError());
      closeController();
      return;
    }

    const rclcpp::Time stamp = now();
    const double dt = last_update_.nanoseconds() == 0
      ? 0.0
      : std::clamp((stamp - last_update_).seconds(), 0.0, 0.1);
    last_update_ = stamp;

    if (!initialized_) {
      roll_ = 0.0;
      pitch_ = 0.0;
      yaw_ = 0.0;
      initialized_ = true;
    } else {
      roll_ += static_cast<double>(angular_velocity[0]) * dt;
      pitch_ += static_cast<double>(angular_velocity[1]) * dt;
      yaw_ += static_cast<double>(angular_velocity[2]) * dt;

      const double acceleration_norm = std::hypot(
        acceleration[0], std::hypot(acceleration[1], acceleration[2]));
      if (acceleration_norm > 0.8 * kGravity && acceleration_norm < 1.2 * kGravity) {
        const double accel_roll = std::atan2(acceleration[1], acceleration[2]);
        const double accel_pitch = std::atan2(
          -acceleration[0], std::hypot(acceleration[1], acceleration[2]));
        roll_ += accelerometer_correction_ * (accel_roll - roll_);
        pitch_ += accelerometer_correction_ * (accel_pitch - pitch_);
      }
    }

    const double cr = std::cos(roll_ / 2.0);
    const double sr = std::sin(roll_ / 2.0);
    const double cp = std::cos(pitch_ / 2.0);
    const double sp = std::sin(pitch_ / 2.0);
    const double cy = std::cos(yaw_ / 2.0);
    const double sy = std::sin(yaw_ / 2.0);

    sensor_msgs::msg::Imu message;
    message.header.stamp = stamp;
    message.header.frame_id = "controller";
    message.orientation.w = cr * cp * cy + sr * sp * sy;
    message.orientation.x = sr * cp * cy - cr * sp * sy;
    message.orientation.y = cr * sp * cy + sr * cp * sy;
    message.orientation.z = cr * cp * sy - sr * sp * cy;
    message.orientation_covariance[0] = 0.0;
    message.angular_velocity.x = angular_velocity[0];
    message.angular_velocity.y = angular_velocity[1];
    message.angular_velocity.z = angular_velocity[2];
    message.linear_acceleration.x = acceleration[0];
    message.linear_acceleration.y = acceleration[1];
    message.linear_acceleration.z = acceleration[2];
    imu_pub_->publish(message);
  }

  std::string imu_topic_;
  double update_period_;
  double accelerometer_correction_;
  SDL_GameController *controller_ = nullptr;
  bool initialized_ = false;
  double roll_ = 0.0;
  double pitch_ = 0.0;
  double yaw_ = 0.0;
  rclcpp::Time last_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ControllerImu>());
  } catch (const std::exception &error) {
    fprintf(stderr, "controller_imu: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
