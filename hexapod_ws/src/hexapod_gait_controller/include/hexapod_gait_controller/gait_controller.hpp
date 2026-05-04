#ifndef HEXAPOD_GAIT_CONTROLLER_GAIT_CONTROLLER_HPP
#define HEXAPOD_GAIT_CONTROLLER_GAIT_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "hexapod_gait_controller/trajectory_generator.hpp"
#include "hexapod_gait_controller/kinematic_solver.hpp"
#include "hexapod_gait_controller/gait_config.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "hexapod_interfaces/srv/set_gait.hpp"
#include "hexapod_interfaces/srv/set_mode.hpp"
#include "hexapod_interfaces/srv/set_height.hpp"
#include "hexapod_interfaces/srv/get_capabilities.hpp"
#include "hexapod_interfaces/msg/locomotion_option.hpp"

using namespace hexapod_gait_controller;

using geometry_msgs::msg::Twist;
using sensor_msgs::msg::JointState;
using hexapod_interfaces::srv::SetGait;
using hexapod_interfaces::srv::SetMode;
using hexapod_interfaces::srv::SetHeight;
using hexapod_interfaces::srv::GetCapabilities;
using Eigen::Vector3d;
using std::placeholders::_1;
using std::placeholders::_2;

struct MotionIntent {
    double forward;
    double lateral;
    double yaw;
};

class GaitController : public rclcpp::Node { 
public: 
    GaitController() : Node("gait_controller") {
        cmd_vel_sub = this->create_subscription<Twist>("cmd_vel", 10, std::bind(&GaitController::cmdVelCallback, this, _1));
        joint_state_sub = this->create_subscription<JointState>("joint_states", 10, std::bind(&GaitController::jointStateCallback, this, _1));

        joint_cmd_pub = this->create_publisher<JointState>("joint_commands", 10);

        set_gait_service = this->create_service<SetGait>("set_gait", std::bind(&GaitController::setGaitCallback, this, _1, _2));
        set_mode_service = this->create_service<SetMode>("set_mode", std::bind(&GaitController::setModeCallback, this, _1, _2));
        set_height_service = this->create_service<SetHeight>("set_height", std::bind(&GaitController::setHeightCallback, this, _1, _2));
        get_capabilities_service = this->create_service<GetCapabilities>("get_capabilities", std::bind(&GaitController::getCapabilitiesCallback, this, _1, _2));
        
        double loop_rate_hz = 50.0;
        gait_timer = this->create_wall_timer(std::chrono::duration<double>(1.0 / loop_rate_hz), std::bind(&GaitController::gaitTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "GaitController node started at %.1f Hz", loop_rate_hz);
    }    
private:
    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Publisher<JointState>::SharedPtr joint_cmd_pub;
    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub;
    rclcpp::TimerBase::SharedPtr gait_timer;
    rclcpp::Service<SetGait>::SharedPtr set_gait_service;
    rclcpp::Service<SetMode>::SharedPtr set_mode_service;
    rclcpp::Service<SetHeight>::SharedPtr set_height_service;
    rclcpp::Service<GetCapabilities>::SharedPtr get_capabilities_service;
    TrajectoryGenerator trajectoryGen;
    GaitConfig gaitConfig;
    KinematicSolver ikSolver;
    
    uint8_t step = 0;
    uint8_t phase = 0;
    uint8_t startup_step = 0;
    
    Twist last_cmd_vel;
    Twist filtered_cmd_vel;
    Twist trajectory_cmd_vel;
    JointState latest_joint_states;
    MotionIntent current_motion_intent{};
    MotionIntent trajectory_motion_intent{};
    
    bool has_joint_states = false;
    bool positions_initialized = false;
    bool idleReturning = false;
    
    static constexpr double CMD_VEL_CHANGE_THRESHOLD = 0.05;

    std::array<std::array<Vector3d, MAX_RESOLUTION + 1>, MAX_LEGS + 1> startup_trajectory;
    std::array<Vector3d, MAX_LEGS + 1> current_leg_positions;
    std::array<int, MAX_LEGS + 1> startup_sizes{};
    
public:
    void cmdVelCallback(const Twist::SharedPtr msg);
    void jointStateCallback(const JointState::SharedPtr msg);
    void setGaitCallback(const std::shared_ptr<SetGait::Request> request, std::shared_ptr<SetGait::Response> response);
    void setModeCallback(const std::shared_ptr<SetMode::Request> request, std::shared_ptr<SetMode::Response> response);
    void setHeightCallback(const std::shared_ptr<SetHeight::Request> request, std::shared_ptr<SetHeight::Response> response);
    void getCapabilitiesCallback(const std::shared_ptr<GetCapabilities::Request> request, std::shared_ptr<GetCapabilities::Response> response);
    void gaitTimerCallback();
    void appendJointCommand(JointState& joint_state_msg, const Vector3d& target, int legNum);
    void sendJointcommands(JointState& msg);
    void home();
    void startup();
    void PerformLegStep(bool idle, bool handlePhaseTransition = true);
    void returnToStart();
    bool HandleIdleReturn();
    void walk();
};

#endif // HEXAPOD_GAIT_CONTROLLER_GAIT_CONTROLLER_HPP