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
        
        double loop_rate_hz = this->declare_parameter("loop_rate", 100.0);
        phase_duration = this->declare_parameter("phase_duration", 1.0);
        min_phase_duration = this->declare_parameter("min_phase_duration", 0.7);
        max_foot_speed = this->declare_parameter("max_foot_speed", 1.5);
        max_joint_speed = this->declare_parameter("max_joint_speed", 4.0);
        idle_return_delay = this->declare_parameter("idle_return_delay", 2.0);
        startup_duration = this->declare_parameter("startup_duration", 1.0);
        nominal_dt = 1.0 / loop_rate_hz;

        // ROS timer, so the gait follows simulation time when use_sim_time is set
        gait_timer = this->create_timer(std::chrono::duration<double>(nominal_dt), std::bind(&GaitController::gaitTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "GaitController node started at %.1f Hz, %.2f s per phase", loop_rate_hz, phase_duration);
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
    
    uint8_t phase = 0;
    double phase_progress = 0.0;    // fraction of the current phase completed, 0 to 1
    double startup_progress = 0.0;  // fraction of the startup move completed, 0 to 1

    // Timing params, in seconds or SI units so the gait is independent of the loop rate
    double phase_duration;      // s per gait phase at low speeds
    double min_phase_duration;  // s, shortest phase allowed, which sets each gait's top speed
    double current_phase_duration = 1.0;  // s, duration chosen for the phase in progress
    double max_foot_speed;   // m/s, cap on how fast a foot target can move
    double max_joint_speed;  // rad/s, phases slow down so no joint exceeds this
    double idle_return_delay;  // s idle before the legs return to their start positions
    double startup_duration;   // s for the startup move to the start positions
    double nominal_dt;       // s, expected time between gait ticks
    rclcpp::Time last_tick_time;
    bool has_last_tick = false;
    
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

    std::array<Vector3d, MAX_LEGS + 1> current_leg_positions;   // commanded foot positions (leg frame)
    std::array<Vector3d, MAX_LEGS + 1> measured_leg_positions;  // foot positions from joint state feedback (leg frame)
    std::array<Vector3d, MAX_LEGS + 1> startup_start_positions;
    bool startup_planned = false;
    
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
    bool startup(double dt);
    double maxFootSpeed();
    std::array<Vector3d, MAX_LEGS + 1> legTargets(double ds);
    double jointSpeedScale(const std::array<Vector3d, MAX_LEGS + 1>& targets, double dt);
    void PerformLegStep(double dt, bool idle, bool handlePhaseTransition = true);
    void returnToStart(double dt);
    bool HandleIdleReturn(double dt);
    void walk(double dt);
};

#endif // HEXAPOD_GAIT_CONTROLLER_GAIT_CONTROLLER_HPP