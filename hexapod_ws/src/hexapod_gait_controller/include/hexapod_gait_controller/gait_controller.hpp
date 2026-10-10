#ifndef HEXAPOD_GAIT_CONTROLLER_GAIT_CONTROLLER_HPP
#define HEXAPOD_GAIT_CONTROLLER_GAIT_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include <unordered_map>
#include "hexapod_gait_controller/trajectory_generator.hpp"
#include "hexapod_gait_controller/kinematic_solver.hpp"
#include "hexapod_gait_controller/gait_config.hpp"
#include "hexapod_gait_controller/leg_odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
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
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std::placeholders::_2;

struct MotionIntent {
    double forward;  // m/s
    double left;     // m/s
    double yaw;      // rad/s, positive anticlockwise
};

class GaitController : public rclcpp::Node { 
public: 
    GaitController() : Node("gait_controller") {
        cmd_vel_sub = this->create_subscription<Twist>("cmd_vel", 10, std::bind(&GaitController::cmdVelCallback, this, _1));
        joint_state_sub = this->create_subscription<JointState>("joint_states", 10, std::bind(&GaitController::jointStateCallback, this, _1));

        joint_cmd_pub = this->create_publisher<JointState>("joint_commands", 10);
        odom_pub = this->create_publisher<Odometry>("odometry/legs", 10);

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
        max_target_speed = this->declare_parameter("max_target_speed", 0.5);
        // Mode at startup: 0 normal (no strafing, linear.y turns), 1 strafe (standard holonomic /cmd_vel)
        int initial_mode = this->declare_parameter("initial_mode", 0);
        if (initial_mode < 0 || initial_mode >= NUM_MODES) {
            RCLCPP_WARN(this->get_logger(), "initial_mode %d is not a valid mode, using normal", initial_mode);
            initial_mode = MODE_NORMAL;
        }
        gaitConfig.setMode(static_cast<Mode>(initial_mode));

        odom_frame = this->declare_parameter("odom_frame", "odom");
        base_frame = this->declare_parameter("base_frame", "base_footprint");
        odom_publish_period = 1.0 / this->declare_parameter("odom_rate", 50.0);

        // base_footprint -> base_link carries the body height and its roll/pitch from the IMU,
        // published every gait tick
        footprint_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS(),
            std::bind(&GaitController::imuCallback, this, _1));
        nominal_dt = 1.0 / loop_rate_hz;

        // Tilt mode: the feet stay planted while body_pose moves the body, and the IMU holds it
        // level (or at the commanded angle) when whatever it stands on tilts
        body_pose_sub = this->create_subscription<geometry_msgs::msg::Pose>("body_pose", 10,
            std::bind(&GaitController::bodyPoseCallback, this, _1));
        tilt_max_angle = this->declare_parameter("tilt_max_angle", 0.3);
        tilt_max_translation = this->declare_parameter("tilt_max_translation", 0.04);
        tilt_angle_rate = this->declare_parameter("tilt_angle_rate", 1.0);
        tilt_translation_rate = this->declare_parameter("tilt_translation_rate", 0.1);
        tilt_leveling = this->declare_parameter("tilt_leveling", true);
        // Off by default: the IMU yaw drifts without a magnetometer, and holding it uses up the stride
        tilt_lock_yaw = this->declare_parameter("tilt_lock_yaw", false);
        tilt_level_gain = this->declare_parameter("tilt_level_gain", 8.0);
        // Foot forces from the joint efforts, and which feet are touching the ground
        contact_force_on = this->declare_parameter("contact_force_on", 2.0);
        contact_force_off = this->declare_parameter("contact_force_off", 1.0);
        foot_force_filter = this->declare_parameter("foot_force_filter", 0.05);
        // Pushing against something: the feet's net horizontal force is a large share of the weight
        blocked_force_ratio = this->declare_parameter("blocked_force_ratio", 0.3);
        // Spreading the weight between planted feet by raising or lowering each a little
        load_balancing = this->declare_parameter("load_balancing", true);
        load_balance_gain = this->declare_parameter("load_balance_gain", 0.0001);
        load_balance_max = this->declare_parameter("load_balance_max", 0.02);
        for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
            foot_force_pubs[legNum] = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
                "foot_force/leg" + std::to_string(legNum), 10);
        }

        // Leveling while walking (normal and strafe modes): the IMU keeps the body level on slopes
        walk_leveling = this->declare_parameter("walk_leveling", true);
        walk_level_gain = this->declare_parameter("walk_level_gain", 2.0);
        walk_max_angle = this->declare_parameter("walk_max_angle", 0.15);

        // The tilt and leveling parameters can be tuned while running
        param_callback = this->add_on_set_parameters_callback(
            std::bind(&GaitController::parametersCallback, this, _1));

        // ROS timer, so the gait follows simulation time when use_sim_time is set
        gait_timer = this->create_timer(std::chrono::duration<double>(nominal_dt), std::bind(&GaitController::gaitTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "GaitController node started at %.1f Hz, %.2f s per phase", loop_rate_hz, phase_duration);
    }
private:
    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Publisher<JointState>::SharedPtr joint_cmd_pub;
    rclcpp::Publisher<Odometry>::SharedPtr odom_pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> footprint_broadcaster;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    double body_roll = 0.0;   // rad, from the IMU
    double body_pitch = 0.0;  // rad, from the IMU
    double body_yaw = 0.0;    // rad, from the IMU (drifts slowly on the robot, which has no magnetometer)
    rclcpp::Time last_imu_time;
    bool has_imu = false;
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

    // Walking: swing legs fly from where they lifted off to a landing target that follows the command
    std::array<Vector3d, MAX_LEGS + 1> swing_start;   // lift-off position (leg frame)
    std::array<Vector3d, MAX_LEGS + 1> swing_target;  // landing target (leg frame), rate limited
    std::array<bool, MAX_LEGS + 1> leg_swinging{};
    double max_target_speed;  // m/s, how fast a landing target can move when the command changes

    // Tilt mode
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr body_pose_sub;
    BodyPose body_pose_request;  // pose asked for on body_pose, relative to level and the starting heading
    BodyPose tilt_command;       // body_pose_request after rate limiting
    BodyPose tilt_correction;    // leveling correction, built up from the IMU error
    BodyPose tilt_applied;       // pose the feet were last placed for, relative to the ground they stand on
    std::array<Vector3d, MAX_LEGS + 1> tilt_ground_feet;  // feet (leg frame) in the neutral pose
    bool tilt_active = false;    // feet are placed for tilt_applied rather than walking
    double tilt_yaw_reference = 0.0;  // rad, IMU yaw when tilt mode started
    double tilt_max_angle;         // rad, largest roll, pitch or yaw in total
    double tilt_max_translation;   // m, largest body shift along each axis
    double tilt_angle_rate;        // rad/s, how fast the commanded angles can change
    double tilt_translation_rate;  // m/s, how fast the commanded shift can change
    bool tilt_leveling;            // hold the body's angles in space using the IMU
    bool tilt_lock_yaw;            // also hold the heading
    double tilt_level_gain;        // 1/s, how fast the leveling correction responds
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback;

    // Foot forces and contact
    std::array<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr, MAX_LEGS + 1> foot_force_pubs;
    std::array<Vector3d, MAX_LEGS + 1> foot_forces{};   // N, ground on foot (body frame), filtered
    std::array<bool, MAX_LEGS + 1> foot_contact{};      // foot is bearing weight
    bool has_foot_forces = false;
    rclcpp::Time last_foot_force_stamp;
    double contact_force_on;   // N, vertical force above which a foot is in contact
    double contact_force_off;  // N, below which it no longer is
    double foot_force_filter;  // s, time constant of the force low-pass filter
    std::array<double, MAX_LEGS + 1> foot_force_offset{};  // N, added to the vertical force so a lifted
                                                           // foot reads 0 (the leg's own weight), learnt mid-swing

    // Blocked detection, for odometry
    double blocked_force_ratio;   // net horizontal / vertical foot force above which the robot is blocked
    double push_ratio = 0.0;      // net horizontal / total vertical force on the feet in contact, filtered
    bool blocked = false;

    // Load balancing
    bool load_balancing;       // adjust foot heights to spread the weight evenly
    double load_balance_gain;  // m/(N s), how fast a foot moves per newton of extra load; depends on how
                               // stiff the legs and ground are (the sim's are very stiff)
    double load_balance_max;   // m, largest height adjustment of a foot
    std::array<double, MAX_LEGS + 1> foot_height_offset{};  // m, added to each foot's height, + raises it

    // Leveling while walking. Foot positions (current_leg_positions) stay in the level "ground"
    // frame the gait works in; walk_level is applied on the way to IK.
    bool walk_leveling;        // hold the body level with the IMU while walking
    double walk_level_gain;    // 1/s, how fast the correction responds; slow, to follow slopes rather than steps
    double walk_max_angle;     // rad, largest roll or pitch correction
    double walk_level_roll = 0.0;   // rad, current correction
    double walk_level_pitch = 0.0;  // rad
    BodyPose walk_level;       // the correction as a pose, rotating about the ground under the body
    double tilt_saved_roll = 0.0;   // walk leveling when tilt mode started, restored when it ends
    double tilt_saved_pitch = 0.0;
    std::array<Vector3d, MAX_LEGS + 1> tilt_saved_feet;  // ground-frame feet when tilt mode started

    // Leg odometry
    LegOdometry legOdometry;
    std::array<Eigen::Vector2d, MAX_LEGS + 1> previous_feet;  // measured feet (body frame) at the last joint state
    std::array<bool, MAX_LEGS + 1> previous_contact{};       // foot_contact at the last joint state
    rclcpp::Time previous_feet_stamp;
    bool has_previous_feet = false;
    rclcpp::Time last_odom_publish;
    double last_odom_x = 0.0, last_odom_y = 0.0, last_odom_yaw = 0.0;
    bool has_odom_publish = false;
    std::string odom_frame;
    std::string base_frame;
    double odom_publish_period;  // s
    
public:
    void cmdVelCallback(const Twist::SharedPtr msg);
    void jointStateCallback(const JointState::SharedPtr msg);
    void updateLegOdometry(const rclcpp::Time& stamp);
    void updateFootForces(const std::unordered_map<std::string, std::pair<double, double>>& joints, const rclcpp::Time& stamp);
    void publishOdometry(const rclcpp::Time& stamp);
    void publishFootprint();
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void bodyPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);
    bool feetAtStart();
    void updateLoadBalance(double dt);
    bool reachable(const Vector3d& target);
    void updateWalkLeveling(double dt);
    bool tiltFeasible(const BodyPose& pose, std::array<Vector3d, MAX_LEGS + 1>& targets);
    void tilt(double dt, bool exiting);
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
    std::array<Vector3d, MAX_LEGS + 1> walkTargets(double forward, double left, double yaw, double ds, double moveDt);
    void sendFootTargets(const std::array<Vector3d, MAX_LEGS + 1>& targets, double dt);
    void applyPendingGait();
    bool isSwinging(int legNum) const;
    double jointSpeedScale(const std::array<Vector3d, MAX_LEGS + 1>& targets, double dt);
    void PerformLegStep(double dt, bool idle, bool handlePhaseTransition = true);
    void returnToStart(double dt);
    bool HandleIdleReturn(double dt);
    void walk(double dt);
};

#endif // HEXAPOD_GAIT_CONTROLLER_GAIT_CONTROLLER_HPP