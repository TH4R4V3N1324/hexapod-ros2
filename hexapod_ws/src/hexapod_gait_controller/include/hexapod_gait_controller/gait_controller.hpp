#ifndef HEXAPOD_GAIT_CONTROLLER_GAIT_CONTROLLER_HPP
#define HEXAPOD_GAIT_CONTROLLER_GAIT_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include <array>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>
#include "gait_core/gait_engine.hpp"
#include "hexapod_gait_controller/height_map.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "hexapod_interfaces/srv/set_gait.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "hexapod_interfaces/srv/set_mode.hpp"
#include "hexapod_interfaces/srv/set_height.hpp"
#include "hexapod_interfaces/srv/get_capabilities.hpp"
#include "hexapod_interfaces/msg/locomotion_option.hpp"

using namespace gait_core;
using namespace hexapod_gait_controller;

// The node passes Eigen doubles straight into gait_core, so it must be built with double Scalars.
// colcon.meta in the workspace root sets this.
static_assert(std::is_same_v<gait_core::Scalar, double>,
              "gait_core must be built with -DGAIT_CORE_USE_DOUBLE=ON (see hexapod_ws/colcon.meta)");

using geometry_msgs::msg::Twist;
using sensor_msgs::msg::JointState;
using hexapod_interfaces::srv::SetGait;
using hexapod_interfaces::srv::SetMode;
using hexapod_interfaces::srv::SetHeight;
using hexapod_interfaces::srv::GetCapabilities;
using Eigen::Vector2d;
using Eigen::Vector3d;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std::placeholders::_2;

/**
 * @brief ROS wrapper around gait_core::GaitEngine
 * @note The gait itself (startup, walking, sitting, tilt mode, leveling, load balancing, terrain
 *       adaptation, leg odometry) is in hexapod-gait-core. This node turns topics, services and
 *       parameters into engine inputs, runs it on a timer, and publishes what it produces. It also
 *       keeps the depth camera height map, which needs TF, and plans footholds from it for the engine.
*/
class GaitController : public rclcpp::Node, private gait_core::FootholdPlanner {
public:
    GaitController();

    void cmdVelCallback(const Twist::SharedPtr msg);
    void jointStateCallback(const JointState::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void bodyPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);
    void setGaitCallback(const std::shared_ptr<SetGait::Request> request, std::shared_ptr<SetGait::Response> response);
    void setModeCallback(const std::shared_ptr<SetMode::Request> request, std::shared_ptr<SetMode::Response> response);
    void setHeightCallback(const std::shared_ptr<SetHeight::Request> request, std::shared_ptr<SetHeight::Response> response);
    void sitCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                     std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void powerDownCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void toggleCallback(const std::string& feature, const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void getCapabilitiesCallback(const std::shared_ptr<GetCapabilities::Request> request,
                                 std::shared_ptr<GetCapabilities::Response> response);
    void gaitTimerCallback();
    void depthCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publishTerrainMap();

    // Called with every joint command just before it is published. Unset in normal use; the golden
    // trace test sets it to record the commands without spinning a subscriber.
    std::function<void(const JointState&)> joint_command_observer;

private:
    std::unique_ptr<GaitEngine> engine;

    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr body_pose_sub;
    rclcpp::Publisher<JointState>::SharedPtr joint_cmd_pub;
    rclcpp::Publisher<Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr seated_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr servo_power_pub;
    bool servo_power_published = false;
    bool last_servo_power = false;
    std::array<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr, NUM_LEGS> foot_force_pubs;
    std::shared_ptr<tf2_ros::TransformBroadcaster> footprint_broadcaster;
    rclcpp::TimerBase::SharedPtr gait_timer;
    rclcpp::Service<SetGait>::SharedPtr set_gait_service;
    rclcpp::Service<SetMode>::SharedPtr set_mode_service;
    rclcpp::Service<SetHeight>::SharedPtr set_height_service;
    rclcpp::Service<GetCapabilities>::SharedPtr get_capabilities_service;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr sit_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr power_down_service;
    std::vector<rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> toggle_services;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback;

    // Gait timing
    double nominal_dt;       // s, expected time between gait ticks
    rclcpp::Time last_tick_time;
    bool has_last_tick = false;

    // cmd_vel, smoothed
    Twist filtered_cmd_vel;

    // foot_force/legN are for display only, so they're published at foot_force_rate
    double foot_force_period;  // s
    rclcpp::Time last_foot_force_publish{0, 0, RCL_ROS_TIME};

    // Leg odometry publishing
    rclcpp::Time last_odom_publish;
    double last_odom_x = 0.0, last_odom_y = 0.0, last_odom_yaw = 0.0;
    bool has_odom_publish = false;
    std::string odom_frame;
    std::string base_frame;
    double odom_publish_period;  // s

    // Height map from the depth camera, for choosing footholds and step heights
    std::string map_frame;      // fixed frame the map is kept in
    int depth_stride;           // use every nth row and column of the depth image
    double map_update_period;   // s, shortest time between depth frames added to the map
    rclcpp::Time last_map_update;
    bool has_map_update = false;
    double map_radius;          // m, how much of the map around the robot is kept
    double foothold_search;     // m, how far a landing spot may move to get off an edge
    double foothold_edge;       // m, flat ground needed around a foothold
    double foothold_flatness;   // m, height difference within foothold_edge that still counts as flat
    double step_clearance;      // m, how far a swinging foot clears the highest ground on its way
    double map_height_deadband; // m, the map only changes a landing height by more than this
    HeightMap heightMap{0.02};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_map_pub;
    rclcpp::TimerBase::SharedPtr terrain_map_timer;
    Eigen::Isometry2d foothold_body_to_map;  // set by ready() for plan()

    void publishSeated(bool seated);
    void publishServoPower(bool powered);
    void publishJointCommands(const GaitOutput& output);
    void publishOdometry(const rclcpp::Time& stamp);
    void publishFootprint();
    bool bodyToMap(Eigen::Isometry2d& transform);

    // gait_core::FootholdPlanner
    bool ready() override;
    void plan(int legNum, Vec3& target, const Vec3& swingStart, Scalar& swingLift) override;
};

#endif  // HEXAPOD_GAIT_CONTROLLER_GAIT_CONTROLLER_HPP
