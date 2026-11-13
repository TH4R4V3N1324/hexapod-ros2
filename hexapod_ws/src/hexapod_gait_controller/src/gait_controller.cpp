#include "hexapod_gait_controller/gait_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_map>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace {

/**
 * @brief Roll, pitch and yaw (rad) of a quaternion
*/
void toRollPitchYaw(const geometry_msgs::msg::Quaternion& o, double& roll, double& pitch, double& yaw) {
    roll = std::atan2(2.0 * (o.w * o.x + o.y * o.z), 1.0 - 2.0 * (o.x * o.x + o.y * o.y));
    pitch = std::asin(std::clamp(2.0 * (o.w * o.y - o.z * o.x), -1.0, 1.0));
    yaw = std::atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z));
}

}  // namespace

GaitController::GaitController() : Node("gait_controller") {
    cmd_vel_sub = this->create_subscription<Twist>("cmd_vel", 10, std::bind(&GaitController::cmdVelCallback, this, _1));
    joint_state_sub = this->create_subscription<JointState>("joint_states", 10, std::bind(&GaitController::jointStateCallback, this, _1));

    joint_cmd_pub = this->create_publisher<JointState>("joint_commands", 10);
    odom_pub = this->create_publisher<Odometry>("odometry/legs", 10);

    set_gait_service = this->create_service<SetGait>("set_gait", std::bind(&GaitController::setGaitCallback, this, _1, _2));
    set_mode_service = this->create_service<SetMode>("set_mode", std::bind(&GaitController::setModeCallback, this, _1, _2));
    set_height_service = this->create_service<SetHeight>("set_height", std::bind(&GaitController::setHeightCallback, this, _1, _2));
    get_capabilities_service = this->create_service<GetCapabilities>("get_capabilities", std::bind(&GaitController::getCapabilitiesCallback, this, _1, _2));
    sit_service = this->create_service<std_srvs::srv::SetBool>("sit", std::bind(&GaitController::sitCallback, this, _1, _2));
    // True from when a sit starts until a stand finishes, i.e. whenever the robot won't walk
    // (it starts seated). Latched, so a node that starts later still learns it.
    seated_pub = this->create_publisher<std_msgs::msg::Bool>("seated", rclcpp::QoS(1).transient_local());
    publishSeated(true);

    // Every gait setting goes into the engine's parameters; see gait_core::GaitParams for what each does
    GaitParams gp;
    double loop_rate_hz = this->declare_parameter("loop_rate", 100.0);
    gp.phase_duration = this->declare_parameter("phase_duration", gp.phase_duration);
    gp.min_phase_duration = this->declare_parameter("min_phase_duration", gp.min_phase_duration);
    gp.max_foot_speed = this->declare_parameter("max_foot_speed", gp.max_foot_speed);
    gp.max_joint_speed = this->declare_parameter("max_joint_speed", gp.max_joint_speed);
    gp.idle_return_delay = this->declare_parameter("idle_return_delay", gp.idle_return_delay);
    gp.startup_duration = this->declare_parameter("startup_duration", gp.startup_duration);
    gp.max_target_speed = this->declare_parameter("max_target_speed", gp.max_target_speed);
    // Mode at startup: 0 normal (no strafing, linear.y turns), 1 strafe (standard holonomic /cmd_vel)
    int initial_mode = this->declare_parameter("initial_mode", 0);
    if (initial_mode < 0 || initial_mode >= NUM_MODES) {
        RCLCPP_WARN(this->get_logger(), "initial_mode %d is not a valid mode, using normal", initial_mode);
        initial_mode = MODE_NORMAL;
    }
    gp.initial_mode = static_cast<uint8_t>(initial_mode);

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
    gp.tilt_max_angle = this->declare_parameter("tilt_max_angle", gp.tilt_max_angle);
    gp.tilt_max_translation = this->declare_parameter("tilt_max_translation", gp.tilt_max_translation);
    gp.tilt_angle_rate = this->declare_parameter("tilt_angle_rate", gp.tilt_angle_rate);
    gp.tilt_translation_rate = this->declare_parameter("tilt_translation_rate", gp.tilt_translation_rate);
    gp.tilt_leveling = this->declare_parameter("tilt_leveling", gp.tilt_leveling);
    // Off by default: the IMU yaw drifts without a magnetometer, and holding it uses up the stride
    gp.tilt_lock_yaw = this->declare_parameter("tilt_lock_yaw", gp.tilt_lock_yaw);
    gp.tilt_level_gain = this->declare_parameter("tilt_level_gain", gp.tilt_level_gain);
    // Foot forces from the joint efforts, and which feet are touching the ground
    gp.contact_force_on = this->declare_parameter("contact_force_on", gp.contact_force_on);
    gp.contact_force_off = this->declare_parameter("contact_force_off", gp.contact_force_off);
    gp.foot_force_filter = this->declare_parameter("foot_force_filter", gp.foot_force_filter);
    // Pushing against something: the feet's net horizontal force is a large share of the weight
    gp.blocked_force_ratio = this->declare_parameter("blocked_force_ratio", gp.blocked_force_ratio);
    // Spreading the weight between planted feet by raising or lowering each a little
    gp.load_balancing = this->declare_parameter("load_balancing", gp.load_balancing);
    gp.load_balance_gain = this->declare_parameter("load_balance_gain", gp.load_balance_gain);
    gp.load_balance_max = this->declare_parameter("load_balance_max", gp.load_balance_max);
    // foot_force/legN are for display only (the controller uses every update itself). At the joint
    // state rate they were 600 messages a second, enough to make Foxglove on the Deck lag.
    foot_force_period = 1.0 / this->declare_parameter("foot_force_rate", 20.0);
    for (int legNum = 0; legNum < NUM_LEGS; ++legNum) {
        foot_force_pubs[legNum] = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "foot_force/leg" + std::to_string(legNum + 1), 10);
    }

    // Uneven ground: swinging feet stop when they touch down early, or reach down to find it
    gp.terrain_adaptation = this->declare_parameter("terrain_adaptation", gp.terrain_adaptation);
    gp.touchdown_force = this->declare_parameter("touchdown_force", gp.touchdown_force);
    gp.max_step_height = this->declare_parameter("max_step_height", gp.max_step_height);
    gp.seek_speed = this->declare_parameter("seek_speed", gp.seek_speed);
    gp.body_height_rate = this->declare_parameter("body_height_rate", gp.body_height_rate);
    gp.step_height = this->declare_parameter("step_height", gp.step_height);
    // How much a swing eases in and out (0 even pace, 1 smootherstep)
    gp.swing_easing = this->declare_parameter("swing_easing", gp.swing_easing);
    // set_height glides the body to the new height rather than snapping the legs there
    gp.height_rate = this->declare_parameter("height_rate", gp.height_rate);
    gp.min_height = this->declare_parameter("min_height", gp.min_height);
    gp.max_height = this->declare_parameter("max_height", gp.max_height);
    // Sitting: feet out at sit_reach, sit_height below the hips
    gp.sit_reach = this->declare_parameter("sit_reach", gp.sit_reach);
    gp.sit_height = this->declare_parameter("sit_height", gp.sit_height);
    // Startup puts the robot in the seated pose; this stands it up straight away
    gp.stand_on_startup = this->declare_parameter("stand_on_startup", gp.stand_on_startup);
    gp.touchdown_ratio = this->declare_parameter("touchdown_ratio", gp.touchdown_ratio);
    gp.stumble_force = this->declare_parameter("stumble_force", gp.stumble_force);
    gp.stumble_lift = this->declare_parameter("stumble_lift", gp.stumble_lift);

    // Height map of the ground ahead from the depth camera, for choosing footholds and step heights.
    // Off by default: in the outdoor scenarios it caused more early touchdowns and stumbles, and
    // walked worse on hills, than sensing the ground with the feet alone.
    gp.terrain_map = this->declare_parameter("terrain_map", gp.terrain_map);
    map_frame = this->declare_parameter("map_frame", "odom");
    depth_stride = this->declare_parameter("depth_stride", 4);
    map_update_period = 1.0 / this->declare_parameter("map_update_rate", 5.0);
    map_radius = this->declare_parameter("map_radius", 1.0);
    foothold_search = this->declare_parameter("foothold_search", 0.03);
    foothold_edge = this->declare_parameter("foothold_edge", 0.025);
    foothold_flatness = this->declare_parameter("foothold_flatness", 0.01);
    step_clearance = this->declare_parameter("step_clearance", 0.03);
    map_height_deadband = this->declare_parameter("map_height_deadband", 0.01);
    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    depth_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        this->declare_parameter("depth_topic", "front_depth/points"), rclcpp::SensorDataQoS(),
        std::bind(&GaitController::depthCallback, this, _1));
    terrain_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("terrain_map", 1);
    terrain_map_timer = this->create_timer(std::chrono::milliseconds(500), std::bind(&GaitController::publishTerrainMap, this));

    // Leveling while walking (normal and strafe modes): the IMU keeps the body level on slopes
    gp.walk_leveling = this->declare_parameter("walk_leveling", gp.walk_leveling);
    gp.walk_level_gain = this->declare_parameter("walk_level_gain", gp.walk_level_gain);
    gp.walk_max_angle = this->declare_parameter("walk_max_angle", gp.walk_max_angle);

    engine = std::make_unique<GaitEngine>(gp);
    engine->log = [this](LogLevel level, const char* message) {
        switch (level) {
            case LogLevel::Debug: RCLCPP_DEBUG(this->get_logger(), "%s", message); break;
            case LogLevel::Info: RCLCPP_INFO(this->get_logger(), "%s", message); break;
            case LogLevel::Warn: RCLCPP_WARN(this->get_logger(), "%s", message); break;
            case LogLevel::Error: RCLCPP_ERROR(this->get_logger(), "%s", message); break;
        }
    };
    engine->seatedChanged = [this](bool seated) { publishSeated(seated); };
    engine->footholdPlanner = this;

    // The tilt and leveling parameters can be tuned while running
    param_callback = this->add_on_set_parameters_callback(
        std::bind(&GaitController::parametersCallback, this, _1));

    // ROS timer, so the gait follows simulation time when use_sim_time is set
    gait_timer = this->create_timer(std::chrono::duration<double>(nominal_dt), std::bind(&GaitController::gaitTimerCallback, this));

    // A SetBool service for each feature that can be switched on and off while running
    for (const char* feature : {"terrain_adaptation", "terrain_map", "walk_leveling", "tilt_leveling",
                                "tilt_lock_yaw", "load_balancing"}) {
        const std::string name = feature;
        toggle_services.push_back(this->create_service<std_srvs::srv::SetBool>("set_" + name,
            [this, name](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                toggleCallback(name, request, response);
            }));
    }

    RCLCPP_INFO(this->get_logger(), "GaitController node started at %.1f Hz, %.2f s per phase", loop_rate_hz, gp.phase_duration);
}

/**
 * @brief Store the latest velocity command and apply exponential smoothing
 * @param[in] msg The incoming velocity command
 * @note A zero command bypasses smoothing so the robot stops immediately
*/
void GaitController::cmdVelCallback(const Twist::SharedPtr msg) {
    const bool stop_command =
        msg->linear.x == 0.0 && msg->linear.y == 0.0 && msg->linear.z == 0.0 &&
        msg->angular.x == 0.0 && msg->angular.y == 0.0 && msg->angular.z == 0.0;
    if (stop_command) {
        filtered_cmd_vel = *msg;
        return;
    }

    // Smoothing factor - higher = faster response (0.5 = ~2 cycles to reach target)
    constexpr double alpha = 0.5;

    filtered_cmd_vel.linear.x = (1.0 - alpha) * filtered_cmd_vel.linear.x + alpha * msg->linear.x;
    filtered_cmd_vel.linear.y = (1.0 - alpha) * filtered_cmd_vel.linear.y + alpha * msg->linear.y;
    filtered_cmd_vel.angular.z = (1.0 - alpha) * filtered_cmd_vel.angular.z + alpha * msg->angular.z;
}

/**
 * @brief Pass the joint states to the engine, then publish the foot forces and leg odometry
 * @param[in] msg The incoming joint states
 * @note Joints are matched by name, since joint_state_broadcaster does not guarantee their order
*/
void GaitController::jointStateCallback(const JointState::SharedPtr msg) {
    std::unordered_map<std::string, double> positions;
    std::unordered_map<std::string, double> efforts;
    const bool has_effort = msg->effort.size() == msg->name.size();
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        positions[msg->name[i]] = msg->position[i];
        if (has_effort) {
            efforts[msg->name[i]] = msg->effort[i];
        }
    }

    const rclcpp::Time stamp(msg->header.stamp, this->get_clock()->get_clock_type());
    JointFeedback feedback;
    feedback.stamp = stamp.nanoseconds();
    feedback.hasEffort = has_effort;
    bool all_found = true;
    for (int legNum = 0; legNum < NUM_LEGS; ++legNum) {
        const std::string prefix = "leg" + std::to_string(legNum + 1) + "_";
        auto coxa = positions.find(prefix + "coxa_joint");
        auto femur = positions.find(prefix + "femur_joint");
        auto tibia = positions.find(prefix + "tibia_joint");
        if (coxa == positions.end() || femur == positions.end() || tibia == positions.end()) {
            all_found = false;
            continue;
        }
        feedback.present[legNum] = true;
        feedback.angles[legNum] = {coxa->second, femur->second, tibia->second};
        if (has_effort) {
            feedback.effort[legNum] = Vector3d(efforts[prefix + "coxa_joint"], efforts[prefix + "femur_joint"],
                                               efforts[prefix + "tibia_joint"]);
        }
    }

    const bool odometryUpdated = engine->onJointStates(feedback);

    // Foot forces for display, at foot_force_rate (a restarted simulation clock also counts as due)
    if (all_found && has_effort) {
        const bool publishForces = stamp < last_foot_force_publish ||
            (stamp - last_foot_force_publish).seconds() >= foot_force_period;
        if (publishForces) {
            last_foot_force_publish = stamp;
            for (int legNum = 0; legNum < NUM_LEGS; ++legNum) {
                if (!engine->footForceUpdated(legNum)) continue;
                const Vector3d& force = engine->footForce(legNum);
                geometry_msgs::msg::WrenchStamped wrench;
                wrench.header.stamp = stamp;
                wrench.header.frame_id = "base_link";
                wrench.wrench.force.x = force.x();
                wrench.wrench.force.y = force.y();
                wrench.wrench.force.z = force.z();
                foot_force_pubs[legNum]->publish(wrench);
            }
        }
    }

    if (odometryUpdated) {
        if (!has_odom_publish) {
            last_odom_publish = stamp;
            has_odom_publish = true;
        } else if ((stamp - last_odom_publish).seconds() >= odom_publish_period) {
            publishOdometry(stamp);
        }
    }
}

/**
 * @brief Publish the leg odometry pose, and the velocity since the last publish
 * @param[in] stamp The timestamp for the message
 * @note Twist is in the body frame. Covariances are rough: the feet can slip, and slip most when turning.
*/
void GaitController::publishOdometry(const rclcpp::Time& stamp) {
    const LegOdometry& legOdometry = engine->odometry();
    double dt = (stamp - last_odom_publish).seconds();
    double dx = legOdometry.x() - last_odom_x;
    double dy = legOdometry.y() - last_odom_y;
    double dyaw = std::remainder(legOdometry.yaw() - last_odom_yaw, 2.0 * M_PI);

    Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = odom_frame;
    msg.child_frame_id = base_frame;
    msg.pose.pose.position.x = legOdometry.x();
    msg.pose.pose.position.y = legOdometry.y();
    msg.pose.pose.position.z = engine->floorRise();  // base_footprint is on the floor; two_d_mode EKFs ignore it
    msg.pose.pose.orientation.z = std::sin(legOdometry.yaw() / 2.0);
    msg.pose.pose.orientation.w = std::cos(legOdometry.yaw() / 2.0);

    // Velocity in the body frame, using the heading at the start of the interval
    double c = std::cos(last_odom_yaw), s = std::sin(last_odom_yaw);
    msg.twist.twist.linear.x = (c * dx + s * dy) / dt;
    msg.twist.twist.linear.y = (-s * dx + c * dy) / dt;
    msg.twist.twist.angular.z = dyaw / dt;

    for (int i : {0, 7, 14, 21, 28, 35}) {
        msg.pose.covariance[i] = 1.0;  // the integrated pose drifts, prefer the EKF's
    }
    msg.twist.covariance[0] = 0.0025;   // vx, ~0.05 m/s
    msg.twist.covariance[7] = 0.0025;   // vy
    msg.twist.covariance[14] = 1e-6;    // vz, not moving vertically
    msg.twist.covariance[21] = 1e-6;    // roll rate
    msg.twist.covariance[28] = 1e-6;    // pitch rate
    msg.twist.covariance[35] = 0.04;    // yaw rate, ~0.2 rad/s
    odom_pub->publish(msg);

    last_odom_publish = stamp;
    last_odom_x = legOdometry.x();
    last_odom_y = legOdometry.y();
    last_odom_yaw = legOdometry.yaw();
}

/**
 * @brief Publish base_footprint, the point on the floor directly below base_link
 * @note base_footprint -> base_link is the body height plus the body's roll and pitch from the IMU.
 *       Odometry, SLAM and Nav2 use base_footprint, so z = 0 in odom is the floor and the costmap
 *       can tell the floor from low obstacles. Called every gait tick.
*/
void GaitController::publishFootprint() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = base_frame;
    t.child_frame_id = "base_link";
    t.transform.translation.z = engine->footprintHeight();
    const Eigen::Quaterniond q = Eigen::AngleAxisd(engine->imuPitch(), Vector3d::UnitY()) *
        Eigen::AngleAxisd(engine->imuRoll(), Vector3d::UnitX());
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    footprint_broadcaster->sendTransform(t);
}

/**
 * @brief Pass the body's orientation from the IMU to the engine
 * @param[in] msg The IMU message (orientation from imu_filter_madgwick on the robot, Gazebo in sim)
 * @note Messages without orientation are skipped
*/
void GaitController::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (msg->orientation_covariance[0] < 0.0) {
        return;
    }
    ImuSample sample;
    double roll, pitch, yaw;
    toRollPitchYaw(msg->orientation, roll, pitch, yaw);
    sample.roll = roll;
    sample.pitch = pitch;
    sample.yaw = yaw;
    sample.stamp = this->now().nanoseconds();
    engine->setImu(sample);
}

/**
 * @brief Apply changes to the parameters that can be tuned while running
 * @param[in] parameters The parameters being set
 * @return Always successful; other parameters only take effect at startup
*/
rcl_interfaces::msg::SetParametersResult GaitController::parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {
    GaitParams& gp = engine->params();
    for (const auto& parameter : parameters) {
        const auto& name = parameter.get_name();
        if (name == "tilt_leveling") gp.tilt_leveling = parameter.as_bool();
        else if (name == "tilt_lock_yaw") gp.tilt_lock_yaw = parameter.as_bool();
        else if (name == "tilt_level_gain") gp.tilt_level_gain = parameter.as_double();
        else if (name == "tilt_max_angle") gp.tilt_max_angle = parameter.as_double();
        else if (name == "tilt_max_translation") gp.tilt_max_translation = parameter.as_double();
        else if (name == "tilt_angle_rate") gp.tilt_angle_rate = parameter.as_double();
        else if (name == "tilt_translation_rate") gp.tilt_translation_rate = parameter.as_double();
        else if (name == "walk_leveling") gp.walk_leveling = parameter.as_bool();
        else if (name == "contact_force_on") gp.contact_force_on = parameter.as_double();
        else if (name == "contact_force_off") gp.contact_force_off = parameter.as_double();
        else if (name == "foot_force_filter") gp.foot_force_filter = parameter.as_double();
        else if (name == "load_balancing") gp.load_balancing = parameter.as_bool();
        else if (name == "terrain_adaptation") gp.terrain_adaptation = parameter.as_bool();
        else if (name == "touchdown_force") gp.touchdown_force = parameter.as_double();
        else if (name == "max_step_height") gp.max_step_height = parameter.as_double();
        else if (name == "seek_speed") gp.seek_speed = parameter.as_double();
        else if (name == "body_height_rate") gp.body_height_rate = parameter.as_double();
        else if (name == "height_rate") gp.height_rate = parameter.as_double();
        else if (name == "min_height") gp.min_height = parameter.as_double();
        else if (name == "max_height") gp.max_height = parameter.as_double();
        else if (name == "step_height") gp.step_height = parameter.as_double();
        else if (name == "swing_easing") gp.swing_easing = std::clamp(parameter.as_double(), 0.0, 1.0);
        else if (name == "touchdown_ratio") gp.touchdown_ratio = parameter.as_double();
        else if (name == "stumble_force") gp.stumble_force = parameter.as_double();
        else if (name == "stumble_lift") gp.stumble_lift = parameter.as_double();
        else if (name == "terrain_map") gp.terrain_map = parameter.as_bool();
        else if (name == "foothold_search") foothold_search = parameter.as_double();
        else if (name == "foothold_edge") foothold_edge = parameter.as_double();
        else if (name == "foothold_flatness") foothold_flatness = parameter.as_double();
        else if (name == "step_clearance") step_clearance = parameter.as_double();
        else if (name == "map_height_deadband") map_height_deadband = parameter.as_double();
        else if (name == "blocked_force_ratio") gp.blocked_force_ratio = parameter.as_double();
        else if (name == "load_balance_gain") gp.load_balance_gain = parameter.as_double();
        else if (name == "load_balance_max") gp.load_balance_max = parameter.as_double();
        else if (name == "walk_level_gain") gp.walk_level_gain = parameter.as_double();
        else if (name == "walk_max_angle") gp.walk_max_angle = parameter.as_double();
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

/**
 * @brief Store the body pose requested for tilt mode
 * @param[in] msg The pose: position is the body shift (m) and orientation its roll, pitch and yaw,
 *                relative to level and the heading when tilt mode started
*/
void GaitController::bodyPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    double roll, pitch, yaw;
    toRollPitchYaw(msg->orientation, roll, pitch, yaw);
    engine->setBodyPose(roll, pitch, yaw, Vector3d(msg->position.x, msg->position.y, msg->position.z));
}

void GaitController::setGaitCallback(
    const std::shared_ptr<SetGait::Request> request,
    std::shared_ptr<SetGait::Response> response
) {
    const CommandResult result = engine->setGait(request->gait);
    response->success = result.success;
    response->message = result.message;
}

void GaitController::setModeCallback(
    const std::shared_ptr<SetMode::Request> request,
    std::shared_ptr<SetMode::Response> response
) {
    const CommandResult result = engine->setMode(request->mode);
    response->success = result.success;
    response->message = result.message;
}

/**
 * @brief Service callback to change height
 * @note The request is the feet's z below the hips, so negative: -0.12 for a 12 cm body height
*/
void GaitController::setHeightCallback(
    const std::shared_ptr<SetHeight::Request> request,
    std::shared_ptr<SetHeight::Response> response
) {
    const CommandResult result = engine->setHeight(static_cast<double>(request->height));
    response->success = result.success;
    response->message = result.message;
}

/**
 * @brief Service callback to sit down (true) or stand up (false)
 * @note Returns straight away; the move carries on in the gait loop, and seated reports when it's done
*/
void GaitController::sitCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response
) {
    const CommandResult result = engine->sit(request->data);
    response->success = result.success;
    response->message = result.message;
}

/**
 * @brief Publish whether the robot is seated, or sitting down or standing up: anything but ready to walk
*/
void GaitController::publishSeated(bool seated) {
    std_msgs::msg::Bool msg;
    msg.data = seated;
    seated_pub->publish(msg);
}

/**
 * @brief Service callback to switch a feature on or off
 * @note Sets the parameter of the same name, so services and ros2 param always agree
*/
void GaitController::toggleCallback(
    const std::string& feature,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response
) {
    const auto result = this->set_parameter(rclcpp::Parameter(feature, static_cast<bool>(request->data)));
    response->success = result.successful;
    response->message = feature + (request->data ? " on" : " off");
    if (!result.successful) {
        response->message += " refused: " + result.reason;
    }
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

/**
 * @brief Service callback to report supported gaits and modes
*/
void GaitController::getCapabilitiesCallback(
    const std::shared_ptr<GetCapabilities::Request> /*request*/,
    std::shared_ptr<GetCapabilities::Response> response
) {
    auto toMsg = [](const gait_core::LocomotionOption& option) {
        hexapod_interfaces::msg::LocomotionOption msg;
        msg.name = option.name;
        msg.id = option.id;
        return msg;
    };
    const GaitConfig& gaitConfig = engine->gaitConfig();
    for (const auto& gait : gaitConfig.getGaits()) {
        response->gaits.push_back(toMsg(gait));
    }
    for (const auto& mode : gaitConfig.getModes()) {
        response->modes.push_back(toMsg(mode));
    }

    response->max_linear_vel = engine->maxFootSpeed();
    response->max_angular_vel = engine->maxFootSpeed() / engine->trajectory().maxBodyRadius();
    response->max_tilt_angle = engine->params().tilt_max_angle;
    response->max_tilt_translation = engine->params().tilt_max_translation;
    response->current_mode = gaitConfig.currentMode;
    response->current_gait = gaitConfig.gaitChangeRequested ? gaitConfig.pendingGait : gaitConfig.currentGait;
}

/**
 * @brief Main gait loop, called by the timer at loop_rate
 * @note Each tick advances the gait by the measured time since the last tick, clamped so a stalled
 *       tick cannot make the legs jump
*/
void GaitController::gaitTimerCallback() {
    constexpr double MAX_TICK_DT = 0.1;  // s

    rclcpp::Time now = this->now();
    double dt = has_last_tick ? (now - last_tick_time).seconds() : nominal_dt;
    dt = std::clamp(dt, 0.0, MAX_TICK_DT);
    last_tick_time = now;
    has_last_tick = true;

    publishFootprint();

    engine->setVelocity({filtered_cmd_vel.linear.x, filtered_cmd_vel.linear.y, filtered_cmd_vel.angular.z});
    const bool wasInitialized = engine->initialized();
    const GaitOutput output = engine->update(dt, now.nanoseconds());
    if (!wasInitialized && engine->initialized()) {
        heightMap.clear();  // anything seen before standing was mapped at the wrong height
    }
    if (output.commanded) {
        publishJointCommands(output);
    }
}

/**
 * @brief Publish the engine's joint targets as a joint command message
*/
void GaitController::publishJointCommands(const GaitOutput& output) {
    JointState msg;
    msg.name.reserve(18);
    msg.position.reserve(18);
    for (int legNum = 0; legNum < NUM_LEGS; ++legNum) {
        const std::string prefix = "leg" + std::to_string(legNum + 1) + "_";
        msg.name.push_back(prefix + "coxa_joint");
        msg.position.push_back(output.joints[legNum].coxa);
        msg.name.push_back(prefix + "femur_joint");
        msg.position.push_back(output.joints[legNum].femur);
        msg.name.push_back(prefix + "tibia_joint");
        msg.position.push_back(output.joints[legNum].tibia);
    }
    msg.header.stamp = this->now();
    if (joint_command_observer) {
        joint_command_observer(msg);
    }
    joint_cmd_pub->publish(msg);
}

/**
 * @brief Add a depth camera frame to the height map
 * @param[in] msg The point cloud
 * @note Only once the startup sequence is complete, and at most map_update_rate times a second.
 *       Uses every depth_stride-th row and column. Points are moved into map_frame with TF; their
 *       height is relative to the ground under the robot at startup (TF height plus floorRise).
 *       Cells further than map_radius from the robot are dropped.
*/
void GaitController::depthCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Not until the robot is standing: before that TF's body height is wrong (in simulation the
    // robot drops in from 0.5 m), and anything seen then would be mapped far too low
    if (!engine->params().terrain_map || !engine->initialized()) {
        return;
    }
    const rclcpp::Time stamp(msg->header.stamp, this->get_clock()->get_clock_type());
    if (has_map_update && (stamp - last_map_update).seconds() < map_update_period) {
        return;
    }
    last_map_update = stamp;
    has_map_update = true;
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer->lookupTransform(map_frame, msg->header.frame_id, msg->header.stamp,
                                        rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException& e) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Depth points not used: %s", e.what());
        return;
    }
    // The robot's own front feet and lower legs come into view at the end of their stride: drop
    // points within 6 cm (horizontally) of any foot, or they map as bumps right where it steps next
    constexpr double SELF_FILTER_RADIUS = 0.06;  // m
    Eigen::Isometry2d bodyToMapNow;
    std::vector<Vector2d> feetInMap;
    if (bodyToMap(bodyToMapNow)) {
        for (int legNum = 0; legNum < NUM_LEGS; ++legNum) {
            feetInMap.push_back(bodyToMapNow * engine->trajectory().legToBody(legNum, engine->measuredFoot(legNum)).head<2>());
        }
    }
    auto onOwnLeg = [&feetInMap](const Vector3d& p) {
        for (const auto& foot : feetInMap) {
            if ((p.head<2>() - foot).squaredNorm() < SELF_FILTER_RADIUS * SELF_FILTER_RADIUS) return true;
        }
        return false;
    };

    const auto& t = tf.transform;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z).toRotationMatrix();
    transform.translation() = Vector3d(t.translation.x, t.translation.y, t.translation.z);

    // Read only the rows and columns used, straight from the buffer: stepping through all ~300k
    // points of a frame took long enough to delay gait ticks
    int xOffset = -1, yOffset = -1, zOffset = -1;
    for (const auto& field : msg->fields) {
        if (field.datatype != sensor_msgs::msg::PointField::FLOAT32) continue;
        if (field.name == "x") xOffset = field.offset;
        else if (field.name == "y") yOffset = field.offset;
        else if (field.name == "z") zOffset = field.offset;
    }
    if (xOffset < 0 || yOffset < 0 || zOffset < 0) {
        return;
    }
    const double floorRise = engine->floorRise();
    const uint32_t stride = static_cast<uint32_t>(std::max(1, depth_stride));
    const uint32_t rows = std::max<uint32_t>(1, msg->height);
    std::vector<Vector3d> points;
    points.reserve((msg->width / stride + 1) * (rows / stride + 1));
    for (uint32_t row = 0; row < rows; row += stride) {
        for (uint32_t col = 0; col < msg->width; col += stride) {
            const size_t base = row * msg->row_step + col * msg->point_step;
            if (base + msg->point_step > msg->data.size()) break;
            float x, y, z;
            std::memcpy(&x, &msg->data[base + xOffset], sizeof(float));
            std::memcpy(&y, &msg->data[base + yOffset], sizeof(float));
            std::memcpy(&z, &msg->data[base + zOffset], sizeof(float));
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
            Vector3d p = transform * Vector3d(x, y, z);
            if (onOwnLeg(p)) continue;
            p.z() += floorRise;
            points.push_back(p);
        }
    }
    heightMap.insert(points);
    heightMap.prune(transform.translation().head<2>(), map_radius);
}

/**
 * @brief Publish the height map as a point cloud in map_frame, for viewing in RViz
 * @note One point per cell, at its height as TF sees it (so it lines up with the robot model)
*/
void GaitController::publishTerrainMap() {
    if (!engine->params().terrain_map || terrain_map_pub->get_subscription_count() == 0) {
        return;
    }
    const auto cells = heightMap.cellCentres();
    const double floorRise = engine->floorRise();
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = this->now();
    cloud.header.frame_id = map_frame;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(cells.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (const auto& c : cells) {
        *x = c.x(); *y = c.y(); *z = c.z() - floorRise;
        ++x; ++y; ++z;
    }
    terrain_map_pub->publish(cloud);
}

/**
 * @brief Current transform from the body's horizontal plane to the map
 * @param[out] transform Body (x, y) to map (x, y)
 * @return false if TF can't provide it yet
*/
bool GaitController::bodyToMap(Eigen::Isometry2d& transform) {
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer->lookupTransform(map_frame, base_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException&) {
        return false;
    }
    const auto& q = tf.transform.rotation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    transform = Eigen::Translation2d(tf.transform.translation.x, tf.transform.translation.y) * Eigen::Rotation2Dd(yaw);
    return true;
}

/**
 * @brief FootholdPlanner: whether the height map can be used this tick
*/
bool GaitController::ready() {
    return bodyToMap(foothold_body_to_map);
}

/**
 * @brief FootholdPlanner: use the height map to place a swinging foot's landing spot and set its lift
 * @param[in] legNum The leg
 * @param[in,out] target The landing target (leg frame); its height comes from the map, and it
 *                moves up to foothold_search to keep foothold_edge of flat ground around it
 * @param[in] swingStart Where the swing lifted off (leg frame)
 * @param[in,out] swingLift The swing's lift (m, 0 = the default step height)
 * @note Ground the camera hasn't seen leaves the target alone (force sensing still finds it), and
 *       so does a mapped height within map_height_deadband of the foot's own ground.
 *       The lift clears the highest seen ground between lift-off and landing by step_clearance.
*/
void GaitController::plan(int legNum, Vec3& target, const Vec3& swingStart, Scalar& swingLift) {
    const TrajectoryGenerator& trajectoryGen = engine->trajectory();
    const Eigen::Isometry2d& bodyToMapTransform = foothold_body_to_map;
    const double maxStepHeight = engine->params().max_step_height;
    const double floorRise = engine->floorRise();
    const Vector3d nominal = engine->gaitConfig().startPosition.at(legNum);
    const Vector3d bodyTarget = trajectoryGen.legToBody(legNum, target);
    Vector2d mapTarget = bodyToMapTransform * bodyTarget.head<2>();

    if (auto flat = heightMap.flatFoothold(mapTarget, foothold_search, foothold_edge, foothold_flatness)) {
        const Vector2d moved = bodyToMapTransform.inverse() * *flat;
        Vector3d legMoved = trajectoryGen.bodyToLeg(legNum, Vector3d(moved.x(), moved.y(), bodyTarget.z()));
        // Stay inside the stride circle
        const Vector2d offset = legMoved.head<2>() - nominal.head<2>();
        if (offset.norm() > MAX_STRIDE_LENGTH) {
            legMoved.head<2>() = nominal.head<2>() + offset * (MAX_STRIDE_LENGTH / offset.norm());
        }
        target.head<2>() = legMoved.head<2>();
        mapTarget = bodyToMapTransform * trajectoryGen.legToBody(legNum, target).head<2>();
    }
    if (auto ground = heightMap.height(mapTarget)) {
        const double mapped = std::clamp(*ground - floorRise, -maxStepHeight, maxStepHeight);
        if (std::abs(nominal.z() + mapped - target.z()) > map_height_deadband) {
            target.z() = nominal.z() + mapped;
        }
    }

    // Lift enough to clear the highest ground on the way (the foot peaks at ~0.88x the lift)
    const Vector2d mapStart = bodyToMapTransform * trajectoryGen.legToBody(legNum, swingStart).head<2>();
    if (auto highest = heightMap.maxAlong(mapStart, mapTarget, 0.02)) {
        const double ends = std::max(swingStart.z(), target.z()) - nominal.z();
        const double needed = (*highest - floorRise) - ends + step_clearance;
        const double wanted = std::clamp(needed / 0.88, trajectoryGen.swingLift, 0.12);
        // Changes during a swing reshape it: limit them to 0.2 m/s so the foot doesn't jerk
        const double current = swingLift > 0.0 ? swingLift : trajectoryGen.swingLift;
        const double maxChange = 0.2 * nominal_dt;
        swingLift = current + std::clamp(wanted - current, -maxChange, maxChange);
    }
}
