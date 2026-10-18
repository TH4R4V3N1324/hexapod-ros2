#include "hexapod_gait_controller/gait_controller.hpp"

#include <cstring>
#include <unordered_map>

#include "sensor_msgs/point_cloud2_iterator.hpp"

/**
 * @brief Entry point, initialises ROS and spins the node
 * @param[in] argc Number of command line arguments
 * @param[in] argv Command line arguments
 * @return Process exit code
*/
int main(int argc, char **argv){    
    rclcpp::init(argc, argv);    
    auto node = std::make_shared<GaitController>();    
    rclcpp::spin(node);    
    rclcpp::shutdown();    
    return 0;
}

/**
 * @brief Store the latest velocity command and apply exponential smoothing
 * @param[in] msg The incoming velocity command
 * @note A zero command bypasses smoothing so the robot stops immediately
*/
void GaitController::cmdVelCallback(const Twist::SharedPtr msg) {
    last_cmd_vel = *msg;

    const bool stop_command =
        msg->linear.x == 0.0 && msg->linear.y == 0.0 && msg->linear.z == 0.0 &&
        msg->angular.x == 0.0 && msg->angular.y == 0.0 && msg->angular.z == 0.0;
    if (stop_command) {
        filtered_cmd_vel = *msg;
        return;
    }

    // Smoothing factor - higher = faster response (0.5 = ~2 cycles to reach target)
    constexpr double alpha = 0.5;

    // Apply exponential smoothing for smooth velocity transitions
    filtered_cmd_vel.linear.x = (1.0 - alpha) * filtered_cmd_vel.linear.x + alpha * msg->linear.x;
    filtered_cmd_vel.linear.y = (1.0 - alpha) * filtered_cmd_vel.linear.y + alpha * msg->linear.y;
    filtered_cmd_vel.angular.z = (1.0 - alpha) * filtered_cmd_vel.angular.z + alpha * msg->angular.z;
}

/**
 * @brief Store the latest joint states and compute each foot's measured position with FK
 * @param[in] msg The incoming joint states
 * @note Joints are matched by name, since joint_state_broadcaster does not guarantee their order.
 *       has_joint_states is only set once all 18 joints have been received.
*/
void GaitController::jointStateCallback(const JointState::SharedPtr msg) {
    latest_joint_states = *msg;

    std::unordered_map<std::string, double> positions;
    std::unordered_map<std::string, std::pair<double, double>> joints;  // position, effort
    const bool has_effort = msg->effort.size() == msg->name.size();
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        positions[msg->name[i]] = msg->position[i];
        if (has_effort) {
            joints[msg->name[i]] = {msg->position[i], msg->effort[i]};
        }
    }

    bool all_found = true;
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        const std::string prefix = "leg" + std::to_string(legNum) + "_";
        auto coxa = positions.find(prefix + "coxa_joint");
        auto femur = positions.find(prefix + "femur_joint");
        auto tibia = positions.find(prefix + "tibia_joint");
        if (coxa == positions.end() || femur == positions.end() || tibia == positions.end()) {
            all_found = false;
            continue;
        }
        measured_leg_positions[legNum] = ikSolver.solveFK({coxa->second, femur->second, tibia->second});
    }
    has_joint_states = all_found;
    const rclcpp::Time stamp(msg->header.stamp, this->get_clock()->get_clock_type());
    if (all_found && has_effort) {
        updateFootForces(joints, stamp);
    }

    if (has_joint_states && positions_initialized) {
        updateLegOdometry(stamp);
    }
}

/**
 * @brief Estimate the force on each foot from its joint efforts, decide which feet are in contact
 *        and publish the forces
 * @param[in] joints Each joint's position (rad) and effort (N m), by joint name
 * @param[in] stamp The joint state timestamp
 * @note Forces are in the body frame and low-pass filtered (foot_force_filter). Contact switches
 *       on above contact_force_on of upward force and off below contact_force_off.
*/
void GaitController::updateFootForces(
    const std::unordered_map<std::string, std::pair<double, double>>& joints, const rclcpp::Time& stamp
) {
    double alpha = 1.0;
    if (has_foot_forces) {
        const double dt = std::max(0.0, (stamp - last_foot_force_stamp).seconds());
        alpha = foot_force_filter > 0.0 ? std::min(1.0, dt / foot_force_filter) : 1.0;
    }
    last_foot_force_stamp = stamp;
    // Published for display at foot_force_rate (a restarted simulation clock also counts as due)
    const bool publishForces = stamp < last_foot_force_publish ||
        (stamp - last_foot_force_publish).seconds() >= foot_force_period;
    if (publishForces) {
        last_foot_force_publish = stamp;
    }

    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        const std::string prefix = "leg" + std::to_string(legNum) + "_";
        const auto& coxa = joints.at(prefix + "coxa_joint");
        const auto& femur = joints.at(prefix + "femur_joint");
        const auto& tibia = joints.at(prefix + "tibia_joint");
        Vector3d legForce;
        if (!ikSolver.groundForce({coxa.first, femur.first, tibia.first},
                Vector3d(coxa.second, femur.second, tibia.second), legForce)) {
            continue;  // leg nearly straight: keep the last estimate
        }
        // Rotate into the body frame (a direction, so without the coxa offset)
        Vector3d bodyForce = trajectoryGen.legToBody(legNum, legForce) - trajectoryGen.legToBody(legNum, Vector3d::Zero());
        // Mid-swing the foot is certainly in the air, so whatever it reads is the leg's own weight
        if (leg_swinging[legNum] && phase_progress > 0.3 && phase_progress < 0.7) {
            foot_force_offset[legNum] += 0.02 * (-bodyForce.z() - foot_force_offset[legNum]);
        }
        bodyForce.z() += foot_force_offset[legNum];
        foot_forces[legNum] = has_foot_forces ? foot_forces[legNum] + alpha * (bodyForce - foot_forces[legNum]) : bodyForce;

        if (foot_forces[legNum].z() > contact_force_on) {
            foot_contact[legNum] = true;
        } else if (foot_forces[legNum].z() < contact_force_off) {
            foot_contact[legNum] = false;
        }

        if (publishForces) {
            geometry_msgs::msg::WrenchStamped wrench;
            wrench.header.stamp = stamp;
            wrench.header.frame_id = "base_link";
            wrench.wrench.force.x = foot_forces[legNum].x();
            wrench.wrench.force.y = foot_forces[legNum].y();
            wrench.wrench.force.z = foot_forces[legNum].z();
            foot_force_pubs[legNum]->publish(wrench);
        }
    }
    has_foot_forces = true;

    // Blocked: the ground's net horizontal push on the feet in contact is a large share of the
    // weight, so something else is pushing back (walking freely it's ~0.07, against a wall ~0.5).
    // Filtered over ~0.3 s, with hysteresis.
    Eigen::Vector2d horizontal = Eigen::Vector2d::Zero();
    double vertical = 0.0;
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        if (foot_contact[legNum]) {
            horizontal += foot_forces[legNum].head<2>();
            vertical += foot_forces[legNum].z();
        }
    }
    const double blockedAlpha = std::min(1.0, alpha * foot_force_filter / 0.3);
    push_ratio += blockedAlpha * ((vertical > 3.0 * contact_force_on ? horizontal.norm() / vertical : 0.0) - push_ratio);
    const bool wasBlocked = blocked;
    if (push_ratio > blocked_force_ratio) {
        blocked = true;
    } else if (push_ratio < 0.67 * blocked_force_ratio) {
        blocked = false;
    }
    if (blocked != wasBlocked) {
        RCLCPP_INFO(this->get_logger(), blocked ? "Pushing against something: odometry ignores the legs' movement"
                                                : "No longer pushing against anything");
    }
}

/**
 * @brief Estimate the body's motion from the planted feet and add it to the leg odometry
 * @param[in] stamp The joint state timestamp
 * @note Feet that are not swinging and are bearing weight are treated as planted. Called for every
 *       joint state message.
*/
void GaitController::updateLegOdometry(const rclcpp::Time& stamp) {
    // In simulation the joint states come faster than the clock ticks, so ~30% share a timestamp
    // with the previous one. Skip those entirely, keeping the last feet, so their motion is
    // counted with the next new timestamp; replacing the feet threw it away (odometry read 20% short).
    if (has_previous_feet && stamp <= previous_feet_stamp) {
        return;
    }
    std::array<Eigen::Vector2d, MAX_LEGS + 1> feet;
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        // In the level frame, so tilting the body to level it doesn't count as movement
        feet[legNum] = trajectoryGen.unposedBodyPoint(
            trajectoryGen.legToBody(legNum, measured_leg_positions[legNum]), walk_level).head<2>();
    }

    if (has_previous_feet) {
        // Only feet bearing weight, when the joint efforts say so, since a foot only stays put once
        // it's loaded. Without effort data (mock hardware) every non-swinging foot counts.
        double totalLoad = 0.0;
        for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
            totalLoad += std::max(0.0, foot_forces[legNum].z());
        }
        const bool useContact = has_foot_forces && totalLoad > 3.0 * contact_force_on;
        std::vector<Eigen::Vector2d> before, after;
        for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
            if (!isSwinging(legNum) && (!useContact || (foot_contact[legNum] && previous_contact[legNum]))) {
                before.push_back(previous_feet[legNum]);
                after.push_back(feet[legNum]);
            }
        }
        BodyMotion motion;
        if (LegOdometry::estimateMotion(before, after, motion)) {
            // Pushing against something, the feet scrub along the ground, so they say the body is
            // moving when it isn't: keep only the turn (which the EKF checks against the gyro).
            // Removing just the motion towards the obstacle still let a third through, because
            // the push direction swings about as the tripods swap.
            legOdometry.integrate(blocked ? BodyMotion{0.0, 0.0, motion.dyaw} : motion);
        }
    }
    previous_feet = feet;
    previous_contact = foot_contact;
    previous_feet_stamp = stamp;
    has_previous_feet = true;

    if (!has_odom_publish) {
        last_odom_publish = stamp;
        has_odom_publish = true;
    } else if ((stamp - last_odom_publish).seconds() >= odom_publish_period) {
        publishOdometry(stamp);
    }
}

/**
 * @brief Publish the leg odometry pose, and the velocity since the last publish
 * @param[in] stamp The timestamp for the message
 * @note Twist is in the body frame. Covariances are rough: the feet can slip, and slip most when turning.
*/
void GaitController::publishOdometry(const rclcpp::Time& stamp) {
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
 *       can tell the floor from low obstacles. Without the tilt, the ~0.8 deg the body rocks while
 *       walking put distant floor seen by the depth camera 5 cm too high, marking phantom obstacles.
 *       Called every gait tick.
*/
void GaitController::publishFootprint() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = base_frame;
    t.child_frame_id = "base_link";
    // Feet are at currentHeight below the hips, plus any lift from tilt mode
    t.transform.translation.z = -gaitConfig.currentHeight + tilt_applied.translation.z();
    Eigen::Quaterniond q = Eigen::AngleAxisd(body_pitch, Vector3d::UnitY()) * Eigen::AngleAxisd(body_roll, Vector3d::UnitX());
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    footprint_broadcaster->sendTransform(t);
}

/**
 * @brief Store the body's roll and pitch from the IMU's orientation
 * @param[in] msg The IMU message (orientation from imu_filter_madgwick on the robot, Gazebo in sim)
 * @note Yaw is only used by tilt mode to hold the heading; odometry provides it otherwise.
 *       Messages without orientation are skipped.
*/
void GaitController::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (msg->orientation_covariance[0] < 0.0) {
        return;
    }
    const auto& o = msg->orientation;
    body_roll = std::atan2(2.0 * (o.w * o.x + o.y * o.z), 1.0 - 2.0 * (o.x * o.x + o.y * o.y));
    body_pitch = std::asin(std::clamp(2.0 * (o.w * o.y - o.z * o.x), -1.0, 1.0));
    body_yaw = std::atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z));
    last_imu_time = this->now();
    has_imu = true;
}

/**
 * @brief Apply changes to the tilt parameters made while running
 * @param[in] parameters The parameters being set
 * @return Always successful; parameters other than the tilt ones only take effect at startup
*/
rcl_interfaces::msg::SetParametersResult GaitController::parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {
    for (const auto& parameter : parameters) {
        const auto& name = parameter.get_name();
        if (name == "tilt_leveling") tilt_leveling = parameter.as_bool();
        else if (name == "tilt_lock_yaw") tilt_lock_yaw = parameter.as_bool();
        else if (name == "tilt_level_gain") tilt_level_gain = parameter.as_double();
        else if (name == "tilt_max_angle") tilt_max_angle = parameter.as_double();
        else if (name == "tilt_max_translation") tilt_max_translation = parameter.as_double();
        else if (name == "tilt_angle_rate") tilt_angle_rate = parameter.as_double();
        else if (name == "tilt_translation_rate") tilt_translation_rate = parameter.as_double();
        else if (name == "walk_leveling") walk_leveling = parameter.as_bool();
        else if (name == "contact_force_on") contact_force_on = parameter.as_double();
        else if (name == "contact_force_off") contact_force_off = parameter.as_double();
        else if (name == "foot_force_filter") foot_force_filter = parameter.as_double();
        else if (name == "load_balancing") load_balancing = parameter.as_bool();
        else if (name == "terrain_adaptation") terrain_adaptation = parameter.as_bool();
        else if (name == "touchdown_force") touchdown_force = parameter.as_double();
        else if (name == "max_step_height") max_step_height = parameter.as_double();
        else if (name == "seek_speed") seek_speed = parameter.as_double();
        else if (name == "body_height_rate") body_height_rate = parameter.as_double();
        else if (name == "height_rate") height_rate = parameter.as_double();
        else if (name == "min_height") min_height = parameter.as_double();
        else if (name == "max_height") max_height = parameter.as_double();
        else if (name == "step_height") trajectoryGen.swingLift = parameter.as_double();
        else if (name == "touchdown_ratio") touchdown_ratio = parameter.as_double();
        else if (name == "stumble_force") stumble_force = parameter.as_double();
        else if (name == "stumble_lift") stumble_lift = parameter.as_double();
        else if (name == "terrain_map") terrain_map = parameter.as_bool();
        else if (name == "foothold_search") foothold_search = parameter.as_double();
        else if (name == "foothold_edge") foothold_edge = parameter.as_double();
        else if (name == "foothold_flatness") foothold_flatness = parameter.as_double();
        else if (name == "step_clearance") step_clearance = parameter.as_double();
        else if (name == "map_height_deadband") map_height_deadband = parameter.as_double();
        else if (name == "blocked_force_ratio") blocked_force_ratio = parameter.as_double();
        else if (name == "load_balance_gain") load_balance_gain = parameter.as_double();
        else if (name == "load_balance_max") load_balance_max = parameter.as_double();
        else if (name == "walk_level_gain") walk_level_gain = parameter.as_double();
        else if (name == "walk_max_angle") walk_max_angle = parameter.as_double();
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

/**
 * @brief Store the body pose requested for tilt mode
 * @param[in] msg The pose: position is the body shift (m) and orientation its roll, pitch and yaw,
 *                relative to level and the heading when tilt mode started
 * @note Each axis is clamped to tilt_max_translation or tilt_max_angle. Ignored outside tilt mode.
*/
void GaitController::bodyPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    const auto& o = msg->orientation;
    const double roll = std::atan2(2.0 * (o.w * o.x + o.y * o.z), 1.0 - 2.0 * (o.x * o.x + o.y * o.y));
    const double pitch = std::asin(std::clamp(2.0 * (o.w * o.y - o.z * o.x), -1.0, 1.0));
    const double yaw = std::atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z));
    body_pose_request.roll = std::clamp(roll, -tilt_max_angle, tilt_max_angle);
    body_pose_request.pitch = std::clamp(pitch, -tilt_max_angle, tilt_max_angle);
    body_pose_request.yaw = std::clamp(yaw, -tilt_max_angle, tilt_max_angle);
    body_pose_request.translation = Vector3d(msg->position.x, msg->position.y, msg->position.z)
        .cwiseMax(-tilt_max_translation).cwiseMin(tilt_max_translation);
}

/**
 * @brief Service callback to change gait
 * @param[in] request The service request
 * @param[out] response The service response
*/
void GaitController::setGaitCallback(
    const std::shared_ptr<SetGait::Request> request,
    std::shared_ptr<SetGait::Response> response
) {
    gaitConfig.setGait(static_cast<Gait>(request->gait));
    response->success = true;
    response->message = "Gait " + std::to_string(request->gait) + " requested, applies at the next step";
    RCLCPP_INFO(this->get_logger(), "Gait %d requested", request->gait);
}

/**
 * @brief Service callback to change mode
 * @param[in] request The service request
 * @param[out] response The service response
*/
void GaitController::setModeCallback(
    const std::shared_ptr<SetMode::Request> request,
    std::shared_ptr<SetMode::Response> response
) {
    if (request->mode >= NUM_MODES) {
        response->success = false;
        response->message = "Mode " + std::to_string(request->mode) + " is not a valid mode";
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    if (request->mode == MODE_TILT && sit_state != SitState::STANDING) {
        response->success = false;
        response->message = "Can't enter tilt mode while sitting";
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    gaitConfig.setMode(static_cast<Mode>(request->mode));
    response->success = true;
    response->message = "Mode changed to " + std::to_string(request->mode);
    RCLCPP_INFO(this->get_logger(), "Mode changed to %d", request->mode);
}

/**
 * @brief Service callback to change height
 * @param[in] request The service request
 * @param[out] response The service response
*/
void GaitController::setHeightCallback(
    const std::shared_ptr<SetHeight::Request> request,
    std::shared_ptr<SetHeight::Response> response
) {
    // The request is the feet's z below the hips, so negative: -0.12 for a 12 cm body height
    const double height = static_cast<double>(request->height);
    if (sit_state != SitState::STANDING) {
        response->success = false;
        response->message = "can't change height while sitting";
        RCLCPP_WARN(this->get_logger(), "Refused height %.3f: %s", height, response->message.c_str());
        return;
    }
    if (-height < min_height || -height > max_height) {
        response->success = false;
        response->message = "height must be between " + std::to_string(min_height) + " and " +
            std::to_string(max_height) + " m (sent as negative z)";
        RCLCPP_WARN(this->get_logger(), "Refused height %.3f: %s", height, response->message.c_str());
        return;
    }
    target_height = height;
    response->success = true;
    response->message = "moving to " + std::to_string(-height) + " m";
    RCLCPP_INFO(this->get_logger(), "Height target %.3f m", -height);
}

/**
 * @brief Service callback to sit down (true) or stand up (false)
 * @param[in] request data is true to sit, false to stand
 * @param[out] response The service response
 * @note Returns straight away; the move carries on in the gait loop, and seated reports when it's
 *       done. A request part way through reverses the move from wherever it has got to.
*/
void GaitController::sitCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response
) {
    response->success = true;
    if (request->data) {
        switch (sit_state) {
            case SitState::STANDING:
                if (!reachable(Vector3d(sit_reach, 0.0, -sit_height)) ||
                    !reachable(Vector3d(sit_reach, 0.0, stand_height))) {
                    response->success = false;
                    response->message = "sit_reach and sit_height put a foot out of reach";
                    RCLCPP_WARN(this->get_logger(), "Refused to sit: %s", response->message.c_str());
                    return;
                }
                stand_height = target_height;
                // Tilt mode eases the body back to neutral before the sit starts
                gaitConfig.setMode(MODE_NORMAL);
                gaitConfig.setReach(sit_reach);
                sit_state = SitState::SPREADING;
                break;
            case SitState::GATHERING:
                gaitConfig.setReach(sit_reach);
                sit_state = SitState::SPREADING;
                break;
            case SitState::RAISING:
                target_height = -sit_height;
                sit_state = SitState::LOWERING;
                break;
            case SitState::SPREADING:
            case SitState::LOWERING:
            case SitState::SEATED:
                response->message = "already sitting";
                return;
        }
        response->message = "sitting down";
    } else {
        switch (sit_state) {
            case SitState::SEATED:
            case SitState::LOWERING:
                target_height = stand_height;
                sit_state = SitState::RAISING;
                publishSeated();
                break;
            case SitState::SPREADING:
                gaitConfig.setReach(stand_reach);
                sit_state = SitState::GATHERING;
                break;
            case SitState::STANDING:
            case SitState::RAISING:
            case SitState::GATHERING:
                response->message = "already standing";
                return;
        }
        response->message = "standing up";
    }
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

/**
 * @brief Publish whether the robot is seated
*/
void GaitController::publishSeated() {
    std_msgs::msg::Bool msg;
    msg.data = sit_state == SitState::SEATED;
    seated_pub->publish(msg);
}

/**
 * @brief Sitting down or standing up, one tick at a time
 * @param[in] dt The time since the last tick (s)
 * @note Sitting steps the feet out to sit_reach a gait group at a time (returnToStart, aiming at
 *       the widened start positions), then lowers the body onto the ground. Standing up is the
 *       reverse. /cmd_vel is ignored throughout, load balancing and walk leveling unwind, and
 *       the gait loop's height glide does the lowering and raising.
*/
void GaitController::sit(double dt) {
    current_motion_intent = MotionIntent{};
    updateWalkLeveling(dt);

    switch (sit_state) {
        case SitState::SPREADING:
        case SitState::GATHERING:
            if (!idleReturning) {
                if (!feetAtStart()) {
                    idleReturning = true;
                    phase_progress = 0.0;
                    leg_swinging.fill(false);
                } else if (sit_state == SitState::SPREADING) {
                    target_height = -sit_height;
                    sit_state = SitState::LOWERING;
                } else {
                    sit_state = SitState::STANDING;
                    RCLCPP_INFO(this->get_logger(), "Standing");
                }
            }
            if (idleReturning) {
                returnToStart(dt);
                return;
            }
            break;
        case SitState::LOWERING:
            if (gaitConfig.currentHeight == target_height) {
                sit_state = SitState::SEATED;
                publishSeated();
                RCLCPP_INFO(this->get_logger(), "Seated");
            }
            break;
        case SitState::RAISING:
            if (gaitConfig.currentHeight == target_height) {
                gaitConfig.setReach(stand_reach);
                sit_state = SitState::GATHERING;
            }
            break;
        case SitState::SEATED:
        case SitState::STANDING:
            break;
    }

    // Hold the feet where they are; the height glide has already moved them this tick
    sendFootTargets(current_leg_positions, dt);
}


/**
 * @brief Service callback to switch a feature on or off
 * @param[in] feature The feature's parameter name, e.g. "terrain_adaptation"
 * @param[in] request data: true to switch it on, false to switch it off
 * @param[out] response Whether the change was accepted, and the feature's state
 * @note Sets the parameter of the same name, so services and ros2 param always agree, and the
 *       change takes effect straight away (through parametersCallback)
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
 * @param[in] request The service request
 * @param[out] response The service response
*/
void GaitController::getCapabilitiesCallback(
    const std::shared_ptr<GetCapabilities::Request> request,
    std::shared_ptr<GetCapabilities::Response> response
) {
    auto toMsg = [this](const hexapod_gait_controller::LocomotionOption& option) {
        hexapod_interfaces::msg::LocomotionOption msg;
        msg.name = option.name;
        msg.id = option.id;
        return msg;
    };

    // Populate gaits
    for (const auto& gait : gaitConfig.getGaits()) {
        response->gaits.push_back(toMsg(gait));
    }
    // Populate modes
    for (const auto& mode : gaitConfig.getModes()) {
        response->modes.push_back(toMsg(mode));
    }

    response->max_linear_vel = maxFootSpeed();
    response->max_angular_vel = maxFootSpeed() / trajectoryGen.maxBodyRadius();
    response->max_tilt_angle = tilt_max_angle;
    response->max_tilt_translation = tilt_max_translation;
    response->current_mode = gaitConfig.currentMode;
    response->current_gait = gaitConfig.gaitChangeRequested ? gaitConfig.pendingGait : gaitConfig.currentGait;
}

/**
 * @brief Fastest a foot can travel relative to the body in the current gait
 * @return The speed (m/s), which is also the gait's top walking speed
 * @note A stance foot sweeps up to 2 x MAX_STRIDE_LENGTH over (groups - 1) phases,
 *       and phases can be no shorter than min_phase_duration
*/
double GaitController::maxFootSpeed() {
    size_t groups = std::max<size_t>(2, gaitConfig.getGaitConfig(gaitConfig.currentGait).size());
    return 2.0 * MAX_STRIDE_LENGTH / ((groups - 1) * min_phase_duration);
}

/**
 * @brief Main gait loop - called by timer at fixed rate
 * @note This replaces your embedded while(true) loop.
 *       Each tick advances the gait by the measured time since the last tick,
 *       clamped so a stalled tick cannot make the legs jump.
*/
void GaitController::gaitTimerCallback() {
    constexpr double MAX_TICK_DT = 0.1;  // s

    rclcpp::Time now = this->now();
    double dt = has_last_tick ? (now - last_tick_time).seconds() : nominal_dt;
    dt = std::clamp(dt, 0.0, MAX_TICK_DT);
    last_tick_time = now;
    has_last_tick = true;

    publishFootprint();

    // Run the startup sequence until the legs reach their start positions
    if (!positions_initialized) {
        positions_initialized = startup(dt);
        return;
    }

    updateLoadBalance(dt);

    // Glide towards the height set_height asked for, moving every foot by the same amount. Planted
    // feet follow the start positions; swings, the last commanded positions (which a swing restarts
    // from) and tilt mode's ground points carry their own heights, so they're shifted too. Standing
    // still, half the legs sit at the start of a swing that never advances, and without this they
    // held the old height until the robot walked, then jumped to the new one in a single step.
    if (gaitConfig.currentHeight != target_height) {
        const double step = height_rate * dt;
        const double delta = std::clamp(target_height - gaitConfig.currentHeight, -step, step);
        gaitConfig.setHeight(gaitConfig.currentHeight + delta);
        for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
            current_leg_positions[legNum].z() += delta;
            swing_start[legNum].z() += delta;
            swing_target[legNum].z() += delta;
            tilt_ground_feet[legNum].z() += delta;
        }
    }

    // Tilt mode, or easing the body back to neutral after leaving it
    if (gaitConfig.currentMode == Mode::MODE_TILT || tilt_active) {
        tilt(dt, gaitConfig.currentMode != Mode::MODE_TILT);
        return;
    }

    if (sit_state != SitState::STANDING) {
        sit(dt);
        return;
    }

    switch (gaitConfig.currentMode) {
        case Mode::MODE_STRAFE:
            current_motion_intent.forward = filtered_cmd_vel.linear.x;
            current_motion_intent.left = filtered_cmd_vel.linear.y;
            current_motion_intent.yaw = filtered_cmd_vel.angular.z;
            break;
        case Mode::MODE_NORMAL:
            // No strafing in this mode. Turning comes from angular.z (as Nav2 sends it) plus
            // linear.y, scaled so the gamepad's full linear range maps to the full yaw range
            current_motion_intent.forward = filtered_cmd_vel.linear.x;
            current_motion_intent.left = 0.0;
            current_motion_intent.yaw = filtered_cmd_vel.angular.z + filtered_cmd_vel.linear.y / trajectoryGen.maxBodyRadius();
            break;
        case Mode::MODE_TILT:  // handled by tilt() above
        case Mode::NUM_MODES:
        default:
            RCLCPP_WARN(this->get_logger(), "Invalid or unimplemented mode");
            break;
    }
    updateWalkLeveling(dt);
    walk(dt);
}

/**
 * @brief Solve IK for a leg target and append the joint angles to a joint command message
 * @param[in,out] joint_state_msg The joint command message to append to
 * @param[in] target The foot target position (leg frame)
 * @param[in] legNum The leg number (1-6)
 * @note Also records target as the leg's current position
*/
void GaitController::appendJointCommand(JointState& joint_state_msg, const Vector3d& target, int legNum) {
    // Compute IK to get joint angles, with the body leveled while walking
    // and each foot raised or lowered by load balancing
    auto angles = ikSolver.solveIK(trajectoryGen.posedFoot(legNum, target, walk_level)
        + Vector3d(0.0, 0.0, foot_height_offset[legNum]));
    
    // Add joint names and positions
    joint_state_msg.name.push_back("leg" + std::to_string(legNum) + "_coxa_joint");
    joint_state_msg.position.push_back(angles.coxa);
    
    joint_state_msg.name.push_back("leg" + std::to_string(legNum) + "_femur_joint");
    joint_state_msg.position.push_back(angles.femur);
    
    joint_state_msg.name.push_back("leg" + std::to_string(legNum) + "_tibia_joint");
    joint_state_msg.position.push_back(angles.tibia);

    current_leg_positions[legNum] = target;
}

/**
 * @brief Stamp and publish a joint command message, then clear it for reuse
 * @param[in,out] msg The joint command message to publish
*/
void GaitController::sendJointcommands(JointState& msg) {
    msg.header.stamp = this->now();
    joint_cmd_pub->publish(msg);
    msg.name.clear();
    msg.position.clear();
}

/**
 * @brief Move all legs to the home position and deactivate servos
*/
void GaitController::home() {
    auto joint_state_msg = JointState();

    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        appendJointCommand(joint_state_msg, gaitConfig.homePos, legNum);
    }
    // Publish joint states
    sendJointcommands(joint_state_msg);
    rclcpp::sleep_for(std::chrono::milliseconds(500));  // Wait for legs to reach home position
    RCLCPP_INFO(this->get_logger(), "Moved to home position");
}

/**
 * @brief Startup sequence to move legs from where they are to their start positions
 * @param[in] dt The time since the last tick (s)
 * @return true once the sequence is complete, false while it is still running
 * @note Waits for joint states first, so the move starts from the measured leg positions
 *       instead of assuming a pose. The move takes startup_duration, or longer if a joint
 *       would otherwise exceed max_joint_speed.
*/
bool GaitController::startup(double dt) {
    if (!startup_planned) {
        if (!has_joint_states) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for joint states from all 18 joints");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Moving legs from their measured positions to the start positions");
        current_leg_positions = measured_leg_positions;
        startup_start_positions = measured_leg_positions;
        startup_progress = 0.0;
        startup_planned = true;
    }

    // Reset phase to 0 when starting up
    phase = 0;

    // Update start positions based on current height
    gaitConfig.setHeight(gaitConfig.currentHeight);

    // Straight line from the measured position to the start position
    auto targetsAt = [this](double progress) {
        std::array<Vector3d, MAX_LEGS + 1> targets;
        for (int leg = 1; leg <= MAX_LEGS; ++leg) {
            const Vector3d& start = startup_start_positions[leg];
            targets[leg] = start + (gaitConfig.startPosition.at(leg) - start) * progress;
        }
        return targets;
    };

    double ds = dt / startup_duration;
    double scale = jointSpeedScale(targetsAt(std::min(1.0, startup_progress + ds)), dt);
    startup_progress = std::min(1.0, startup_progress + ds * scale);

    auto targets = targetsAt(startup_progress);
    auto joint_state_msg = JointState();
    for (int leg = 1; leg <= MAX_LEGS; ++leg) {
        appendJointCommand(joint_state_msg, targets[leg], leg);
    }
    sendJointcommands(joint_state_msg);

    if (startup_progress >= 1.0) {
        heightMap.clear();
        RCLCPP_INFO(this->get_logger(), "Startup sequence complete");
        return true;
    }
    return false;
}

/**
 * @brief Evaluate every leg's trajectory target after advancing it by ds, without changing the splines
 * @param[in] ds How far to advance each spline (fraction of a phase)
 * @return The target foot position of each leg (leg frame)
 * @note Legs with no active swing or stance spline hold their commanded position
*/
std::array<Vector3d, MAX_LEGS + 1> GaitController::legTargets(double ds) {
    std::array<Vector3d, MAX_LEGS + 1> targets;
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        // Each leg is assigned EITHER swing OR stance trajectory for this phase (not both)
        const SwingSpline& spline = trajectoryGen.gaitState.swingSplines[legNum];
        const LineSpline& stanceSpline = trajectoryGen.gaitState.stanceSplines[legNum];

        if (spline.active) {
            double s = std::min(1.0, spline.s + ds);
            targets[legNum] = (s >= 1.0) ? spline.P4
                : limitSwingLift(legNum, trajectoryGen.evalBezier(spline.P0, spline.P1, spline.P2, spline.P3, spline.P4, s),
                                 spline.P0.z() + (spline.P4.z() - spline.P0.z()) * s);
        } else if (stanceSpline.active) {
            double s = std::min(1.0, stanceSpline.s + ds);
            targets[legNum] = stanceSpline.P0 + (stanceSpline.P1 - stanceSpline.P0) * s;
        } else {
            targets[legNum] = current_leg_positions[legNum];
        }
    }
    return targets;
}

/**
 * @brief Work out how much to slow this tick so no joint moves faster than max_joint_speed
 * @param[in] targets The foot target of each leg for this tick (leg frame)
 * @param[in] dt The time since the last tick (s)
 * @return A factor from 0 to 1 to scale this tick's progress by (1 = full speed)
 * @note Compares the joint angles of the targets with those of the last commanded positions.
 *       Joint motion is close to linear over one tick, so scaling progress scales joint speed.
*/
double GaitController::jointSpeedScale(const std::array<Vector3d, MAX_LEGS + 1>& targets, double dt) {
    double max_delta = 0.0;
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        JointAngles from = ikSolver.solveIK(current_leg_positions[legNum]);
        JointAngles to = ikSolver.solveIK(targets[legNum]);
        max_delta = std::max({max_delta,
            std::abs(to.coxa - from.coxa),
            std::abs(to.femur - from.femur),
            std::abs(to.tibia - from.tibia)});
    }
    double allowed = max_joint_speed * dt;
    return (max_delta > allowed) ? allowed / max_delta : 1.0;
}

/**
 * @brief Perform a leg step based on the current trajectories
 * @param[in] dt The time since the last tick (s)
 * @param[in] idle Whether the joystick is idle
 * @param[in] handlePhaseTransition Whether to handle phase transitions
 * @note If idle is true, the step will not advance
 * @note If handlePhaseTransition is false, phase transitions will be skipped and must be handled externally
 * @note Progress is dt / phase_duration per tick, reduced if a joint would exceed max_joint_speed.
 *       All legs share the same progress, so they stay in phase when the gait slows down.
*/
void GaitController::PerformLegStep(double dt, bool idle, bool handlePhaseTransition) {
    double ds = idle ? 0.0 : dt / current_phase_duration;
    std::array<Vector3d, MAX_LEGS + 1> targets = legTargets(ds);
    double scale = jointSpeedScale(targets, dt);
    if (scale < 1.0) {
        ds *= scale;
        targets = legTargets(ds);
    }

    // Advance the splines and deactivate any that have reached their end
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        SwingSpline& spline = trajectoryGen.gaitState.swingSplines[legNum];
        LineSpline& stanceSpline = trajectoryGen.gaitState.stanceSplines[legNum];
        if (spline.active) {
            spline.s = std::min(1.0, spline.s + ds);
            spline.active = spline.s < 1.0;
        } else if (stanceSpline.active) {
            stanceSpline.s = std::min(1.0, stanceSpline.s + ds);
            stanceSpline.active = stanceSpline.s < 1.0;
        }
    }

    sendFootTargets(targets, dt);

    phase_progress = std::min(1.0, phase_progress + ds);
    if (!handlePhaseTransition) return;

    // Phase transition
    if (phase_progress >= 1.0) {
        phase = (phase + 1) % trajectoryGen.gaitState.config.size();
        phase_progress = 0.0;
    }
}

/**
 * @brief Switch to a requested gait, if there is one
 * @note Only call between phases, when no leg is mid-swing. Restarts at the new gait's first phase.
*/
void GaitController::applyPendingGait() {
    if (!gaitConfig.gaitChangeRequested) {
        return;
    }
    gaitConfig.gaitChangeRequested = false;
    if (gaitConfig.pendingGait == gaitConfig.currentGait) {
        return;
    }
    gaitConfig.currentGait = gaitConfig.pendingGait;
    trajectoryGen.gaitState.config = gaitConfig.getGaitConfig(gaitConfig.currentGait);
    phase = 0;
    phase_progress = 0.0;
    leg_swinging.fill(false);
    for (const auto& option : gaitConfig.getGaits()) {
        if (option.id == gaitConfig.currentGait) {
            RCLCPP_INFO(this->get_logger(), "Gait changed to %s", option.name.c_str());
        }
    }
}

/**
 * @brief Command every foot towards its target, limited to max_foot_speed
 * @param[in] targets The foot target of each leg (leg frame)
 * @param[in] dt The time since the last tick (s)
*/
void GaitController::sendFootTargets(const std::array<Vector3d, MAX_LEGS + 1>& targets, double dt) {
    auto joint_state_msg = JointState();
    joint_state_msg.name.reserve(18);
    joint_state_msg.position.reserve(18);

    const double maxStep = max_foot_speed * dt;
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        Eigen::Vector3d targetPos = targets[legNum];
        Eigen::Vector3d delta = targetPos - current_leg_positions[legNum];
        if (delta.norm() > maxStep) {
            targetPos = current_leg_positions[legNum] + delta.normalized() * maxStep;
        }
        appendJointCommand(joint_state_msg, targetPos, legNum);
    }
    sendJointcommands(joint_state_msg);
}

/**
 * @brief Whether a leg is currently in the air
 * @param[in] legNum The leg number
 * @return true if the leg is swinging, while walking or returning to start
*/
bool GaitController::isSwinging(int legNum) const {
    return leg_swinging[legNum] || trajectoryGen.gaitState.swingSplines[legNum].active;
}

/**
 * @brief A leg's start position on its own patch of ground
 * @param[in] legNum The leg number
 * @return The start position (leg frame), raised or lowered by the leg's terrain offset
*/
Vector3d GaitController::plantedStart(int legNum) {
    return gaitConfig.startPosition.at(legNum) + Vector3d(0.0, 0.0, terrain_offset[legNum]);
}

/**
 * @brief Whether the foot forces can be trusted
 * @return true once the forces add up to a robot standing on its feet; false without effort data
 *         (mock hardware reports none) or with the robot lifted
*/
bool GaitController::forcesValid() {
    double total = 0.0;
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        total += std::max(0.0, foot_forces[legNum].z());
    }
    return has_foot_forces && total > 3.0 * contact_force_on;
}

/**
 * @brief Follow uneven ground with the feet
 * @param[in] dt The time since the last tick (s)
 * @note A swinging foot that meets the ground in the second half of its swing stops there and
 *       becomes a stance foot, so stepping onto something higher doesn't push the body up. One
 *       that finishes its swing without touching down keeps reaching down at seek_speed until it
 *       does. Each foot remembers its ground height (up to max_step_height either way), stays on
 *       it in stance and aims for it on its next swing. The body rises or falls at
 *       body_height_rate so the feet's average ground is back at nominal; slopes are left to
 *       leveling. Off, or without force data, the offsets ease back to zero.
*/
void GaitController::adaptToTerrain(double dt) {
    const Vector3d nominal = gaitConfig.startPosition.at(1);  // all legs share the start height
    if (!terrain_adaptation || !forcesValid()) {
        seeking.fill(false);
        landing_check.fill(false);
    } else {
        for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
            const Vector3d& force = foot_forces[legNum];
            const double sideways = force.head<2>().norm();
            const bool loaded = force.z() > touchdown_force;
            // Landing pushes up; catching an edge on the way across pushes back
            const bool landed = loaded && force.z() > touchdown_ratio * sideways;
            // Only once the foot is up: at lift-off it still carries its stance forces. Blocked means
            // pushed back without bearing weight: a foot coming down still moving forward pushes both
            // ways, and counting that as a stumble lifted every front foot again as it landed. It has
            // to last 3 ticks, since the force filter still carries the push-off as the window opens.
            const bool blocked = leg_swinging[legNum] && phase_progress > 0.2 && phase_progress < 0.9 &&
                sideways > stumble_force && !loaded;
            stumble_ticks[legNum] = blocked ? stumble_ticks[legNum] + 1 : 0;
            if (stumble_ticks[legNum] >= 3 && stumble_target[legNum] == 0.0) {
                stumble_target[legNum] = stumble_lift;
                RCLCPP_DEBUG(this->get_logger(), "Leg %d stumbled at s=%.2f (%.1f N sideways, %.1f N up)",
                    legNum, phase_progress, sideways, force.z());
            }
            // Only well above where it was aiming: touching a little early is just a normal landing,
            // and counting it as higher ground made the body creep upwards on flat floor
            const bool aboveTarget = current_leg_positions[legNum].z() - swing_target[legNum].z() > 0.01;
            // It has to look landed for 3 ticks in a row: a one-tick force spike mid-swing once
            // planted a foot 33 mm up on flat floor
            touchdown_ticks[legNum] = (leg_swinging[legNum] && phase_progress > 0.6 && landed && aboveTarget)
                ? touchdown_ticks[legNum] + 1 : 0;
            if (touchdown_ticks[legNum] >= 3) {
                // Touched down early: plant the foot here
                touchdown_ticks[legNum] = 0;
                leg_swinging[legNum] = false;
                terrain_offset[legNum] = std::clamp(current_leg_positions[legNum].z() - nominal.z(),
                    -max_step_height, max_step_height);
                RCLCPP_DEBUG(this->get_logger(), "Leg %d touched down %.1f mm high", legNum, terrain_offset[legNum] * 1000.0);
            } else if (landing_check[legNum] && !leg_swinging[legNum]) {
                // Its swing ended unloaded: if it is still unloaded once the weight has had time to
                // come onto it, the ground is lower than expected
                if (force.z() > contact_force_off) {
                    landing_check[legNum] = false;
                } else if ((unloaded_time[legNum] += dt) > 0.15) {
                    landing_check[legNum] = false;
                    seeking[legNum] = true;
                }
            } else if (seeking[legNum] && !leg_swinging[legNum]) {
                if (force.z() > contact_force_on || terrain_offset[legNum] <= -max_step_height) {
                    seeking[legNum] = false;
                    RCLCPP_DEBUG(this->get_logger(), "Leg %d found ground %.1f mm low", legNum, -terrain_offset[legNum] * 1000.0);
                } else {
                    terrain_offset[legNum] = std::max(-max_step_height, terrain_offset[legNum] - seek_speed * dt);
                }
            }
        }
    }

    // Body height follows the average ground (or the offsets unwind when adaptation is off)
    double mean = 0.0;
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        mean += terrain_offset[legNum] / MAX_LEGS;
    }
    const double shift = std::clamp(mean, -body_height_rate * dt, body_height_rate * dt);
    ground_level += shift;  // the body rises by shift
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        terrain_offset[legNum] -= shift;
        if (!terrain_adaptation || !forcesValid()) {
            terrain_offset[legNum] -= std::clamp(terrain_offset[legNum], -body_height_rate * dt, body_height_rate * dt);
        }
    }
}

/**
 * @brief Whether every foot is at its start position
 * @return true if every commanded foot is within 1 mm of its start position
*/
bool GaitController::feetAtStart() {
    // 1 mm: stopping can leave a foot a few mm up, frozen near the end of its swing, and a
    // looser tolerance counted it as down, so it was never planted
    constexpr double POSITION_TOLERANCE = 0.001;  // m
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        if ((current_leg_positions[legNum] - plantedStart(legNum)).norm() > POSITION_TOLERANCE) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Returns the hexapod to its start position
 * @param[in] dt The time since the last tick (s)
*/
void GaitController::returnToStart(double dt) {
    static int counter = 0;

    // Safety check: phase must be valid
    if (phase >= trajectoryGen.gaitState.config.size()) {
        idleReturning = false;
        counter = 0;
        phase_progress = 0.0;
        return;
    }

    if (feetAtStart()) {
        // Already at start, finish immediately
        idleReturning = false;
        counter = 0;
        phase_progress = 0.0;
        phase = 0;
        return;
    }

    if (phase_progress == 0.0) {
        current_phase_duration = phase_duration;
        trajectoryGen.GenerateTrajectories(
            // Swing: move to start position
            [this](int legNum, const Vector3d& currentPos) {
                (void)currentPos;  // Use actual current position from current_leg_positions
                return plantedStart(legNum);
            },
            // Stance: hold current position
            [](int, const Vector3d& currentPos) {
                return currentPos;
            },
            current_leg_positions,  // Use tracked current positions
            phase
        );
    }

    // Move all legs for this step
    PerformLegStep(dt, false, false);

    // Phase transition
    if (phase_progress >= 1.0) {
        counter++;
        phase_progress = 0.0;
        phase = (phase + 1) % trajectoryGen.gaitState.config.size();

        // After all phases, finish return-to-start and handle gait change if requested
        if (counter >= static_cast<int>(trajectoryGen.gaitState.config.size())) {
            counter = 0;
            idleReturning = false;

            applyPendingGait();
        }
    }
}

/**
 * @brief Handle idle/return-to-start logic
 * @param[in] dt The time since the last tick (s)
 * @return true if the stick is idle, false otherwise
 * @note The legs return to their start positions after idle_return_delay seconds idle
*/
bool GaitController::HandleIdleReturn(double dt) {
    static double idleTime = 0.0;
    static bool hasMovedFromStart = false;
    static bool returnCompleted = false;  // Track if we just finished returning

    // Use a small deadzone to avoid floating point comparison issues
    constexpr double DEADZONE = 0.01;

    // Check if motion intent is idle (mode-aware)
    bool idle = (std::abs(current_motion_intent.forward) < DEADZONE &&
                      std::abs(current_motion_intent.left) < DEADZONE &&
                      std::abs(current_motion_intent.yaw) < DEADZONE);

    // Track if we've ever moved (received non-zero command and started stepping)
    if (!idle && phase_progress > 0.0) {
        hasMovedFromStart = true;
        returnCompleted = false;  // Reset since we're moving again
    }

    if (idle) {
        idleTime += dt;
    } else {
        idleTime = 0.0;
    }

    // Handle idle/return-to-start logic
    // Only trigger if: we've moved, been idle long enough, haven't just completed return, OR already returning
    if (idleReturning) {
        returnToStart(dt);
        // Check if returnToStart finished (it sets idleReturning = false when done)
        if (!idleReturning) {
            returnCompleted = true;
            hasMovedFromStart = false;  // Reset so we don't re-trigger until user moves again
        }
        idleTime = 0.0;
    } else if (hasMovedFromStart && !returnCompleted && idleTime > idle_return_delay) {
        idleReturning = true;
        phase_progress = 0.0;
        leg_swinging.fill(false);
        returnToStart(dt);
        idleTime = 0.0;
    }
    return idle; // Return whether stick is idle
}

/**
 * @brief Foot targets for one walking tick, without changing any state
 * @param[in] forward Forward body velocity (m/s)
 * @param[in] left Leftward body velocity (m/s)
 * @param[in] yaw Body yaw rate (rad/s)
 * @param[in] ds How far to advance the phase this tick (fraction of a phase)
 * @param[in] moveDt How much body motion to apply to the stance feet this tick (s)
 * @return The target foot position of each leg (leg frame)
 * @note Swing feet follow a curve from their lift-off point to their landing target. Stance feet
 *       move back by the body's motion. Both stay within MAX_STRIDE_LENGTH of neutral, which keeps
 *       neighbouring legs from colliding (see test_leg_clearance).
*/
std::array<Vector3d, MAX_LEGS + 1> GaitController::walkTargets(double forward, double left, double yaw, double ds, double moveDt) {
    std::array<Vector3d, MAX_LEGS + 1> targets;
    const double s = std::min(1.0, phase_progress + ds);
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        if (leg_swinging[legNum]) {
            if (s >= 1.0) {
                targets[legNum] = swing_target[legNum];
                continue;
            }
            SwingSpline spline;
            trajectoryGen.genSwingSpline(spline, swing_start[legNum], swing_target[legNum],
                swing_lift[legNum] > 0.0 ? swing_lift[legNum] : trajectoryGen.swingLift);
            targets[legNum] = trajectoryGen.evalBezier(spline.P0, spline.P1, spline.P2, spline.P3, spline.P4, s);
            // A stumble lifts the foot higher, fading out over the last 30% so it still lands on target
            targets[legNum].z() += stumble_boost[legNum] * std::min(1.0, (1.0 - s) / 0.3);
            targets[legNum] = limitSwingLift(legNum, targets[legNum],
                swing_start[legNum].z() + (swing_target[legNum].z() - swing_start[legNum].z()) * s);
        } else {
            const Vector3d& current = current_leg_positions[legNum];
            targets[legNum] = trajectoryGen.stanceTarget(legNum, current,
                trajectoryGen.footDisplacementFrom(forward, left, yaw, legNum, current, moveDt));
            targets[legNum].z() = plantedStart(legNum).z();  // on its own ground
        }
    }
    return targets;
}

/**
 * @brief Handles walking motion of the hexapod
 * @param[in] dt The time since the last tick (s)
 * @note The command is re-read every tick, so changes take effect immediately rather than at the
 *       next phase: stance feet follow the current body velocity, and swing feet re-aim their
 *       landing target (at up to max_target_speed). The phase length also follows the command.
*/
void GaitController::walk(double dt) {
    // Check if velocity is idle
    bool velocityIdle = HandleIdleReturn(dt);
    if (idleReturning) return;

    // Ensure gait config is set
    trajectoryGen.EnsureGaitConfig();

    // Switch gait between phases, when every foot is planted. The per-tick planning carries on from
    // wherever the feet are, and every foot stays in its stride circle, so the change is safe.
    if (phase_progress == 0.0) {
        applyPendingGait();
    }
    adaptToTerrain(dt);

    // Current command, capped at what this gait can do
    double forward = current_motion_intent.forward;
    double left = current_motion_intent.left;
    double yaw = current_motion_intent.yaw;
    double footSpeed = std::hypot(forward, left) + std::abs(yaw) * trajectoryGen.maxBodyRadius();
    double maxSpeed = maxFootSpeed();
    if (footSpeed > maxSpeed) {
        double scale = maxSpeed / footSpeed;
        forward *= scale;
        left *= scale;
        yaw *= scale;
        footSpeed = maxSpeed;
    }

    // Step faster once the stride would exceed MAX_STRIDE_LENGTH
    size_t groups = std::max<size_t>(2, trajectoryGen.gaitState.config.size());
    current_phase_duration = phase_duration;
    if (footSpeed > 1e-6) {
        double strideLimitedDuration = 2.0 * MAX_STRIDE_LENGTH / ((groups - 1) * footSpeed);
        current_phase_duration = std::clamp(strideLimitedDuration, min_phase_duration, phase_duration);
    }
    const double stanceTime = (groups - 1) * current_phase_duration;

    // Start of a phase: the next group lifts off from where it stands
    if (phase_progress == 0.0) {
        const auto& swingGroup = trajectoryGen.gaitState.config[phase];
        stumble_target.fill(0.0);
        stumble_boost.fill(0.0);
        swing_lift.fill(0.0);
        for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
            leg_swinging[legNum] = std::find(swingGroup.begin(), swingGroup.end(), legNum) != swingGroup.end();
            swing_start[legNum] = current_leg_positions[legNum];
            swing_target[legNum] = current_leg_positions[legNum];
            trajectoryGen.gaitState.swingSplines[legNum].active = false;
            trajectoryGen.gaitState.stanceSplines[legNum].active = false;
        }
    }

    // Stumble lift ramps in at 0.5 m/s rather than jerking the foot up
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        stumble_boost[legNum] += std::clamp(stumble_target[legNum] - stumble_boost[legNum], -0.5 * dt, 0.5 * dt);
    }

    // Swing landing targets follow the command, at a limited rate. Both ends stay inside the
    // stride circle, so every point in between does too.
    const double maxTargetStep = max_target_speed * dt;
    Eigen::Isometry2d bodyToMapTransform;
    const bool useMap = terrain_map && terrain_adaptation && forcesValid() && bodyToMap(bodyToMapTransform);
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        if (!leg_swinging[legNum]) continue;
        Vector3d desired = trajectoryGen.swingTarget(legNum,
            trajectoryGen.footDisplacement(forward, left, yaw, legNum, stanceTime / 2.0));
        desired.z() = plantedStart(legNum).z();  // expect the ground where this foot last stood
        if (useMap) {
            planFoothold(legNum, desired, bodyToMapTransform);  // or what the camera saw there
        }
        Vector3d step = desired - swing_target[legNum];
        if (step.norm() > maxTargetStep) {
            step *= maxTargetStep / step.norm();
        }
        swing_target[legNum] += step;
    }

    // Advance, slowing the whole gait if a joint would move faster than max_joint_speed
    double ds = velocityIdle ? 0.0 : dt / current_phase_duration;
    double moveDt = velocityIdle ? 0.0 : dt;
    auto targets = walkTargets(forward, left, yaw, ds, moveDt);
    double scale = jointSpeedScale(targets, dt);
    if (scale < 1.0) {
        ds *= scale;
        moveDt *= scale;
        targets = walkTargets(forward, left, yaw, ds, moveDt);
    }
    sendFootTargets(targets, dt);

    phase_progress = std::min(1.0, phase_progress + ds);
    if (phase_progress >= 1.0) {
        // A swing that ends without the foot touching down: keep reaching down for the ground
        for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
            if (leg_swinging[legNum] && terrain_adaptation) {
                // The foot is where its swing aimed: that's its ground now
                terrain_offset[legNum] = std::clamp(current_leg_positions[legNum].z() - gaitConfig.startPosition.at(legNum).z(),
                    -max_step_height, max_step_height);
            }
            // A foot that has just landed takes a moment to load up (the other legs still carry the
            // weight), and treating that as missing ground ratcheted the feet down a few mm every
            // step: check again once the weight has had time to come onto it
            if (leg_swinging[legNum] && terrain_adaptation && forcesValid() &&
                foot_forces[legNum].z() < contact_force_off) {
                landing_check[legNum] = true;
                unloaded_time[legNum] = 0.0;
            }
        }
        leg_swinging.fill(false);
        phase = (phase + 1) % trajectoryGen.gaitState.config.size();
        phase_progress = 0.0;
    }
}

/**
 * @brief Foot positions for a body pose in tilt mode, and whether the legs can take it up safely
 * @param[in] pose The body pose relative to the planted feet
 * @param[out] targets The foot position of each leg (leg frame)
 * @return true if every foot stays within MAX_STRIDE_LENGTH of its neutral position, which keeps
 *         neighbouring legs apart (see test_leg_clearance), and every joint is within reach
*/
bool GaitController::tiltFeasible(const BodyPose& pose, std::array<Vector3d, MAX_LEGS + 1>& targets) {
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        targets[legNum] = trajectoryGen.posedFoot(legNum, tilt_ground_feet[legNum], pose);
        const Vector3d offset = targets[legNum] - gaitConfig.startPosition.at(legNum);
        if (std::hypot(offset.x(), offset.y()) > MAX_STRIDE_LENGTH) {
            return false;
        }
        if (!reachable(targets[legNum])) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Spread the robot's weight evenly between its planted feet
 * @param[in] dt The time since the last tick (s)
 * @note With stiff legs on hard ground, sub-millimetre differences decide which feet carry the
 *       weight, so a robot can stand on three legs with the others just touching. Each planted
 *       foot carrying more than its share is raised, and each carrying less lowered, at
 *       load_balance_gain per newton. Only with four or more feet down, since three feet carry
 *       whatever the centre of mass puts on them. Each change is stripped of any part that
 *       would raise or tilt the body (a plane fitted through the planted feet), leaving only
 *       the twist that moves weight between feet, so body height and leveling are untouched.
 *       With load_balancing off, the offsets unwind. Only while standing still (tilt mode
 *       included): walking, the offsets are folded into each foot's terrain height instead.
*/
void GaitController::updateLoadBalance(double dt) {
    constexpr double MAX_RATE = 0.02;  // m/s, fastest a foot is raised or lowered
    const double maxStep = MAX_RATE * dt;

    // Only while standing still: walking, terrain adaptation owns the foot heights, and the two
    // fought. Once walking starts the balancing is folded into each foot's ground height.
    const bool standing = std::abs(current_motion_intent.forward) < 1e-3 && std::abs(current_motion_intent.left) < 1e-3 &&
        std::abs(current_motion_intent.yaw) < 1e-3;  // tilt mode counts: its feet stay planted
    if (!standing && terrain_adaptation) {
        for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
            terrain_offset[legNum] = std::clamp(terrain_offset[legNum] + foot_height_offset[legNum],
                -max_step_height, max_step_height);
            foot_height_offset[legNum] = 0.0;
        }
        return;
    }

    std::array<double, MAX_LEGS + 1> change{};
    if (!load_balancing || !standing || sit_state != SitState::STANDING) {
        for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
            change[legNum] = -foot_height_offset[legNum];
        }
    } else {
        std::vector<int> planted;
        double total = 0.0;
        for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
            if (!isSwinging(legNum)) {
                planted.push_back(legNum);
                total += foot_forces[legNum].z();
            }
        }
        if (!has_foot_forces || planted.size() < 4 || total <= 0.0) {
            return;
        }
        const double share = total / planted.size();
        Eigen::VectorXd raise(planted.size());
        Eigen::MatrixXd plane(planted.size(), 3);
        for (size_t i = 0; i < planted.size(); ++i) {
            raise(i) = load_balance_gain * dt * (foot_forces[planted[i]].z() - share);
            const Vector3d foot = trajectoryGen.neutralFootInBody(planted[i]);
            plane.row(i) << 1.0, foot.x(), foot.y();
        }
        // Remove the best-fit plane through the planted feet
        raise -= plane * plane.colPivHouseholderQr().solve(raise);
        for (size_t i = 0; i < planted.size(); ++i) {
            change[planted[i]] = raise(i);
        }
    }
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        foot_height_offset[legNum] = std::clamp(
            foot_height_offset[legNum] + std::clamp(change[legNum], -maxStep, maxStep),
            -load_balance_max, load_balance_max);
    }
}

/**
 * @brief Whether a leg can put its foot at a target
 * @param[in] target The foot position (leg frame)
 * @return true if IK lands the foot within 1 mm of the target with every joint inside its range
 * @note The femur and tibia ranges match femur_lower/upper and tibia_lower/upper in the URDF (from
 *       the CAD meshes); the coxa keeps +-1.4 rad until its range is worked out
*/
bool GaitController::reachable(const Vector3d& target) {
    constexpr double MAX_COXA_ANGLE = 1.4;   // rad
    constexpr double FEMUR_MIN = -1.6;       // rad, raised
    constexpr double FEMUR_MAX = 1.4;        // rad, lowered
    constexpr double TIBIA_MIN = -2.2;       // rad, straightened
    constexpr double TIBIA_MAX = 0.94;       // rad, folded
    constexpr double REACH_TOLERANCE = 1e-3; // m
    const JointAngles angles = ikSolver.solveIK(target);
    return std::abs(angles.coxa) <= MAX_COXA_ANGLE && angles.femur >= FEMUR_MIN && angles.femur <= FEMUR_MAX &&
        angles.tibia >= TIBIA_MIN && angles.tibia <= TIBIA_MAX &&
        (ikSolver.solveFK(angles) - target).norm() <= REACH_TOLERANCE;
}

/**
 * @brief Lower a swinging foot until every joint of its leg is inside its range
 * @param[in] legNum The leg number
 * @param[in] target The swing's foot target (leg frame)
 * @param[in] groundZ Height of the swing's ground line (lift-off to landing) under the target
 * @return target, with its height above groundZ cut back as far as needed, but no lower than groundZ
 * @note A full lift near the inner end of a stride folds the knee into the femur and raises the
 *       femur into the coxa: strafing swings the middle feet straight in and out from the body,
 *       and a stumble adds more lift. Checked with walk leveling applied, as the IK will be.
*/
Vector3d GaitController::limitSwingLift(int legNum, const Vector3d& target, double groundZ) {
    auto fits = [&](const Vector3d& foot) { return reachable(trajectoryGen.posedFoot(legNum, foot, walk_level)); };
    const double lift = target.z() - groundZ;
    if (lift <= 0.0 || fits(target)) {
        return target;
    }
    Vector3d lowered = target;
    double low = 0.0, high = 1.0;  // share of the lift kept
    for (int i = 0; i < 10; ++i) {
        const double mid = 0.5 * (low + high);
        lowered.z() = groundZ + lift * mid;
        (fits(lowered) ? low : high) = mid;
    }
    lowered.z() = groundZ + lift * low;
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Leg %d swing lowered by %.0f mm to stay inside its joint limits", legNum, (1.0 - low) * lift * 1000.0);
    return lowered;
}

/**
 * @brief Update the leveling correction used while walking
 * @param[in] dt The time since the last tick (s)
 * @note The correction builds up the IMU's measured roll and pitch, so the body levels out on a
 *       slope. It rotates the body about the ground point under it, which keeps the body above
 *       the middle of its feet and barely moves them sideways, so they stay in their stride
 *       circles. It changes at most 0.5 rad/s and only as far as every foot can still reach.
 *       With walk_leveling off it unwinds; without fresh IMU data it holds.
*/
void GaitController::updateWalkLeveling(double dt) {
    constexpr double MAX_RATE = 0.5;  // rad/s
    const double maxStep = MAX_RATE * dt;
    const bool imuFresh = has_imu && (this->now() - last_imu_time).seconds() < 0.2;

    double roll = walk_level_roll;
    double pitch = walk_level_pitch;
    if (!walk_leveling || sit_state != SitState::STANDING) {
        roll = 0.0;
        pitch = 0.0;
    } else if (imuFresh) {
        roll -= walk_level_gain * dt * body_roll;
        pitch -= walk_level_gain * dt * body_pitch;
    }
    roll = std::clamp(roll, -walk_max_angle, walk_max_angle);
    pitch = std::clamp(pitch, -walk_max_angle, walk_max_angle);
    roll = walk_level_roll + std::clamp(roll - walk_level_roll, -maxStep, maxStep);
    pitch = walk_level_pitch + std::clamp(pitch - walk_level_pitch, -maxStep, maxStep);

    // Scale back towards level until every foot is reachable (level always is)
    const Vector3d pivot(0.0, 0.0, gaitConfig.currentHeight);
    auto fits = [&](double scale) {
        const BodyPose pose = TrajectoryGenerator::rotationAbout(roll * scale, pitch * scale, pivot);
        for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
            if (!reachable(trajectoryGen.posedFoot(legNum, current_leg_positions[legNum], pose))) {
                return false;
            }
        }
        return true;
    };
    if (!fits(1.0)) {
        double low = 0.0, high = 1.0;
        for (int i = 0; i < 10; ++i) {
            const double mid = 0.5 * (low + high);
            (fits(mid) ? low : high) = mid;
        }
        roll *= low;
        pitch *= low;
    }
    walk_level_roll = roll;
    walk_level_pitch = pitch;
    walk_level = TrajectoryGenerator::rotationAbout(roll, pitch, pivot);
}

/**
 * @brief Tilt mode: hold the feet where they are and move the body to the pose from body_pose
 * @param[in] dt The time since the last tick (s)
 * @param[in] exiting true once another mode is selected, to ease the body back to neutral
 * @note On entry the legs step back to their start positions first. The requested pose is rate
 *       limited, then the IMU leveling correction is added: it builds up the difference between
 *       the requested angles and the body's measured angles, so the body holds its angles in space
 *       when the ground beneath it tilts or turns. Translation can't be held this way, since the
 *       IMU can't measure position, so it is relative to the feet. A pose the legs can't reach is
 *       cut back to the furthest safe pose towards it.
*/
void GaitController::tilt(double dt, bool exiting) {
    current_motion_intent = MotionIntent{};

    if (!tilt_active) {
        // Plant every foot at its start position, so the body has the full range to move in
        if (!idleReturning && !feetAtStart()) {
            idleReturning = true;
            phase_progress = 0.0;
            leg_swinging.fill(false);
        }
        if (idleReturning) {
            returnToStart(dt);
            return;
        }
        // Take over from walk leveling: the feet as actually placed become tilt mode's neutral
        tilt_saved_roll = walk_level_roll;
        tilt_saved_pitch = walk_level_pitch;
        tilt_saved_feet = current_leg_positions;
        for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
            tilt_ground_feet[legNum] = trajectoryGen.posedFoot(legNum, current_leg_positions[legNum], walk_level);
        }
        current_leg_positions = tilt_ground_feet;
        walk_level = BodyPose{};
        body_pose_request = BodyPose{};
        tilt_command = BodyPose{};
        tilt_correction = BodyPose{};
        tilt_applied = BodyPose{};
        tilt_yaw_reference = body_yaw;
        tilt_active = true;
        RCLCPP_INFO(this->get_logger(), "Tilt mode: feet planted, body follows body_pose%s",
            tilt_leveling ? ", held in place by the IMU" : "");
    }

    // Move the command towards the request (or neutral when leaving) at a limited rate
    auto approach = [](double& value, double goal, double maxStep) {
        value += std::clamp(goal - value, -maxStep, maxStep);
    };
    const BodyPose& goal = exiting ? BodyPose{} : body_pose_request;
    const double angleStep = tilt_angle_rate * dt;
    approach(tilt_command.roll, goal.roll, angleStep);
    approach(tilt_command.pitch, goal.pitch, angleStep);
    approach(tilt_command.yaw, goal.yaw, angleStep);
    for (int i = 0; i < 3; ++i) {
        approach(tilt_command.translation[i], goal.translation[i], tilt_translation_rate * dt);
    }

    // Leveling: integrate the error between the commanded and measured angles. Without fresh IMU
    // data the correction holds, and when leaving or with leveling off it unwinds.
    const BodyPose previousCorrection = tilt_correction;
    const bool imuFresh = has_imu && (this->now() - last_imu_time).seconds() < 0.2;
    if (exiting || !tilt_leveling) {
        approach(tilt_correction.roll, 0.0, angleStep);
        approach(tilt_correction.pitch, 0.0, angleStep);
        approach(tilt_correction.yaw, 0.0, angleStep);
    } else if (imuFresh) {
        const double gain = tilt_level_gain * dt;
        tilt_correction.roll += gain * (tilt_command.roll - body_roll);
        tilt_correction.pitch += gain * (tilt_command.pitch - body_pitch);
        if (tilt_lock_yaw) {
            tilt_correction.yaw += gain * std::remainder(tilt_command.yaw - (body_yaw - tilt_yaw_reference), 2.0 * M_PI);
        }
    }
    // The command plus correction stays within tilt_max_angle
    tilt_correction.roll = std::clamp(tilt_correction.roll, -tilt_max_angle - tilt_command.roll, tilt_max_angle - tilt_command.roll);
    tilt_correction.pitch = std::clamp(tilt_correction.pitch, -tilt_max_angle - tilt_command.pitch, tilt_max_angle - tilt_command.pitch);
    tilt_correction.yaw = std::clamp(tilt_correction.yaw, -tilt_max_angle - tilt_command.yaw, tilt_max_angle - tilt_command.yaw);

    BodyPose desired;
    desired.roll = tilt_command.roll + tilt_correction.roll;
    desired.pitch = tilt_command.pitch + tilt_correction.pitch;
    desired.yaw = tilt_command.yaw + tilt_correction.yaw;
    desired.translation = tilt_command.translation;

    // Step from the last pose (always safe) towards the desired one, never jumping: a pose change
    // of more than a small step per tick snaps the feet across the ground, which spins the robot
    constexpr double MAX_POSE_ANGLE_SPEED = 2.0;   // rad/s
    constexpr double MAX_POSE_SHIFT_SPEED = 0.2;   // m/s
    const double maxAngle = MAX_POSE_ANGLE_SPEED * dt;
    const double maxShift = MAX_POSE_SHIFT_SPEED * dt;
    BodyPose step = tilt_applied;
    approach(step.roll, desired.roll, maxAngle);
    approach(step.pitch, desired.pitch, maxAngle);
    approach(step.yaw, desired.yaw, maxAngle);
    for (int i = 0; i < 3; ++i) {
        approach(step.translation[i], desired.translation[i], maxShift);
    }

    std::array<Vector3d, MAX_LEGS + 1> targets;
    BodyPose pose = step;
    if (!tiltFeasible(pose, targets)) {
        // Out of reach: ease yaw back towards zero instead, since turning swings the feet furthest
        // and keeping the body level matters more. Easing back also stops yaw pinning the pose.
        BodyPose yawEased = step;
        yawEased.yaw = tilt_applied.yaw;
        approach(yawEased.yaw, 0.0, maxAngle);
        if (tiltFeasible(yawEased, targets)) {
            pose = yawEased;
            tilt_correction.yaw = previousCorrection.yaw;  // stop it winding up against the limit
        } else {
            // Go as far along the step as is safe, possibly nowhere
            double low = 0.0, high = 1.0;
            for (int i = 0; i < 10; ++i) {
                const double mid = 0.5 * (low + high);
                BodyPose p;
                p.roll = tilt_applied.roll + mid * (step.roll - tilt_applied.roll);
                p.pitch = tilt_applied.pitch + mid * (step.pitch - tilt_applied.pitch);
                p.yaw = tilt_applied.yaw + mid * (step.yaw - tilt_applied.yaw);
                p.translation = tilt_applied.translation + mid * (step.translation - tilt_applied.translation);
                (tiltFeasible(p, targets) ? low : high) = mid;
            }
            pose.roll = tilt_applied.roll + low * (step.roll - tilt_applied.roll);
            pose.pitch = tilt_applied.pitch + low * (step.pitch - tilt_applied.pitch);
            pose.yaw = tilt_applied.yaw + low * (step.yaw - tilt_applied.yaw);
            pose.translation = tilt_applied.translation + low * (step.translation - tilt_applied.translation);
            tilt_correction = previousCorrection;
        }
        tiltFeasible(pose, targets);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Tilt pose limited by the legs' reach");
    }
    tilt_applied = pose;
    sendFootTargets(targets, dt);

    // Back at neutral after leaving tilt mode: hand over to walking
    const bool neutral = std::abs(pose.roll) < 1e-4 && std::abs(pose.pitch) < 1e-4 &&
        std::abs(pose.yaw) < 1e-4 && pose.translation.norm() < 1e-4;
    if (exiting && neutral) {
        tilt_active = false;
        tilt_applied = BodyPose{};
        // Hand back to walk leveling where it left off; the feet are where it had them
        walk_level_roll = tilt_saved_roll;
        walk_level_pitch = tilt_saved_pitch;
        walk_level = TrajectoryGenerator::rotationAbout(walk_level_roll, walk_level_pitch,
            Vector3d(0.0, 0.0, gaitConfig.currentHeight));
        current_leg_positions = tilt_saved_feet;
        phase = 0;
        phase_progress = 0.0;
        RCLCPP_INFO(this->get_logger(), "Tilt mode finished, body back to neutral");
    }
}

/**
 * @brief Add a depth camera frame to the height map
 * @param[in] msg The point cloud
 * @note Only once the startup sequence is complete, and at most map_update_rate times a second.
 *       Uses every depth_stride-th row and column. Points are moved into map_frame with TF; their
 *       height is relative to the ground under the robot at startup (TF height plus ground_level).
 *       Cells further than map_radius from the robot are dropped.
*/
void GaitController::depthCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Not until the robot is standing: before that TF's body height is wrong (in simulation the
    // robot drops in from 0.5 m), and anything seen then would be mapped far too low
    if (!terrain_map || !positions_initialized) {
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
        for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
            feetInMap.push_back(bodyToMapNow * trajectoryGen.legToBody(legNum, measured_leg_positions[legNum]).head<2>());
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
            p.z() += ground_level;
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
    if (!terrain_map || terrain_map_pub->get_subscription_count() == 0) {
        return;
    }
    const auto cells = heightMap.cellCentres();
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = this->now();
    cloud.header.frame_id = map_frame;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(cells.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (const auto& c : cells) {
        *x = c.x(); *y = c.y(); *z = c.z() - ground_level;
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
 * @brief Use the height map to place a swinging foot's landing spot and set its lift
 * @param[in] legNum The leg number
 * @param[in,out] target The landing target (leg frame); its height comes from the map, and it
 *                moves up to foothold_search to keep foothold_edge of flat ground around it
 * @param[in] bodyToMapTransform Body (x, y) to map (x, y)
 * @note Ground the camera hasn't seen leaves the target alone (force sensing still finds it), and
 *       so does a mapped height within map_height_deadband of the foot's own ground.
 *       The lift clears the highest seen ground between lift-off and landing by step_clearance.
*/
void GaitController::planFoothold(int legNum, Vector3d& target, const Eigen::Isometry2d& bodyToMapTransform) {
    const Vector3d nominal = gaitConfig.startPosition.at(legNum);
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
        const double mapped = std::clamp(*ground - ground_level, -max_step_height, max_step_height);
        if (std::abs(nominal.z() + mapped - target.z()) > map_height_deadband) {
            target.z() = nominal.z() + mapped;
        }
    }

    // Lift enough to clear the highest ground on the way (the foot peaks at ~0.88x the lift)
    const Vector2d mapStart = bodyToMapTransform * trajectoryGen.legToBody(legNum, swing_start[legNum]).head<2>();
    if (auto highest = heightMap.maxAlong(mapStart, mapTarget, 0.02)) {
        const double ends = std::max(swing_start[legNum].z(), target.z()) - nominal.z();
        const double needed = (*highest - ground_level) - ends + step_clearance;
        const double wanted = std::clamp(needed / 0.88, trajectoryGen.swingLift, 0.12);
        // Changes during a swing reshape it: limit them to 0.2 m/s so the foot doesn't jerk
        const double current = swing_lift[legNum] > 0.0 ? swing_lift[legNum] : trajectoryGen.swingLift;
        const double maxChange = 0.2 * nominal_dt;
        swing_lift[legNum] = current + std::clamp(wanted - current, -maxChange, maxChange);
    }
}
