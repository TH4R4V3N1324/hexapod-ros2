#include "hexapod_gait_controller/gait_controller.hpp"

#include <unordered_map>

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
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        positions[msg->name[i]] = msg->position[i];
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

    if (has_joint_states && positions_initialized) {
        updateLegOdometry(rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()));
    }
}

/**
 * @brief Estimate the body's motion from the planted feet and add it to the leg odometry
 * @param[in] stamp The joint state timestamp
 * @note Feet that are not swinging are treated as planted. Called for every joint state message.
*/
void GaitController::updateLegOdometry(const rclcpp::Time& stamp) {
    std::array<Eigen::Vector2d, MAX_LEGS + 1> feet;
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        feet[legNum] = trajectoryGen.legToBody(legNum, measured_leg_positions[legNum]).head<2>();
    }

    if (has_previous_feet && stamp > previous_feet_stamp) {
        std::vector<Eigen::Vector2d> before, after;
        for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
            if (!isSwinging(legNum)) {
                before.push_back(previous_feet[legNum]);
                after.push_back(feet[legNum]);
            }
        }
        BodyMotion motion;
        if (LegOdometry::estimateMotion(before, after, motion)) {
            legOdometry.integrate(motion);
        }
    }
    previous_feet = feet;
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
    t.transform.translation.z = -gaitConfig.currentHeight;  // feet are at currentHeight below the hips
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
 * @note Yaw is ignored: heading comes from odometry. Messages without orientation are skipped.
*/
void GaitController::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (msg->orientation_covariance[0] < 0.0) {
        return;
    }
    const auto& o = msg->orientation;
    body_roll = std::atan2(2.0 * (o.w * o.x + o.y * o.z), 1.0 - 2.0 * (o.x * o.x + o.y * o.y));
    body_pitch = std::asin(std::clamp(2.0 * (o.w * o.y - o.z * o.x), -1.0, 1.0));
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
    response->message = "Gait changed to " + std::to_string(request->gait);
    RCLCPP_INFO(this->get_logger(), "Gait changed to %d", request->gait);
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
    double height = static_cast<double>(request->height);
    gaitConfig.setHeight(height);
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Height changed to %.2f", height);
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
        case Mode::MODE_CONFIG:
            // TODO: Configuration mode handling
            break;
        case Mode::MODE_TILT:
            // TODO: BodyTilt();
            break;
        case Mode::NUM_MODES:
        default:
            RCLCPP_WARN(this->get_logger(), "Invalid or unimplemented mode");
            break;
    }
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
    // Compute IK to get joint angles
    auto angles = ikSolver.solveIK(target);
    
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
                : trajectoryGen.evalBezier(spline.P0, spline.P1, spline.P2, spline.P3, spline.P4, s);
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

    // Check if all legs are already at start position (within tolerance)
    constexpr double POSITION_TOLERANCE = 0.005;  // 5mm tolerance
    bool allAtStart = true;
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        auto it = gaitConfig.startPosition.find(legNum);
        if (it != gaitConfig.startPosition.end()) {
            Vector3d delta = current_leg_positions[legNum] - it->second;
            if (delta.norm() > POSITION_TOLERANCE) {
                allAtStart = false;
                break;
            }
        }
    }

    if (allAtStart) {
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
                auto it = gaitConfig.startPosition.find(legNum);
                return (it != gaitConfig.startPosition.end()) ? it->second : current_leg_positions[legNum];
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

            if (gaitConfig.gaitChangeRequested) {
                gaitConfig.currentGait = gaitConfig.pendingGait;
                trajectoryGen.gaitState.config = gaitConfig.getGaitConfig(gaitConfig.currentGait);
                phase = 0;
                phase_progress = 0.0;
                gaitConfig.gaitChangeRequested = false;
            }
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
            trajectoryGen.genSwingSpline(spline, swing_start[legNum], swing_target[legNum]);
            targets[legNum] = trajectoryGen.evalBezier(spline.P0, spline.P1, spline.P2, spline.P3, spline.P4, s);
        } else {
            const Vector3d& current = current_leg_positions[legNum];
            targets[legNum] = trajectoryGen.stanceTarget(legNum, current,
                trajectoryGen.footDisplacementFrom(forward, left, yaw, legNum, current, moveDt));
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
        for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
            leg_swinging[legNum] = std::find(swingGroup.begin(), swingGroup.end(), legNum) != swingGroup.end();
            swing_start[legNum] = current_leg_positions[legNum];
            swing_target[legNum] = current_leg_positions[legNum];
            trajectoryGen.gaitState.swingSplines[legNum].active = false;
            trajectoryGen.gaitState.stanceSplines[legNum].active = false;
        }
    }

    // Swing landing targets follow the command, at a limited rate. Both ends stay inside the
    // stride circle, so every point in between does too.
    const double maxTargetStep = max_target_speed * dt;
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        if (!leg_swinging[legNum]) continue;
        Vector3d desired = trajectoryGen.swingTarget(legNum,
            trajectoryGen.footDisplacement(forward, left, yaw, legNum, stanceTime / 2.0));
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
        leg_swinging.fill(false);
        phase = (phase + 1) % trajectoryGen.gaitState.config.size();
        phase_progress = 0.0;
    }
}
