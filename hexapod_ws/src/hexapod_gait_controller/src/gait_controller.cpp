#include "hexapod_gait_controller/gait_controller.hpp"

int main(int argc, char **argv){    
    rclcpp::init(argc, argv);    
    auto node = std::make_shared<GaitController>();    
    rclcpp::spin(node);    
    rclcpp::shutdown();    
    return 0;
}

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

void GaitController::jointStateCallback(const JointState::SharedPtr msg) {
    latest_joint_states = *msg;
    
    // Compute current leg positions from joint angles using FK
    if (msg->position.size() >= 18) {
        for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
            int base_idx = (legNum - 1) * 3;
            
            hexapod_gait_controller::JointAngles angles{
                msg->position[base_idx + 0],  // coxa
                msg->position[base_idx + 1],  // femur
                msg->position[base_idx + 2]   // tibia
            };
            
            //current_leg_positions[legNum] = ikSolver.solveFK(angles);
        }
        has_joint_states = true;
    }
}

/*
@brief Service callback to change gait
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

/* 
@brief Service callback to change mode
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

/*
@brief Service callback to change height
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


/*
@brief Service callback to report supported gaits and modes
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

    response->max_linear_vel = gaitConfig.max_velocity;
    response->max_angular_vel = gaitConfig.max_angular_velocity;
}

/*
@brief Main gait loop - called by timer at fixed rate
@note This replaces your embedded while(true) loop
*/
void GaitController::gaitTimerCallback() {
    // Run startup sequence once on first timer callback
    if (!positions_initialized) {
        startup();
        
        // Check if startup is complete by checking if we've progressed through all steps
        int max_startup_size = *std::max_element(startup_sizes.begin() + 1, startup_sizes.end());
        if (startup_step >= max_startup_size) {
            positions_initialized = true;
        }
        return;  // Continue startup in next callback
    }

    switch (gaitConfig.currentMode) {
        case Mode::MODE_STRAFE:
            current_motion_intent.forward = filtered_cmd_vel.linear.x;
            current_motion_intent.lateral = filtered_cmd_vel.linear.y;
            current_motion_intent.yaw = filtered_cmd_vel.angular.z;
            break;
        case Mode::MODE_NORMAL:
            current_motion_intent.forward = filtered_cmd_vel.linear.x;
            current_motion_intent.lateral = 0.0;
            current_motion_intent.yaw = filtered_cmd_vel.linear.y;
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
    walk();
}

/*
@brief Move all legs to the home position and deactivate servos
*/
void GaitController::home() {
    auto joint_state_msg = JointState();
    joint_state_msg.header.stamp = this->now();

    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        // Compute IK to get joint angles
        auto angles = ikSolver.solveIK(gaitConfig.homePos);
        
        // Add joint names and positions
        joint_state_msg.name.push_back("leg" + std::to_string(legNum) + "_coxa_joint");
        joint_state_msg.position.push_back(angles.coxa);
        
        joint_state_msg.name.push_back("leg" + std::to_string(legNum) + "_femur_joint");
        joint_state_msg.position.push_back(angles.femur);
        
        joint_state_msg.name.push_back("leg" + std::to_string(legNum) + "_tibia_joint");
        joint_state_msg.position.push_back(angles.tibia);

        current_leg_positions[legNum] = gaitConfig.homePos;
    }
    // Publish joint states
    joint_cmd_pub->publish(joint_state_msg);
    rclcpp::sleep_for(std::chrono::milliseconds(500));  // Wait for legs to reach home position
    RCLCPP_INFO(this->get_logger(), "Moved to home position");
}

/*
@brief Startup sequence to move legs to start position
*/
void GaitController::startup() {
    auto joint_state_msg = JointState();
    joint_state_msg.header.stamp = this->now();
    static bool initialized = false;

    if(!initialized){
        home();
        initialized = true;
        
        // Pre-generate all trajectories for the startup sequence
        for (int leg = 1; leg <= MAX_LEGS; ++leg) {
            trajectoryGen.GenStraightTrajectory(
                startup_trajectory[leg].data(), 
                startup_sizes[leg], 
                gaitConfig.homePos, 
                gaitConfig.startPosition.at(leg), 
                MAX_RESOLUTION-1
            );
        }
        startup_step = 0;
        return;  // Exit first call and wait for next timer callback
    }

    // Reset phase to 0 when starting up
    phase = 0;

    // Update start positions based on current height
    gaitConfig.setHeight(gaitConfig.currentHeight);

    // Execute one step of the startup sequence per timer callback
    for (int leg = 1; leg <= MAX_LEGS; ++leg) {
        if (startup_step < startup_sizes[leg]) {
            Vector3d pos = startup_trajectory[leg][startup_step];

            // Compute IK to get joint angles
            auto angles = ikSolver.solveIK(pos);
            
            // Add joint names and positions
            joint_state_msg.name.push_back("leg" + std::to_string(leg) + "_coxa_joint");
            joint_state_msg.position.push_back(angles.coxa);
            
            joint_state_msg.name.push_back("leg" + std::to_string(leg) + "_femur_joint");
            joint_state_msg.position.push_back(angles.femur);
            
            joint_state_msg.name.push_back("leg" + std::to_string(leg) + "_tibia_joint");
            joint_state_msg.position.push_back(angles.tibia);

            current_leg_positions[leg] = pos;
        }
    }
    joint_cmd_pub->publish(joint_state_msg);
    startup_step++;
    
    // Check if startup sequence is complete
    int max_startup_size = *std::max_element(startup_sizes.begin() + 1, startup_sizes.end());
    if (startup_step >= max_startup_size) {
        RCLCPP_INFO(this->get_logger(), "Startup sequence complete");
    }
}

/*
@brief Check if cmd_vel has changed significantly enough to warrant mid-trajectory regeneration
@return true if trajectory should be regenerated
*/
bool GaitController::shouldRegenerateTrajectory() {
    double df = std::abs(current_motion_intent.forward - trajectory_motion_intent.forward);
    double dl = std::abs(current_motion_intent.lateral - trajectory_motion_intent.lateral);
    double dy = std::abs(current_motion_intent.yaw - trajectory_motion_intent.yaw);

    return (df > CMD_VEL_CHANGE_THRESHOLD ||
            dl > CMD_VEL_CHANGE_THRESHOLD ||
            dy > CMD_VEL_CHANGE_THRESHOLD);
}

void GaitController::retargetSwingSplines() {
    bool anySwingActive = false;
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        if (trajectoryGen.gaitState.swingSplines[legNum].active) {
            anySwingActive = true;
            break;
        }
    }
    if (!anySwingActive) return;

    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        auto& spline = trajectoryGen.gaitState.swingSplines[legNum];
        if (!spline.active) continue;

        // Only retarget mid-swing (expanded window for smoother transitions)
        if (spline.s < 0.15 || spline.s > 0.85) continue;

        Vector3d currentPos = current_leg_positions[legNum];

        // Compute new target using current cmd_vel
        Vector3d forwardPos = trajectoryGen.linearTarget(
            current_motion_intent.forward,
            current_motion_intent.lateral,
            currentPos,
            legNum,
            false
        );

        Vector3d rotationPos = trajectoryGen.rotationalTarget(
            current_motion_intent.yaw,
            currentPos,
            legNum,
            false
        );

        Vector3d idealTarget = trajectoryGen.BlendTargetPosition(
            currentPos,
            forwardPos,
            rotationPos,
            legNum
        );

        // Update spline endpoint
        spline.P3 = idealTarget;

        Vector3d dir = spline.P3 - currentPos;
        if (dir.norm() < 1e-6) continue;

        Vector3d dirNorm = dir.normalized();
        double offsetScale = dir.norm() * 0.25;

        spline.P2 = spline.P3 + dirNorm * offsetScale;
        spline.P2.z() = spline.P3.z() + liftHeight;
    }
}

void GaitController::retargetStanceTrajectories() {
    bool anyStanceActive = false;
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        if (trajectoryGen.gaitState.stanceSplines[legNum].active) {
            anyStanceActive = true;
            break;
        }
    }
    if (!anyStanceActive) return;

    double strideMultiplier = trajectoryGen.CalculateStrideMultiplier();

    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        auto& spline = trajectoryGen.gaitState.stanceSplines[legNum];
        if (!spline.active) continue;

        Vector3d currentPos = current_leg_positions[legNum];

        Vector3d forwardPos = trajectoryGen.linearTarget(
            current_motion_intent.forward,
            current_motion_intent.lateral,
            currentPos,
            legNum,
            true
        );

        Vector3d rotationPos = trajectoryGen.rotationalTarget(
            current_motion_intent.yaw,
            currentPos,
            legNum,
            true
        );

        Vector3d idealTarget = trajectoryGen.BlendTargetPosition(
            currentPos,
            forwardPos,
            rotationPos,
            legNum,
            strideMultiplier
        );

        spline.P0 = currentPos;
        spline.P1 = idealTarget;
        spline.s = 0.0;
    }
}

/*
@brief Perform a leg step based on the current trajectories
@param idle Whether the joystick is idle
@param resolution The resolution of the trajectories
@param handlePhaseTransition Whether to handle phase transitions
@note If idle is true, the step will not advance
@note If handlePhaseTransition is false, phase transitions will be skipped and must be handled externally
*/
void GaitController::PerformLegStep(bool idle, int resolution, bool handlePhaseTransition) {
    auto joint_state_msg = JointState();
    joint_state_msg.header.stamp = this->now();
    
    // Prepare joint names (18 joints total: 6 legs × 3 joints)
    joint_state_msg.name.reserve(18);
    joint_state_msg.position.reserve(18);
    
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        // Get target position for this leg
        // Each leg is assigned EITHER swing OR stance trajectory for this phase (not both)
        Eigen::Vector3d targetPos = current_leg_positions[legNum];
        SwingSpline& spline = trajectoryGen.gaitState.swingSplines[legNum];
        LineSpline& stanceSpline = trajectoryGen.gaitState.stanceSplines[legNum];

        if (spline.active) {
            targetPos = trajectoryGen.evalBezier(
                spline.P0,
                spline.P1,
                spline.P2,
                spline.P3,
                spline.P4,
                spline.s
            );

            // Advance spline progress
            double ds = idle ? 0.0 : (1.0 / MAX_RESOLUTION);
            spline.s += ds;
            if (spline.s >= 1.0) {
                spline.s = 1.0;
                // Snap to exact endpoint and deactivate to prevent drift
                targetPos = spline.P4;
                spline.active = false;
            }
        }
        else if (stanceSpline.active) {
            targetPos = stanceSpline.P0 + (stanceSpline.P1 - stanceSpline.P0) * stanceSpline.s;

            double ds = idle ? 0.0 : (1.0 / resolution);
            stanceSpline.s += ds;
            if (stanceSpline.s >= 1.0) {
                stanceSpline.s = 1.0;
                targetPos = stanceSpline.P1;
                stanceSpline.active = false;
            }
        }
        
        Eigen::Vector3d delta = targetPos - current_leg_positions[legNum];
        double maxStep = 0.03; // 3 cm per control cycle

            if (delta.norm() > maxStep) {
                targetPos = current_leg_positions[legNum] + delta.normalized() * maxStep;
        }

        // Compute IK to get joint angles
        auto angles = ikSolver.solveIK(targetPos);
        
        // Add joint names and positions
        joint_state_msg.name.push_back("leg" + std::to_string(legNum) + "_coxa_joint");
        joint_state_msg.position.push_back(angles.coxa);
        
        joint_state_msg.name.push_back("leg" + std::to_string(legNum) + "_femur_joint");
        joint_state_msg.position.push_back(angles.femur);
        
        joint_state_msg.name.push_back("leg" + std::to_string(legNum) + "_tibia_joint");
        joint_state_msg.position.push_back(angles.tibia);

        current_leg_positions[legNum] = targetPos;
    }
    
    // Publish joint states
    joint_cmd_pub->publish(joint_state_msg);
    
    // Advance step if not idle
    if (!idle) step++;
    if (!handlePhaseTransition) return;

    // Phase transition
    if (step > resolution) {
        phase = (phase + 1) % trajectoryGen.gaitState.config.size();
        step = 0;
    }    
}

/*
@brief Returns the hexapod to its start position
*/
void GaitController::returnToStart() {
    static int counter = 0;

    // Safety check: phase must be valid
    if (phase >= trajectoryGen.gaitState.config.size()) {
        idleReturning = false;
        counter = 0;
        step = 0;
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
        step = 0;
        phase = 0;
        return;
    }

    if (step == 0) {
        trajectoryGen.GenerateTrajectories(
            liftHeight,
            MAX_RESOLUTION-1,
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
    PerformLegStep(false, MAX_RESOLUTION-1, false);

    // Phase transition
    if (step > MAX_RESOLUTION-1) {
        counter++;
        step = 0;
        phase = (phase + 1) % trajectoryGen.gaitState.config.size();

        // After all phases, finish return-to-start and handle gait change if requested
        if (counter >= static_cast<int>(trajectoryGen.gaitState.config.size())) {
            counter = 0;
            idleReturning = false;

            if (gaitConfig.gaitChangeRequested) {
                gaitConfig.currentGait = gaitConfig.pendingGait;
                trajectoryGen.gaitState.config = gaitConfig.getGaitConfig(gaitConfig.currentGait);
                phase = 0;
                step = 0;
                gaitConfig.gaitChangeRequested = false;
            }
        }
    }
}

/*
@brief Handle idle/return-to-start logic
@return true if the stick is idle, false otherwise
*/
bool GaitController::HandleIdleReturn() {
    static int idleCount = 0;
    static const int idleThreshold = 100;
    static bool hasMovedFromStart = false;
    static bool returnCompleted = false;  // Track if we just finished returning

    // Use a small deadzone to avoid floating point comparison issues
    constexpr double DEADZONE = 0.01;

    // Check if motion intent is idle (mode-aware)
    bool idle = (std::abs(current_motion_intent.forward) < DEADZONE &&
                      std::abs(current_motion_intent.lateral) < DEADZONE &&
                      std::abs(current_motion_intent.yaw) < DEADZONE);

    // Track if we've ever moved (received non-zero command and started stepping)
    if (!idle && step > 0) {
        hasMovedFromStart = true;
        returnCompleted = false;  // Reset since we're moving again
    }

    if (idle) {
        idleCount++;
    } else {
        idleCount = 0;
    }

    // Handle idle/return-to-start logic
    // Only trigger if: we've moved, been idle long enough, haven't just completed return, OR already returning
    if (idleReturning) {
        returnToStart();
        // Check if returnToStart finished (it sets idleReturning = false when done)
        if (!idleReturning) {
            returnCompleted = true;
            hasMovedFromStart = false;  // Reset so we don't re-trigger until user moves again
        }
        idleCount = 0;
    } else if (hasMovedFromStart && !returnCompleted && idleCount > idleThreshold) {
        idleReturning = true;
        step = 0;
        returnToStart();
        idleCount = 0;
    }
    return idle; // Return whether stick is idle
}

/*
@brief Handles walking motion of the hexapod
@note The direction of movement is determined by the current_motion_intent, 
@note which is updated from cmd_vel in the timer callback based on the current mode. 
@note This allows for dynamic switching between different control schemes (e.g. strafing vs normal) while maintaining a consistent gait generation logic.
*/
void GaitController::walk() {
    // Check if velocity is idle
    bool velocityIdle = HandleIdleReturn();
    if (idleReturning) return;

    // Ensure gait config is set
    trajectoryGen.EnsureGaitConfig();

    if (shouldRegenerateTrajectory()) {
        retargetSwingSplines();
        retargetStanceTrajectories();
        trajectory_motion_intent = current_motion_intent;
    }

    // Calculate stride multiplier safely
    double strideMultiplier = trajectoryGen.CalculateStrideMultiplier();

    // Generate trajectories at the start of each phase
    if (step == 0) {
        trajectory_motion_intent = current_motion_intent;
        trajectoryGen.GenerateTrajectories(
            liftHeight,
            MAX_RESOLUTION-1,
            // Swing target - use Twist-based direction
            [this](int legNum, const Vector3d& currentPos) {
                Vector3d forwardPos = trajectoryGen.linearTarget(current_motion_intent.forward, current_motion_intent.lateral, currentPos, legNum, false);
                Vector3d rotationPos = trajectoryGen.rotationalTarget(current_motion_intent.yaw, currentPos, legNum, false);
                Vector3d targetPos = trajectoryGen.BlendTargetPosition(currentPos, forwardPos, rotationPos, legNum);
                return targetPos;
            },
            // Stance target - use Twist-based direction (inverted)
            [this, strideMultiplier](int legNum, const Vector3d& currentPos) {
                Vector3d forwardPos = trajectoryGen.linearTarget(current_motion_intent.forward, current_motion_intent.lateral, currentPos, legNum, true);
                Vector3d rotationPos = trajectoryGen.rotationalTarget(current_motion_intent.yaw, currentPos, legNum, true);
                Vector3d targetPos = trajectoryGen.BlendTargetPosition(currentPos, forwardPos, rotationPos, legNum, strideMultiplier);
                return targetPos;
            },
            current_leg_positions,
            phase
        );
    }
    // Move all legs for this step
    PerformLegStep(velocityIdle, MAX_RESOLUTION-1);
}