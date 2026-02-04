#include "rclcpp/rclcpp.hpp"
#include "hexapod_gait_controller/trajectory_generator.hpp"
#include "hexapod_gait_controller/kinematic_solver.hpp"
#include "hexapod_gait_controller/gait_config.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <cmath>

using namespace hexapod_gait_controller;

using geometry_msgs::msg::Twist;
using sensor_msgs::msg::JointState;
using Eigen::Vector3d;
using std::placeholders::_1;

class GaitController : public rclcpp::Node { 
public: 
    GaitController() : Node("gait_controller") {
        cmd_vel_sub = this->create_subscription<Twist>("cmd_vel", 10, std::bind(&GaitController::cmdVelCallback, this, _1));
        joint_cmd_pub = this->create_publisher<JointState>("joint_commands", 10);
        joint_state_sub = this->create_subscription<JointState>("joint_states", 10, std::bind(&GaitController::jointStateCallback, this, _1));

        double loop_rate_hz = 50.0;
        gait_timer = this->create_wall_timer(std::chrono::duration<double>(1.0 / loop_rate_hz), std::bind(&GaitController::gaitTimerCallback, this));

        filtered_cmd_vel = Twist();

        RCLCPP_INFO(this->get_logger(), "GaitController node started at %.1f Hz", loop_rate_hz);
    }    
private:
    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Publisher<JointState>::SharedPtr joint_cmd_pub;
    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub;
    rclcpp::TimerBase::SharedPtr gait_timer;
    
    Twist last_cmd_vel;
    Twist filtered_cmd_vel;
    Twist trajectory_cmd_vel;  // cmd_vel used when trajectory was generated
    TrajectoryGenerator trajectoryGen;
    GaitConfig gaitConfig;
    hexapod_gait_controller::KinematicSolver ikSolver;
    uint8_t step = 0;
    uint8_t phase = 0;
    bool idleReturning = false;
    
    // Joint state feedback
    JointState latest_joint_states;
    std::array<Vector3d, MAX_LEGS + 1> current_leg_positions;  // Computed from FK
    bool has_joint_states = false;
    bool positions_initialized = false;
    
    // Threshold for mid-trajectory regeneration
    static constexpr double CMD_VEL_CHANGE_THRESHOLD = 0.05;
    
public:
    void cmdVelCallback(const Twist::SharedPtr msg);
    void jointStateCallback(const JointState::SharedPtr msg);
    void gaitTimerCallback();
    void home();
    void startup();
    bool shouldRegenerateTrajectory();
    void retargetSwingSplines();
    void PerformLegStep(bool idle, int resolution, bool handlePhaseTransition = true);
    void returnToStart();
    bool HandleIdleReturn();
    void Strafe();
    void Normal();
};

int main(int argc, char **argv){    
    rclcpp::init(argc, argv);    
    auto node = std::make_shared<GaitController>();    
    rclcpp::spin(node);    
    rclcpp::shutdown();    
    return 0;
}

void GaitController::cmdVelCallback(const Twist::SharedPtr msg) {
    last_cmd_vel = *msg;

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
@brief Main gait loop - called by timer at fixed rate
@note This replaces your embedded while(true) loop
*/
void GaitController::gaitTimerCallback() {
    // Run startup sequence once on first timer callback
    if (!positions_initialized) {
        startup();
        positions_initialized = true;
        return;  // Let startup complete before entering gait modes
    }

    switch (gaitConfig.currentMode) {
        case Mode::MODE_STRAFE:
            Strafe();
            break;
        case Mode::MODE_NORMAL:
            Normal();
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
    }

    // Reset phase to 0 when starting up
    phase = 0;

    // Update start positions based on current height
    gaitConfig.setHeight(gaitConfig.currentHeight);

    // Generate trajectories for each leg to move to home position
    std::array<std::array<Vector3d, MAX_RESOLUTION>, MAX_LEGS + 1> trajectory;
    std::array<int, MAX_LEGS + 1> sizes{};
    for (int leg = 1; leg <= MAX_LEGS; ++leg) {
        trajectoryGen.GenStraightTrajectory(
            trajectory[leg].data(), 
            sizes[leg], 
            gaitConfig.homePos, 
            gaitConfig.startPosition.at(leg), 
            MAX_RESOLUTION-1
        );
    }
    for (int step = 0; step < MAX_RESOLUTION-1; ++step) {
        for (int leg = 1; leg <= MAX_LEGS; ++leg) {
            if (step < sizes[leg]) {
                Vector3d pos = trajectory[leg][step];

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
    }
}

/*
@brief Check if cmd_vel has changed significantly enough to warrant mid-trajectory regeneration
@return true if trajectory should be regenerated
*/
bool GaitController::shouldRegenerateTrajectory() {
    double dx = std::abs(filtered_cmd_vel.linear.x - trajectory_cmd_vel.linear.x);
    double dy = std::abs(filtered_cmd_vel.linear.y - trajectory_cmd_vel.linear.y);
    double dz = std::abs(filtered_cmd_vel.angular.z - trajectory_cmd_vel.angular.z);

    return (dx > CMD_VEL_CHANGE_THRESHOLD ||
            dy > CMD_VEL_CHANGE_THRESHOLD ||
            dz > CMD_VEL_CHANGE_THRESHOLD);
}

void GaitController::retargetSwingSplines() {
    double liftHeight = 0.020;
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
            filtered_cmd_vel.linear.x,
            filtered_cmd_vel.linear.y,
            currentPos,
            legNum,
            false
        );

        Vector3d rotationPos = trajectoryGen.rotationalTarget(
            filtered_cmd_vel.angular.z,
            currentPos,
            legNum,
            false,
            1.0
        );

        Vector3d idealTarget = trajectoryGen.BlendTargetPosition(
            currentPos,
            forwardPos,
            rotationPos
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
        int swingSize = trajectoryGen.gaitState.swingSizes[legNum];
        int stanceSize = trajectoryGen.gaitState.stanceSizes[legNum];
        
        // Get target position for this leg
        // Each leg is assigned EITHER swing OR stance trajectory for this phase (not both)
        Eigen::Vector3d targetPos;
        SwingSpline& spline = trajectoryGen.gaitState.swingSplines[legNum];

        if (spline.active) {
            targetPos = trajectoryGen.evalBezier(
                spline.P0,
                spline.P1,
                spline.P2,
                spline.P3,
                spline.s
            );

            // Advance spline progress
            double ds = idle ? 0.0 : (1.0 / MAX_RESOLUTION);
            spline.s += ds;
            if (spline.s >= 1.0) {
                spline.s = 1.0;
                // Snap to exact endpoint and deactivate to prevent drift
                targetPos = spline.P3;
                spline.active = false;
            }
        }
        else if (stanceSize > 0 && step < stanceSize) {
            targetPos = trajectoryGen.gaitState.stanceTrajectory[legNum][step];
        }
        else {
            continue;
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
    double liftHeight = 0.020;

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

    // Check if stick is idle (within deadzone)
    bool stickIdle = (std::abs(last_cmd_vel.linear.x) < DEADZONE &&
                      std::abs(last_cmd_vel.linear.y) < DEADZONE &&
                      std::abs(last_cmd_vel.linear.z) < DEADZONE &&
                      std::abs(last_cmd_vel.angular.x) < DEADZONE &&
                      std::abs(last_cmd_vel.angular.y) < DEADZONE &&
                      std::abs(last_cmd_vel.angular.z) < DEADZONE);

    // Track if we've ever moved (received non-zero command and started stepping)
    if (!stickIdle && step > 0) {
        hasMovedFromStart = true;
        returnCompleted = false;  // Reset since we're moving again
    }

    if (stickIdle) {
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
    return stickIdle; // Return whether stick is idle
}

/*
@brief Handles the strafing motion of the hexapod
@note This mode uses all cmd_vel components:
      - linear.x: forward/backward
      - linear.y: lateral (strafe left/right)
      - angular.z: rotation (turning)
*/
void GaitController::Strafe() {
    double liftHeight = 0.020;  // meters (20mm)

    // Check if stick is idle
    bool stickIdle = HandleIdleReturn();
    if (idleReturning) return;

    // Ensure gait config is set
    trajectoryGen.EnsureGaitConfig();

    if (shouldRegenerateTrajectory()) {
        retargetSwingSplines();
        trajectory_cmd_vel = filtered_cmd_vel;
    }

    // Calculate stride multiplier safely
    double strideMultiplier = trajectoryGen.CalculateStrideMultiplier();

    // Generate trajectories at the start of each phase
    if (step == 0) {
        trajectory_cmd_vel = filtered_cmd_vel;
        trajectoryGen.GenerateTrajectories(
            liftHeight,
            MAX_RESOLUTION-1,
            // Swing target - use Twist-based direction
            [this](int legNum, const Vector3d& currentPos) {
                Vector3d forwardPos = trajectoryGen.linearTarget(filtered_cmd_vel.linear.x, filtered_cmd_vel.linear.y, currentPos, legNum, false);
                Vector3d rotationPos = trajectoryGen.rotationalTarget(filtered_cmd_vel.angular.z, currentPos, legNum, false, 1.0);
                Vector3d targetPos = trajectoryGen.BlendTargetPosition(currentPos, forwardPos, rotationPos);
                return targetPos;
            },
            // Stance target - use Twist-based direction (inverted)
            [this, strideMultiplier](int legNum, const Vector3d& currentPos) {
                Vector3d forwardPos = trajectoryGen.linearTarget(filtered_cmd_vel.linear.x, filtered_cmd_vel.linear.y, currentPos, legNum, true);
                Vector3d rotationPos = trajectoryGen.rotationalTarget(filtered_cmd_vel.angular.z, currentPos, legNum, true, strideMultiplier);
                Vector3d targetPos = trajectoryGen.BlendTargetPosition(currentPos, forwardPos, rotationPos);
                return targetPos;
            },
            current_leg_positions,
            phase
        );
    }
    // Move all legs for this step
    PerformLegStep(stickIdle, MAX_RESOLUTION-1);
}

/*
@brief Handles the normal walking motion of the hexapod (car-like steering)
@note In this mode:
      - linear.x: forward/backward (left stick Y)
      - linear.y: turning (left stick X) - remapped to rotation
*/
void GaitController::Normal() {
    double liftHeight = 0.020;  // meters (20mm)

    // Check if stick is idle
    bool stickIdle = HandleIdleReturn();
    if (idleReturning) return;

    // Ensure gait config is set
    trajectoryGen.EnsureGaitConfig();

    if (shouldRegenerateTrajectory()) {
        retargetSwingSplines();
        trajectory_cmd_vel = filtered_cmd_vel;
    }

    // Calculate stride multiplier safely
    double strideMultiplier = trajectoryGen.CalculateStrideMultiplier();

    // Generate trajectories at the start of each phase
    if (step == 0) {
        trajectory_cmd_vel = filtered_cmd_vel;
        trajectoryGen.GenerateTrajectories(
            liftHeight,
            MAX_RESOLUTION-1,
            // Swing target - use Twist-based direction
            [this](int legNum, const Vector3d& currentPos) {
                Vector3d forwardPos = trajectoryGen.linearTarget(filtered_cmd_vel.linear.x, 0.0, currentPos, legNum, false);
                Vector3d rotationPos = trajectoryGen.rotationalTarget(filtered_cmd_vel.linear.y, currentPos, legNum, false, 1.0);
                Vector3d targetPos = trajectoryGen.BlendTargetPosition(currentPos, forwardPos, rotationPos);
                return targetPos;
            },
            // Stance target - use Twist-based direction (inverted)
            [this, strideMultiplier](int legNum, const Vector3d& currentPos) {
                Vector3d forwardPos = trajectoryGen.linearTarget(filtered_cmd_vel.linear.x, 0.0, currentPos, legNum, true);
                Vector3d rotationPos = trajectoryGen.rotationalTarget(filtered_cmd_vel.linear.y, currentPos, legNum, true, strideMultiplier);
                Vector3d targetPos = trajectoryGen.BlendTargetPosition(currentPos, forwardPos, rotationPos);
                return targetPos;
            },
            current_leg_positions,
            phase
        );
    }
    // Move all legs for this step
    PerformLegStep(stickIdle, MAX_RESOLUTION-1);
}
