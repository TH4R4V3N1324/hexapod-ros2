#include "rclcpp/rclcpp.hpp"
#include "hexapod_gait_controller/trajectory_generator.hpp"
#include "hexapod_gait_controller/kinematic_solver.hpp"
#include "hexapod_gait_controller/gait_config.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

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

        RCLCPP_INFO(this->get_logger(), "GaitController node started at %.1f Hz", loop_rate_hz);
    }    
private:
    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Publisher<JointState>::SharedPtr joint_cmd_pub;
    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub;
    rclcpp::TimerBase::SharedPtr gait_timer;
    
    Twist last_cmd_vel;
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
    
    // Threshold for mid-trajectory regeneration
    static constexpr double CMD_VEL_CHANGE_THRESHOLD = 0.05;
    
public:
    void cmdVelCallback(const Twist::SharedPtr msg);
    void jointStateCallback(const JointState::SharedPtr msg);
    void gaitTimerCallback();
    void home();
    void startup();
    bool shouldRegenerateTrajectory();
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
    // Check if current cmd_vel is zero (idle)
    bool current_is_zero = (std::abs(last_cmd_vel.linear.x) < CMD_VEL_CHANGE_THRESHOLD &&
                            std::abs(last_cmd_vel.linear.y) < CMD_VEL_CHANGE_THRESHOLD &&
                            std::abs(last_cmd_vel.angular.z) < CMD_VEL_CHANGE_THRESHOLD);
    
    // If cmd_vel is zero, don't regenerate (stop executing)
    if (current_is_zero) {
        return false;
    }
    
    double dx = std::abs(last_cmd_vel.linear.x - trajectory_cmd_vel.linear.x);
    double dy = std::abs(last_cmd_vel.linear.y - trajectory_cmd_vel.linear.y);
    double dz = std::abs(last_cmd_vel.angular.z - trajectory_cmd_vel.angular.z);
    
    return (dx > CMD_VEL_CHANGE_THRESHOLD || 
            dy > CMD_VEL_CHANGE_THRESHOLD || 
            dz > CMD_VEL_CHANGE_THRESHOLD);
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
        if (swingSize > 0 && step < swingSize) {
            // Leg is in swing group - use swing trajectory
            targetPos = trajectoryGen.gaitState.swingTrajectory[legNum][step];
        } else if (stanceSize > 0 && step < stanceSize) {
            // Leg is in stance group - use stance trajectory
            targetPos = trajectoryGen.gaitState.stanceTrajectory[legNum][step];
        } else {
            continue; // Skip if no valid trajectory or step out of bounds
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
        RCLCPP_ERROR(this->get_logger(), "[returnToStart] Invalid phase: %d, config size: %zu", phase, trajectoryGen.gaitState.config.size());
        idleReturning = false;
        counter = 0;
        step = 0;
        return;
    }

    if (step == 0) {
        // Get current leg positions (estimate from previous trajectory or use defaults)
        std::array<Vector3d, MAX_LEGS + 1> currentPositions{};
        for (int i = 1; i <= MAX_LEGS; ++i) {
            int swingSize = trajectoryGen.gaitState.swingSizes[i];
            int stanceSize = trajectoryGen.gaitState.stanceSizes[i];
            // Use last trajectory position if available, otherwise use home position
            if (swingSize > 0) {
                currentPositions[i] = trajectoryGen.gaitState.swingTrajectory[i][swingSize - 1];
            } else if (stanceSize > 0) {
                currentPositions[i] = trajectoryGen.gaitState.stanceTrajectory[i][stanceSize - 1];
            } else {
                currentPositions[i] = Vector3d(0.2, 0, 0.15); // Default position (X outward, Z down)
            }
        }
        
        trajectoryGen.GenerateTrajectories(
            liftHeight,
            MAX_RESOLUTION-1,
            // Swing: move to start position
            [this](int legNum, const Vector3d& currentPos) {
                auto it = gaitConfig.startPosition.find(legNum);
                return (it != gaitConfig.startPosition.end()) ? it->second : currentPos;
            },
            // Stance: hold current position
            [](int, const Vector3d& currentPos) {
                return currentPos;
            },
            currentPositions,
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

    // Check if stick is idle
    bool stickIdle = (last_cmd_vel.linear.x == 0.0 &&
                      last_cmd_vel.linear.y == 0.0 &&
                      last_cmd_vel.linear.z == 0.0 &&
                      last_cmd_vel.angular.x == 0.0 &&
                      last_cmd_vel.angular.y == 0.0 &&
                      last_cmd_vel.angular.z == 0.0);
    if (stickIdle) idleCount++;
    else idleCount = 0;

    // Handle idle/return-to-start logic
    if (idleCount > idleThreshold || idleReturning) {
        if (!idleReturning) {
            idleReturning = true;
            step = 0;
        }
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

    // Calculate stride multiplier safely
    double strideMultiplier = trajectoryGen.CalculateStrideMultiplier();

    // Generate trajectories at the start of each phase
    if (step == 0) {
        trajectoryGen.GenerateTrajectories(
            liftHeight,
            MAX_RESOLUTION-1,
            // Swing target - use Twist-based direction
            [this](int legNum, const Vector3d& currentPos) {
                Vector3d forwardPos = trajectoryGen.linearTarget(last_cmd_vel.linear.x, last_cmd_vel.linear.y, currentPos, legNum, false);
                Vector3d rotationPos = trajectoryGen.rotationalTarget(last_cmd_vel.angular.z, currentPos, legNum, false, 1.0);
                Vector3d targetPos = trajectoryGen.BlendTargetPosition(currentPos, forwardPos, rotationPos);
                return targetPos;
            },
            // Stance target - use Twist-based direction (inverted)
            [this, strideMultiplier](int legNum, const Vector3d& currentPos) {
                Vector3d forwardPos = trajectoryGen.linearTarget(last_cmd_vel.linear.x, last_cmd_vel.linear.y, currentPos, legNum, true);
                Vector3d rotationPos = trajectoryGen.rotationalTarget(last_cmd_vel.angular.z, currentPos, legNum, true, strideMultiplier);
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

    // Calculate stride multiplier safely
    double strideMultiplier = trajectoryGen.CalculateStrideMultiplier();

    // Generate trajectories at the start of each phase
    if (step == 0) {
        trajectoryGen.GenerateTrajectories(
            liftHeight,
            MAX_RESOLUTION-1,
            // Swing target - use Twist-based direction
            [this](int legNum, const Vector3d& currentPos) {
                Vector3d forwardPos = trajectoryGen.linearTarget(last_cmd_vel.linear.x, 0.0, currentPos, legNum, false);
                Vector3d rotationPos = trajectoryGen.rotationalTarget(last_cmd_vel.linear.y, currentPos, legNum, false, 1.0);
                Vector3d targetPos = trajectoryGen.BlendTargetPosition(currentPos, forwardPos, rotationPos);
                return targetPos;
            },
            // Stance target - use Twist-based direction (inverted)
            [this, strideMultiplier](int legNum, const Vector3d& currentPos) {
                Vector3d forwardPos = trajectoryGen.linearTarget(last_cmd_vel.linear.x, 0.0, currentPos, legNum, true);
                Vector3d rotationPos = trajectoryGen.rotationalTarget(last_cmd_vel.linear.y, currentPos, legNum, true, strideMultiplier);
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
