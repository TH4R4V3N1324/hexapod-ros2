#include "rclcpp/rclcpp.hpp"
#include "hexapod_gait_controller/trajectory_generator.hpp"
#include "hexapod_gait_controller/kinematic_solver.hpp"
#include "hexapod_gait_controller/gait_config.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using geometry_msgs::msg::Twist;
using sensor_msgs::msg::JointState;
using hexapod_gait_controller::MAX_LEGS;
using hexapod_gait_controller::TrajectoryGenerator;
using hexapod_gait_controller::GaitConfig;
using Eigen::Vector3d;

class GaitController : public rclcpp::Node { public: GaitController() : Node("gait_controller") {
    cmd_vel_sub = this->create_subscription<Twist>( "cmd_vel", 10, std::bind(&GaitController::cmdVelCallback, this, std::placeholders::_1));
    joint_cmd_pub = this->create_publisher<JointState>("joint_states", 10);
    RCLCPP_INFO(this->get_logger(), "GaitController node has been started.");
}    
private:
    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Publisher<JointState>::SharedPtr joint_cmd_pub;
    Twist last_cmd_vel;
    TrajectoryGenerator trajectoryGen;
    GaitConfig gaitConfig;
    hexapod_gait_controller::KinematicSolver ikSolver;
    uint8_t step = 0;
    uint8_t phase = 0;
    bool idleReturning = false;
public:
    void cmdVelCallback(const Twist::SharedPtr msg);
    void PerformLegStep(bool idle, int resolution, bool handlePhaseTransition = true);
    void returnToStart();
    bool HandleIdleReturn();
    void ExecuteGait(const Twist& velocityCmd, int liftHeight = 50, int resolution = 50);
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
    RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear(%.2f, %.2f, %.2f), angular(%.2f, %.2f, %.2f)",
                msg->linear.x, msg->linear.y, msg->linear.z,
                msg->angular.x, msg->angular.y, msg->angular.z);
    // Here you would typically update the gait based on the received cmd_vel
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
        Eigen::Vector3d targetPos;
        if (step < swingSize) {
            targetPos = trajectoryGen.gaitState.swingTrajectory[legNum][step];
        } else if (step < stanceSize) {
            targetPos = trajectoryGen.gaitState.stanceTrajectory[legNum][step];
        } else {
            continue; // Skip if no valid trajectory
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
    static bool trajectoryGenerated = false;
    int liftHeight = 50;
    int resolution = 50;

    // Safety check: phase must be valid
    if (phase >= trajectoryGen.gaitState.config.size()) {
        RCLCPP_ERROR(this->get_logger(), "[returnToStart] Invalid phase: %d, config size: %zu", phase, trajectoryGen.gaitState.config.size());
        idleReturning = false;
        counter = 0;
        step = 0;
        trajectoryGenerated = false;
        return;
    }

    if (step == 0 && !trajectoryGenerated) {
        // Get current leg positions (estimate from previous trajectory or use defaults)
        std::array<Vector3d, MAX_LEGS + 1> currentPositions{};
        for (int i = 1; i <= MAX_LEGS; ++i) {
            // Use last trajectory position if available, otherwise use home position
            if (trajectoryGen.gaitState.swingSizes[i] > 0) {
                currentPositions[i] = trajectoryGen.gaitState.swingTrajectory[i][0];
            } else if (trajectoryGen.gaitState.stanceSizes[i] > 0) {
                currentPositions[i] = trajectoryGen.gaitState.stanceTrajectory[i][0];
            } else {
                currentPositions[i] = Vector3d(0, 130, -50); // Default home position
            }
        }
        
        trajectoryGen.GenerateTrajectories(
            liftHeight,
            resolution,
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
        trajectoryGenerated = true;
    }

    // Move all legs for this step
    PerformLegStep(false, resolution, false);

    // Phase transition
    if (step > resolution) {
        counter++;
        step = 0;
        phase = (phase + 1) % trajectoryGen.gaitState.config.size();

        // After all phases, finish return-to-start and handle gait change if requested
        if (counter > static_cast<int>(trajectoryGen.gaitState.config.size())) {
            counter = 0;
            idleReturning = false;
            trajectoryGenerated = false;

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
@brief Common gait execution logic for leg-based locomotion
@param velocityCmd The velocity command to execute (can be mode-adjusted)
@param liftHeight The height to lift legs during swing phase
@param resolution The number of steps in the trajectory
@note This is a helper function called by specific gait modes (Strafe, Normal, etc.)
*/
void GaitController::ExecuteGait(const Twist& velocityCmd, int liftHeight, int resolution) {
    // Check if stick is idle
    bool stickIdle = HandleIdleReturn();
    if (idleReturning) return;

    // Ensure gait config is set
    trajectoryGen.EnsureGaitConfig();

    // Calculate stride multiplier safely
    double strideMultiplier = trajectoryGen.CalculateStrideMultiplier();

    // Generate trajectories at the start of each phase
    if (step == 0) {
        // Get current leg positions from previous trajectory or use defaults
        std::array<Vector3d, MAX_LEGS + 1> currentPositions{};
        for (int i = 1; i <= MAX_LEGS; ++i) {
            if (trajectoryGen.gaitState.swingSizes[i] > 0) {
                currentPositions[i] = trajectoryGen.gaitState.swingTrajectory[i][0];
            } else if (trajectoryGen.gaitState.stanceSizes[i] > 0) {
                currentPositions[i] = trajectoryGen.gaitState.stanceTrajectory[i][0];
            } else {
                currentPositions[i] = Vector3d(0, 130, -50); // Default home position
            }
        }

        trajectoryGen.GenerateTrajectories(
        liftHeight,
        resolution,
        // Swing target - single call handles both translation and rotation
        [this, velocityCmd](int legNum, const Vector3d& currentPos) {
            return trajectoryGen.direction(velocityCmd, currentPos, legNum, false, 1.0, true);
        },
        // Stance target - single call handles both translation and rotation
        [this, velocityCmd, strideMultiplier](int legNum, const Vector3d& currentPos) {
            return trajectoryGen.direction(velocityCmd, currentPos, legNum, true, strideMultiplier, true);
        },
        currentPositions,
        phase
        );
    }
    // Move all legs for this step
    PerformLegStep(stickIdle, resolution);
}

/*
@brief Handles the strafing motion of the hexapod
@note This mode uses all cmd_vel components:
      - linear.x: forward/backward
      - linear.y: lateral (strafe left/right)
      - angular.z: rotation (turning)
*/
void GaitController::Strafe() {
    // Use cmd_vel as-is for full omnidirectional movement
    ExecuteGait(last_cmd_vel);
}

/*
@brief Handles the normal walking motion of the hexapod
@note This mode uses only forward and rotation:
      - linear.x: forward/backward
      - angular.z: rotation (turning)
      - linear.y: ignored (no lateral movement)
*/
void GaitController::Normal() {
    // Create mode-adjusted cmd_vel: disable lateral movement
    Twist normalVel = last_cmd_vel;
    normalVel.linear.y = 0.0;  // No lateral strafe in normal mode
    ExecuteGait(normalVel);
}
