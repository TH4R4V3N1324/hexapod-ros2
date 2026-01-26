# Hexapod ROS2 Architecture

This document describes the ROS2 package architecture for the Hexapod robot, designed to work with an ESP32-S3 as a dumb I/O board.

## Table of Contents

- [System Overview](#system-overview)
- [Package Summary](#package-summary)
- [1. hexapod\_msgs](#1-hexapod_msgs)
- [2. hexapod\_description](#2-hexapod_description)
- [3. hexapod\_hardware\_interface](#3-hexapod_hardware_interface)
- [4. hexapod\_gait\_controller](#4-hexapod_gait_controller)
- [5. hexapod\_control](#5-hexapod_control)
- [6. hexapod\_teleop](#6-hexapod_teleop)
- [Data Flow](#data-flow)
- [Topic Reference](#topic-reference)
- [Service Reference](#service-reference)
- [Launch Files](#launch-files)

---

## System Overview

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                                    ROS2 System                                       │
│                                                                                      │
│  ┌────────────────┐     ┌────────────────────┐     ┌──────────────────────────────┐ │
│  │ hexapod_teleop │────▶│  hexapod_control   │────▶│  hexapod_gait_controller     │ │
│  │  (user input)  │     │  (behavior/state)  │     │  (motion + IK library)       │ │
│  └────────────────┘     └────────────────────┘     └──────────────┬───────────────┘ │
│                                                                    │                 │
│                                                                    ▼                 │
│  ┌────────────────────────────────────────────────────────────────────────────────┐ │
│  │                      hexapod_hardware_interface                                 │ │
│  │                      (ESP32 serial bridge)                                      │ │
│  └────────────────────────────────────────────────────────────────┬───────────────┘ │
│                                                                    │                 │
│  ┌────────────────┐                                               │                 │
│  │hexapod_descrip-│  URDF/meshes/configs used by all packages     │                 │
│  │tion            │                                               │                 │
│  └────────────────┘                                               │                 │
└───────────────────────────────────────────────────────────────────┼─────────────────┘
                                                                    │ Serial
                                                                    ▼
                                                        ┌───────────────────────┐
                                                        │   ESP32-S3 (Dumb I/O) │
                                                        │   PWM + Sensors +     │
                                                        │   ESP-NOW Relay       │
                                                        └───────────────────────┘
```

### Design Philosophy

- **ESP32 as Dumb I/O**: The ESP32-S3 main board only handles:
  - Receiving joint angles → driving servo PWM
  - Reading sensors → sending to ROS2
  - Relaying ESP-NOW controller data → sending to ROS2
- **ROS2 as Brain**: All intelligence (gait generation, inverse kinematics, behavior control) runs on the companion computer
- **ESP-NOW Controller**: A separate ESP32 controller connects wirelessly to the main board and provides joystick input

---

## Package Summary

| Package | Purpose | Key Components |
|---------|---------|----------------|
| `hexapod_msgs` | Custom message/service/action definitions | ControllerInput, PowerStatus, LegContactArray |
| `hexapod_description` | Robot model and configuration | URDF, meshes, parameter files |
| `hexapod_hardware_interface` | ESP32 serial communication | esp32_bridge_node |
| `hexapod_gait_controller` | Motion planning + kinematics | gait_controller_node, kinematics library |
| `hexapod_control` | Behavior and state management | behavior_manager_node, config_manager_node |
| `hexapod_teleop` | Alternative user input methods | joy_teleop_node, keyboard_teleop_node |

---

## 1. hexapod_msgs

**Purpose**: Centralized custom message, service, and action definitions. All other packages depend on this.

### Messages

#### ControllerInput.msg

Raw ESP-NOW controller data relayed from ESP32. Maps directly to the embedded `ControlPacket` structure.

```
std_msgs/Header header
int16 joy_left_x
int16 joy_left_y
int16 joy_right_x
int16 joy_right_y
int16 height
uint8 command
int16[3] command_args
```

#### LegContactArray.msg

Ground contact state for each leg (1-6). Used for adaptive gait and terrain sensing.

```
std_msgs/Header header
bool[6] contacts
```

#### PowerStatus.msg

Power monitoring from INA260. For battery management and safety shutdowns.

```
float32 voltage
float32 current
float32 power
float32 battery_percent
bool low_battery_warning
```

#### HexapodState.msg

Overall robot state broadcast for monitoring/logging.

```
uint8 mode
uint8 gait
float32 body_height
int8 gait_phase
bool is_moving
bool is_stable
```

#### LegCommand.msg

Single leg command with both Cartesian position and joint angles.

```
uint8 leg_id
geometry_msgs/Point position
float64[3] joint_angles
```

### Services

#### SetGait.srv

Change walking gait pattern.

```
# Request
uint8 gait  # 0=tripod, 1=ripple, 2=wave
---
# Response
bool success
string message
```

#### SetMode.srv

Change behavior mode.

```
# Request
uint8 mode  # 0=normal, 1=strafe, 2=tilt, 3=config
---
# Response
bool success
string message
```

#### SetHeight.srv

Adjust body height (clamped to valid range).

```
# Request
float32 height_mm
---
# Response
bool success
float32 actual_height
```

#### GetLegConfig.srv

Retrieve servo calibration offsets.

```
# Request
uint8 leg_id
---
# Response
int16[3] offsets  # coxa, femur, tibia
```

#### SetLegConfig.srv

Set servo calibration offset.

```
# Request
uint8 leg_id
uint8 joint_id
int16 offset
---
# Response
bool success
```

#### SaveConfig.srv

Persist calibration to file.

```
# Request
---
# Response
bool success
string filepath
```

#### EmergencyStop.srv

Disable all servos immediately.

```
# Request
bool stop
---
# Response
bool success
```

### Actions

#### HomeStance.action

Return all legs to home position smoothly.

```
# Goal
---
# Feedback
float32 progress
uint8 current_leg
---
# Result
bool success
float32 duration
```

#### ExecuteAnimation.action

Run predefined animations (startup, shutdown, wave).

```
# Goal
string animation_name
float32 speed
---
# Feedback
float32 progress
uint8 phase
---
# Result
bool success
```

---

## 2. hexapod_description

**Purpose**: Contains the robot's physical description (URDF), visual meshes, and default configuration parameters. This is a **data-only package** with no nodes.

### Directory Structure

```
hexapod_description/
├── urdf/
│   ├── hexapod.urdf.xacro          # Main robot description
│   ├── leg.urdf.xacro              # Single leg macro (reused 6x)
│   └── materials.xacro             # Colors/materials
├── meshes/
│   ├── body.stl
│   ├── coxa.stl
│   ├── femur.stl
│   └── tibia.stl
├── config/
│   ├── hexapod_params.yaml         # Default parameters
│   ├── servo_calibration.yaml      # Joint offsets
│   ├── gait_configs.yaml           # Gait phase patterns
│   └── joint_limits.yaml           # Min/max angles per joint
├── launch/
│   ├── display.launch.py           # RViz visualization
│   ├── hardware.launch.py          # Real robot
│   └── simulation.launch.py        # Gazebo
└── rviz/
    └── hexapod.rviz                # RViz config
```

### Configuration Files

#### hexapod_params.yaml

```yaml
hexapod:
  geometry:
    coxa_length: 50.5      # mm
    femur_length: 90.0     # mm
    tibia_length: 150.35   # mm
    body_radius: 100.0     # mm - distance from center to coxa
  
  legs:
    # Leg mounting angles and mirror flags
    leg_1: { angle: -45.0, mirrored: false, home: [0, 150, -120] }
    leg_2: { angle: 0.0, mirrored: false, home: [0, 150, -120] }
    leg_3: { angle: 45.0, mirrored: false, home: [0, 150, -120] }
    leg_4: { angle: 135.0, mirrored: true, home: [0, 150, -120] }
    leg_5: { angle: 180.0, mirrored: true, home: [0, 150, -120] }
    leg_6: { angle: -135.0, mirrored: true, home: [0, 150, -120] }
  
  motion:
    default_height: 120    # mm
    min_height: 60
    max_height: 180
    step_height: 40        # mm - lift height during swing
    step_resolution: 50    # trajectory points per step
```

#### gait_configs.yaml

```yaml
gaits:
  tripod:
    phase_groups: [[1, 3, 5], [2, 4, 6]]    # Two alternating groups
    duty_cycle: 0.5                          # 50% stance, 50% swing
  ripple:
    phase_groups: [[1, 4], [2, 5], [3, 6]]  # Three groups
    duty_cycle: 0.67
  wave:
    phase_groups: [[1], [2], [3], [4], [5], [6]]  # One leg at a time
    duty_cycle: 0.83
```

### Data Flow

- **To**: All other packages load URDF and configs via `xacro` and ROS2 parameter server
- **Used by**: `robot_state_publisher` for TF, RViz for visualization, Gazebo for simulation

---

## 3. hexapod_hardware_interface

**Purpose**: Bridge between ROS2 and ESP32. Handles serial communication, sensor data ingestion, and joint command transmission. The ESP32 is treated as a dumb I/O device.

### Nodes

#### esp32_bridge_node

Bidirectional serial protocol handler.

**Implementation Details**:
- Uses serial port (e.g., `/dev/ttyUSB0` at 115200 baud)
- Binary protocol with checksums for reliability
- Runs at 100Hz to match ESP32 sensor rate

**Subscribes to**:

| Topic | Type | Source | Purpose |
|-------|------|--------|---------|
| `/hexapod/joint_commands` | `sensor_msgs/JointState` | `gait_controller` | 18 joint angles to send to ESP32 for PWM |

**Publishes**:

| Topic | Type | Destination | Purpose |
|-------|------|-------------|---------|
| `/hexapod/controller_input` | `hexapod_msgs/ControllerInput` | `hexapod_control` | Raw joystick data relayed from ESP-NOW controller |
| `/hexapod/joint_states` | `sensor_msgs/JointState` | `robot_state_publisher`, RViz | Current joint positions |
| `/imu/data` | `sensor_msgs/Imu` | `hexapod_control` | Pitch/roll from BMI330 |
| `/hexapod/power` | `hexapod_msgs/PowerStatus` | `hexapod_control`, diagnostics | Voltage/current from INA260 |
| `/hexapod/leg_contacts` | `hexapod_msgs/LegContactArray` | `gait_controller` | Ground contact switches |

**Services**:

| Service | Purpose |
|---------|---------|
| `/hexapod/emergency_stop` | Sends stop command to ESP32, disables all servo PWM |

**Parameters**:

```yaml
esp32_bridge:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"
    baud_rate: 115200
    publish_rate: 100.0        # Hz
    command_timeout: 0.1       # seconds before servo disable
    reconnect_interval: 2.0    # seconds between reconnection attempts
```

### Serial Protocol

**ROS2 → ESP32 (Joint Commands)**:
```
┌──────┬────────┬────────────────────┬──────────┐
│ 0xAA │ Length │ float32[18] angles │ Checksum │
└──────┴────────┴────────────────────┴──────────┘
```

**ESP32 → ROS2 (Sensor Data)**:
```
┌──────┬────────┬───────────────────────────────────────────────┬──────────┐
│ 0xBB │ Length │ pitch, roll, current, voltage, contacts[6],   │ Checksum │
│      │        │ joy_lx, joy_ly, joy_rx, joy_ry, cmd, args[3]  │          │
└──────┴────────┴───────────────────────────────────────────────┴──────────┘
```

---

## 4. hexapod_gait_controller

**Purpose**: Combines gait pattern generation and inverse kinematics into a single package. The embedded `Animation`, `Move`, and `Calculate` classes are ported here. Kinematics is implemented as a **library within this package** (not a separate node).

### Package Structure

```
hexapod_gait_controller/
├── include/hexapod_gait_controller/
│   ├── kinematics.hpp              # IK/FK calculations
│   ├── trajectory_generator.hpp    # Swing/stance curve generation
│   ├── gait_patterns.hpp           # Tripod/ripple/wave configurations
│   └── leg_controller.hpp          # Per-leg state management
├── src/
│   ├── kinematics.cpp
│   ├── trajectory_generator.cpp
│   ├── gait_patterns.cpp
│   ├── leg_controller.cpp
│   └── gait_controller_node.cpp    # Main node
├── launch/
│   └── gait_controller.launch.py
└── config/
    └── gait_params.yaml
```

### Kinematics Library

Provides IK/FK functions used internally. Not exposed as services (faster, no RPC overhead).

```cpp
namespace hexapod_kinematics {

struct JointAngles {
    double coxa, femur, tibia;
};

struct LegConfig {
    double rotation_angle;    // Leg mounting angle
    bool is_mirrored;         // Left vs right side
    Eigen::Vector3d offset;   // Translation from body center
};

class Kinematics {
public:
    Kinematics(double coxa_len, double femur_len, double tibia_len);
    
    // Inverse kinematics: foot position → joint angles
    // Returns false if position unreachable
    bool solveIK(const Eigen::Vector3d& foot_pos, int leg_id, JointAngles& angles);
    
    // Forward kinematics: joint angles → foot position
    Eigen::Vector3d solveFK(const JointAngles& angles, int leg_id);
    
    // Transform between body frame and leg frame
    Eigen::Vector3d bodyToLegFrame(const Eigen::Vector3d& pos, int leg_id);
    Eigen::Vector3d legToBodyFrame(const Eigen::Vector3d& pos, int leg_id);
    
    void setLegConfig(int leg_id, const LegConfig& config);

private:
    double coxa_length_, femur_length_, tibia_length_;
    std::array<LegConfig, 6> leg_configs_;
};

} // namespace hexapod_kinematics
```

### Nodes

#### gait_controller_node

Main motion planning node. Generates leg trajectories based on velocity commands, runs IK, outputs joint angles at 100Hz.

**Key Responsibilities**:
1. Receive velocity commands (`/cmd_vel`)
2. Generate foot trajectories for current gait pattern
3. Run IK to convert foot positions to joint angles
4. Publish joint commands at high rate

**Subscribes to**:

| Topic | Type | Source | Why Needed |
|-------|------|--------|------------|
| `/hexapod/cmd_vel` | `geometry_msgs/Twist` | `hexapod_control` | Linear.x = forward, linear.y = strafe, angular.z = rotation |
| `/hexapod/leg_contacts` | `hexapod_msgs/LegContactArray` | `hardware_interface` | Detect early ground contact for adaptive stepping |
| `/imu/data` | `sensor_msgs/Imu` | `hardware_interface` | Body orientation for leveling on slopes |

**Publishes**:

| Topic | Type | Destination | Content |
|-------|------|-------------|---------|
| `/hexapod/joint_commands` | `sensor_msgs/JointState` | `hardware_interface` | 18 joint angles at 100Hz |
| `/hexapod/leg_poses` | `geometry_msgs/PoseArray` | RViz, debugging | Current foot positions in body frame |
| `/hexapod/state` | `hexapod_msgs/HexapodState` | `hexapod_control`, monitoring | Current gait, phase, stability status |

**Services**:

| Service | Purpose |
|---------|---------|
| `/hexapod/set_gait` | Switch gait pattern (tripod/ripple/wave) |
| `/hexapod/set_height` | Adjust body height with smooth interpolation |

**Action Servers**:

| Action | Purpose |
|--------|---------|
| `/hexapod/home_stance` | Return all legs to home position with progress feedback |
| `/hexapod/execute_animation` | Run predefined sequences (startup, shutdown, wave) |

**Parameters**:

```yaml
gait_controller:
  ros__parameters:
    # Geometry (loaded from hexapod_description)
    coxa_length: 50.5
    femur_length: 90.0
    tibia_length: 150.35
    
    # Motion
    control_rate: 100.0         # Hz
    default_gait: "tripod"
    step_height: 40.0           # mm
    trajectory_resolution: 50   # points per swing phase
    
    # Body
    default_height: 120.0       # mm
    
    # Stability
    enable_body_leveling: true
    max_level_correction: 20.0  # mm max leg adjustment
    
    # Adaptive gait
    enable_contact_adaptation: true
```

**Internal State Machine**:

```
┌──────────┐    cmd_vel    ┌──────────┐   all legs    ┌──────────┐
│   IDLE   │──────────────▶│ WALKING  │───grounded───▶│   IDLE   │
└──────────┘   (non-zero)  └────┬─────┘   (zero vel)  └──────────┘
      │                         │
      │                         │ home_stance action
      ▼                         ▼
┌──────────┐              ┌──────────┐
│ STARTUP  │              │  HOMING  │
└──────────┘              └──────────┘
```

---

## 5. hexapod_control

**Purpose**: High-level behavior control. Manages operating modes (normal/strafe/tilt/config), interprets controller input, and coordinates system-wide state. This is the embedded `StateFSM` and `CommandFSM` logic.

### Nodes

#### behavior_manager_node

State machine that interprets user input and manages operating modes.

**Subscribes to**:

| Topic | Type | Source | Why Needed |
|-------|------|--------|------------|
| `/hexapod/controller_input` | `hexapod_msgs/ControllerInput` | `hardware_interface` | Raw joystick data from ESP-NOW controller |
| `/hexapod/state` | `hexapod_msgs/HexapodState` | `gait_controller` | Current robot state for decision making |
| `/imu/data` | `sensor_msgs/Imu` | `hardware_interface` | Body orientation for tilt mode |
| `/hexapod/power` | `hexapod_msgs/PowerStatus` | `hardware_interface` | Battery level for warnings/shutdown |

**Publishes**:

| Topic | Type | Destination | Content |
|-------|------|-------------|---------|
| `/hexapod/cmd_vel` | `geometry_msgs/Twist` | `gait_controller` | Processed velocity command based on mode |
| `/hexapod/mode` | `std_msgs/UInt8` | Monitoring, UI | Current mode (0-3) |

**Services Called**:

| Service | When Called |
|---------|-------------|
| `/hexapod/set_gait` | When `command == CMD_SET_GAIT` received |
| `/hexapod/set_mode` | When `command == CMD_SET_MODE` received |
| `/hexapod/home_stance` | When `command == CMD_HOME_STANCE` received |
| `/hexapod/emergency_stop` | When battery critical or error detected |

**Mode Behaviors**:

| Mode | Joystick Mapping | Special Behavior |
|------|------------------|------------------|
| **NORMAL** | Left stick: forward/rotate, Right stick: height | Standard walking |
| **STRAFE** | Left stick: forward/strafe, Right stick: rotate/height | Omnidirectional movement |
| **TILT** | Left stick: pitch body, Right stick: roll body | IMU-assisted body tilting |
| **CONFIG** | Left stick: select leg/joint, Right stick: adjust offset | Servo calibration mode |

**Command Handling** (from embedded `CommandFSM`):

```cpp
void handleCommand(const ControllerInput& input) {
    switch (input.command) {
        case CMD_SET_GAIT:
            callService("/hexapod/set_gait", input.command_args[0]);
            break;
        case CMD_SET_MODE:
            current_mode_ = input.command_args[0];
            break;
        case CMD_HOME_STANCE:
            callAction("/hexapod/home_stance");
            break;
        case CMD_SET_CONFIG:
            callService("/hexapod/set_leg_config", 
                input.command_args[0],  // leg
                input.command_args[1],  // joint  
                input.command_args[2]); // offset
            break;
    }
}
```

#### config_manager_node

Manages servo calibration offsets. Replaces the embedded `ConfigManager` class.

**Services**:

| Service | Purpose |
|---------|---------|
| `/hexapod/get_leg_config` | Retrieve offsets for a leg |
| `/hexapod/set_leg_config` | Set offset for a specific joint |
| `/hexapod/save_config` | Write current offsets to YAML file |
| `/hexapod/load_config` | Load offsets from YAML file |

**Publishes**:

| Topic | Type | Purpose |
|-------|------|---------|
| `/hexapod/joint_offsets` | `sensor_msgs/JointState` | Current calibration offsets |

**Data Storage**:
- Offsets stored in `hexapod_description/config/servo_calibration.yaml`
- Loaded at startup, saved on request

---

## 6. hexapod_teleop

**Purpose**: Alternative input methods beyond the ESP-NOW controller. Useful for development, testing, and when ESP32 controller isn't available.

### Nodes

#### joy_teleop_node

Convert standard gamepad to controller input format.

**Subscribes to**:

| Topic | Type | Source |
|-------|------|--------|
| `/joy` | `sensor_msgs/Joy` | `joy` package (gamepad driver) |

**Publishes**:

| Topic | Type | Purpose |
|-------|------|---------|
| `/hexapod/cmd_vel` | `geometry_msgs/Twist` | Direct velocity commands |

**Parameters**:

```yaml
joy_teleop:
  ros__parameters:
    # Axis mappings (for Xbox controller)
    axis_linear_x: 1      # Left stick Y
    axis_linear_y: 0      # Left stick X
    axis_angular_z: 3     # Right stick X
    axis_height: 4        # Right stick Y
    
    # Button mappings
    btn_gait_cycle: 0     # A button
    btn_mode_cycle: 1     # B button
    btn_home: 2           # X button
    btn_emergency: 7      # Start button
    
    # Scaling
    max_linear: 100.0     # mm/s
    max_angular: 0.5      # rad/s
    deadzone: 0.1
```

#### keyboard_teleop_node (Optional)

Simple keyboard control for testing.

**Key Mappings**:
- WASD: Move forward/strafe
- QE: Rotate
- RF: Height up/down
- 1/2/3: Select gait
- Space: Home stance

---

## Data Flow

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                                                                                      │
│   ESP32 Controller ──ESP-NOW──▶ ESP32 Main Board                                    │
│                                      │                                               │
│                                      │ Serial @ 100Hz                               │
│                                      ▼                                               │
│   ┌─────────────────────────────────────────────────────────────────────────────┐   │
│   │                        esp32_bridge_node                                     │   │
│   │  OUT: /controller_input, /imu/data, /power, /leg_contacts, /joint_states    │   │
│   │  IN:  /joint_commands                                                        │   │
│   └───────┬────────────┬────────────┬────────────┬───────────────────────────────┘   │
│           │            │            │            │                                   │
│           ▼            ▼            ▼            │                                   │
│   /controller_input  /imu/data   /power     /leg_contacts                           │
│           │            │   │        │            │                                   │
│           ▼            │   │        ▼            │                                   │
│   ┌──────────────────┐ │   │  ┌──────────┐       │                                   │
│   │behavior_manager  │◀┘   │  │Diagnostics│      │                                   │
│   │                  │     │  │Low battery│      │                                   │
│   │ Mode: NORMAL     │     │  └──────────┘       │                                   │
│   │ CMD handling     │     │                     │                                   │
│   └────────┬─────────┘     │                     │                                   │
│            │               │                     │                                   │
│            ▼               │                     │                                   │
│      /cmd_vel              │                     │                                   │
│            │               │                     │                                   │
│            ▼               ▼                     ▼                                   │
│   ┌────────────────────────────────────────────────────────────────────────────┐    │
│   │                      gait_controller_node                                   │    │
│   │                                                                             │    │
│   │  ┌──────────────┐   ┌──────────────┐   ┌─────────────────┐                 │    │
│   │  │Gait Generator│──▶│Trajectory Gen│──▶│IK (kinematics)  │                 │    │
│   │  │tripod/ripple │   │swing/stance  │   │pos → angles     │                 │    │
│   │  └──────────────┘   └──────────────┘   └────────┬────────┘                 │    │
│   │                                                  │                          │    │
│   └──────────────────────────────────────────────────┼──────────────────────────┘    │
│                                                      │                               │
│                                                      ▼                               │
│                                              /joint_commands                         │
│                                                      │                               │
│                                                      ▼                               │
│   ┌─────────────────────────────────────────────────────────────────────────────┐   │
│   │                        esp32_bridge_node                                     │   │
│   └─────────────────────────────────────────────────────────────────────────────┘   │
│                                      │                                               │
│                                      │ Serial                                        │
│                                      ▼                                               │
│                              ESP32 Main Board                                        │
│                                      │                                               │
│                                      ▼ PWM                                           │
│                                  [18 Servos]                                         │
│                                                                                      │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

---

## Topic Reference

| Topic | Type | Publisher | Subscriber(s) | Rate |
|-------|------|-----------|---------------|------|
| `/hexapod/controller_input` | `ControllerInput` | `esp32_bridge` | `behavior_manager` | 50Hz |
| `/hexapod/joint_commands` | `JointState` | `gait_controller` | `esp32_bridge` | 100Hz |
| `/hexapod/joint_states` | `JointState` | `esp32_bridge` | `robot_state_publisher` | 100Hz |
| `/hexapod/cmd_vel` | `Twist` | `behavior_manager` | `gait_controller` | 50Hz |
| `/hexapod/leg_contacts` | `LegContactArray` | `esp32_bridge` | `gait_controller` | 100Hz |
| `/hexapod/power` | `PowerStatus` | `esp32_bridge` | `behavior_manager` | 10Hz |
| `/hexapod/state` | `HexapodState` | `gait_controller` | `behavior_manager` | 20Hz |
| `/hexapod/leg_poses` | `PoseArray` | `gait_controller` | RViz | 20Hz |
| `/hexapod/mode` | `UInt8` | `behavior_manager` | Monitoring | On change |
| `/imu/data` | `Imu` | `esp32_bridge` | `gait_controller`, `behavior_manager` | 100Hz |
| `/tf` | `TFMessage` | `robot_state_publisher` | RViz, others | 100Hz |

---

## Service Reference

| Service | Package | Purpose |
|---------|---------|---------|
| `/hexapod/set_gait` | `gait_controller` | Change gait pattern |
| `/hexapod/set_mode` | `control` | Change behavior mode |
| `/hexapod/set_height` | `gait_controller` | Adjust body height |
| `/hexapod/get_leg_config` | `control` | Get servo offsets |
| `/hexapod/set_leg_config` | `control` | Set servo offset |
| `/hexapod/save_config` | `control` | Save calibration |
| `/hexapod/load_config` | `control` | Load calibration |
| `/hexapod/emergency_stop` | `hardware_interface` | Disable servos |

---

## Launch Files

### hexapod_bringup/launch/hexapod.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
        # Robot description & TF
        IncludeLaunchDescription(
            'hexapod_description/launch/robot_state_publisher.launch.py'
        ),
        
        # Hardware interface
        Node(
            package='hexapod_hardware_interface',
            executable='esp32_bridge_node',
            parameters=['config/esp32_params.yaml']
        ),
        
        # Gait controller (includes kinematics library)
        Node(
            package='hexapod_gait_controller',
            executable='gait_controller_node',
            parameters=[
                'config/gait_params.yaml',
                'config/hexapod_params.yaml'
            ]
        ),
        
        # Behavior manager
        Node(
            package='hexapod_control',
            executable='behavior_manager_node',
            parameters=['config/control_params.yaml']
        ),
        
        # Config manager
        Node(
            package='hexapod_control',
            executable='config_manager_node',
            parameters=['config/servo_calibration.yaml']
        ),
    ])
```

---

## Mapping: Embedded Code → ROS2

| Embedded Component | ROS2 Equivalent |
|--------------------|-----------------|
| `ControlPacket` | `/hexapod/controller_input` topic |
| `HexPacket` | `/hexapod/state` topic |
| `CommandFSM()` | Service handlers in `behavior_manager_node` |
| `StateFSM()` | Main loop in `behavior_manager_node` |
| `Animation` class | `gait_controller_node` |
| `Move` class | `gait_controller_node` + kinematics library |
| `Calculate` class | `hexapod_kinematics` library |
| `Servo` class | `esp32_bridge_node` → ESP32 |
| `ConfigManager` | `config_manager_node` |
| `IMUSensor` | `esp32_bridge_node` → `/imu/data` |
| `CurrentSensor` | `esp32_bridge_node` → `/hexapod/power` |
| `LegContactSensor` | `esp32_bridge_node` → `/hexapod/leg_contacts` |
| ESP-NOW comm | Relayed through ESP32 to `/hexapod/controller_input` |

---

## ESP32 Firmware (Simplified)

With ROS2 handling all intelligence, the ESP32 code becomes minimal:

```cpp
// main.cpp - Dumb I/O only
void setup() {
    Serial.begin(115200);
    servoController.init();
    imu.init();
    currentSensor.init();
    legContact.init();
    initEspNow();
}

void loop() {
    // 1. Receive joint angles from ROS2 → set servos
    if (receiveJointAngles(angles)) {
        for (int i = 0; i < 18; i++) {
            servoController.setAngle(i, angles[i]);
        }
    }
    
    // 2. Send sensor data to ROS2
    sendSensorData(
        imu.getPitch(), imu.getRoll(),
        currentSensor.readCurrent(),
        currentSensor.readVoltage(),
        legContacts
    );
    
    // 3. Relay ESP-NOW controller input to ROS2
    if (controllerDataReceived) {
        sendControllerInput(controlPacket);
    }
}
```

**Removed from ESP32**: `Animation`, `Move`, `Calculate`, `StateFSM`, `CommandFSM` — all moved to ROS2.
