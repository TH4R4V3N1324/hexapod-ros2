# hexapod-ros2

ROS 2 workspace for a hexapod robot with:
- gait generation (`hexapod_gait_controller`)
- teleoperation (`hexapod_teleop`)
- robot model + Gazebo simulation (`hexapod_description`)
- interface definitions (`hexapod_interfaces`)
- hardware interface scaffolding (`hexapod_hardware_interface`)
- bringup launch files (`hexapod_bringup`, `operator_bringup`)

> This README is based on the current repository implementation. It intentionally does **not** use `ROS2_ARCHITECTURE.md` as a source of truth.

---

## Repository layout

```text
.
├── hexapod_ws/
│   ├── Dockerfile
│   ├── docker-compose.yml
│   ├── docker-compose.dev.yml
│   └── src/
│       ├── hexapod_bringup/
│       ├── hexapod_control/
│       ├── hexapod_description/
│       ├── hexapod_gait_controller/
│       ├── hexapod_hardware_interface/
│       ├── hexapod_interfaces/
│       └── hexapod_teleop/
├── operator_ws/
│   ├── docker-compose.yml
│   ├── dockerfile
│   └── src/operator_bringup/
├── steamdeck/
│   ├── hexapod_launch.sh
│   ├── ros2_hexapod_launch.sh
│   └── images/
└── init_operator.sh
```

---

## Packages and executables

| Package | Type | Main contents |
|---|---|---|
| `hexapod_interfaces` | Interface package | Custom messages/services (`ControllerInput`, `HexapodState`, `PowerStatus`, `LegContact`, `SetGait`, `SetMode`, `SetHeight`, `GetCapabilities`, `IKSolver`, `FKSolver`) |
| `hexapod_gait_controller` | C++ nodes/libs | `gait_controller`, `leg_axis_tester`, trajectory generation, IK/FK math |
| `hexapod_teleop` | C++ nodes | `controller_teleop`, `keyboard_teleop` |
| `hexapod_hardware_interface` | C++ nodes/libs | `hardware_interface_fake`, `hardware_interface`, servo/sensor handlers |
| `hexapod_description` | Description/sim + C++ node | URDF/Xacro, Gazebo world/config, RViz config, `joint_state_relay` |
| `hexapod_bringup` | Launch-only | `hexapod_fake.launch.py`, `hexapod_simulation.launch.py` |
| `operator_bringup` | Launch-only | `operator.launch.py` (`joy_node` + `foxglove_bridge`) |
| `hexapod_control` | Placeholder | Empty `config_manager.cpp` / `behaviour_manager.cpp`, no built node currently |

---

## Runtime/data flow (current implementation)

### A) Fake hardware flow (`hexapod_fake.launch.py`)

1. `joy_node` publishes `sensor_msgs/msg/Joy` on `joy`
2. `controller_teleop` converts joystick axes to `geometry_msgs/msg/Twist` on `cmd_vel`
3. `gait_controller`:
   - subscribes: `cmd_vel`, `joint_states`
   - publishes: `joint_commands` (`sensor_msgs/msg/JointState`)
   - serves: `set_gait`, `set_mode`, `set_height`, `get_capabilities`
4. `hardware_interface_fake`:
   - `ServoHandler` subscribes `joint_commands`, simulates actuator movement, publishes `joint_states`
   - `SensorHandler` publishes fake `imu_data`, `power_status_data`, `leg_contact_data`
5. `robot_state_publisher` uses `hexapod.urdf.xacro`
6. `foxglove_bridge` is launched for remote visualization

### B) Gazebo simulation flow (`hexapod_simulation.launch.py`)

1. Launches `hexapod_description/launch/gazebo.launch.py`
2. Gazebo Harmonic starts via `gz sim -r worlds/hexapod_world.sdf`
3. `robot_state_publisher` publishes robot description from `hexapod_primitives.urdf.xacro`
4. `ros_gz_sim create` spawns the robot from `robot_description`
5. Controllers are spawned in order:
   - `joint_state_broadcaster`
   - `hexapod_joint_controller`
6. `joint_state_relay` converts `joint_commands` into `/hexapod_joint_controller/commands`
7. `ros_gz_bridge parameter_bridge` bridges `/clock`, `/imu`, `/imu/mag`, `/front_depth/*`, `/lidar/scan` (plus optional RGB/depth image topics)
8. `gait_controller`, `controller_teleop`, `foxglove_bridge`, and RViz run alongside simulation

---

## Topics and services

### Core topics

| Name | Type | Producers | Consumers |
|---|---|---|---|
| `joy` | `sensor_msgs/msg/Joy` | `joy_node` | `controller_teleop` |
| `cmd_vel` | `geometry_msgs/msg/Twist` | `controller_teleop`, `keyboard_teleop` | `gait_controller` |
| `joint_commands` | `sensor_msgs/msg/JointState` | `gait_controller`, `leg_axis_tester` | `ServoHandler`, `joint_state_relay` |
| `joint_states` | `sensor_msgs/msg/JointState` | `ServoHandler` / `joint_state_broadcaster` | `gait_controller`, RViz/robot_state_publisher |
| `/hexapod_joint_controller/commands` | `std_msgs/msg/Float64MultiArray` | `joint_state_relay` | `ros2_control` joint group controller |
| `imu_data` | `sensor_msgs/msg/Imu` | `SensorHandler` | external consumers |
| `power_status_data` | `hexapod_interfaces/msg/PowerStatus` | `SensorHandler` | external consumers |
| `leg_contact_data` | `hexapod_interfaces/msg/LegContact` | `SensorHandler` | external consumers |

### Services exposed by `gait_controller`

| Name | Type | Purpose |
|---|---|---|
| `set_gait` | `hexapod_interfaces/srv/SetGait` | switch gait (`tripod`, `ripple`, `wave`) |
| `set_mode` | `hexapod_interfaces/srv/SetMode` | switch mode (`normal`, `strafe`, `tilt`, `config`) |
| `set_height` | `hexapod_interfaces/srv/SetHeight` | update target body height |
| `get_capabilities` | `hexapod_interfaces/srv/GetCapabilities` | returns supported modes/gaits and max velocities |

> `IKSolver` / `FKSolver` services are implemented in `kinematic_solver.cpp`, but no node currently instantiates `KinematicSolverService` in launch files.

---

## Robot model and simulation assets

- URDF/Xacro:
  - `hexapod_description/urdf/hexapod.urdf.xacro`
  - `hexapod_description/urdf/hexapod_primitives.urdf.xacro`
- 18 joints total (6 legs × coxa/femur/tibia)
- `ros2_control` configured with `gz_ros2_control/GazeboSimSystem`
- Controller config: `hexapod_description/config/controllers.yaml`
- Gazebo world: `hexapod_description/worlds/hexapod_world.sdf`
- Sensors in `hexapod_primitives.urdf.xacro` include camera, depth camera, IMU, magnetometer, and lidar

---

## Prerequisites

Verified from package manifests, launch files, and Dockerfiles:

- ROS 2 Jazzy environment for the main robot workspace (`hexapod_ws`)
- `colcon`, `rosdep`, `xacro`
- Gazebo + ROS GZ stack for simulation launches (`ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`)
- `joy` and `foxglove_bridge` for teleop/operator workflows

Optional:
- Foxglove Studio desktop app
- Docker / Docker Compose

---

## Build instructions

### 1) Build robot workspace

```bash
cd /home/runner/work/hexapod-ros2/hexapod-ros2/hexapod_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
```

### 2) Build operator workspace

```bash
cd /home/runner/work/hexapod-ros2/hexapod-ros2/operator_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
```

---

## Run instructions

### Fake hardware stack (fast local bringup)

```bash
cd /home/runner/work/hexapod-ros2/hexapod-ros2/hexapod_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch hexapod_bringup hexapod_fake.launch.py
```

### Full Gazebo simulation stack

```bash
cd /home/runner/work/hexapod-ros2/hexapod-ros2/hexapod_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch hexapod_bringup hexapod_simulation.launch.py
```

Optional launch args (from launch files):

```bash
ros2 launch hexapod_bringup hexapod_simulation.launch.py \
  bridge_vision:=true \
  front_depth_width:=640 \
  front_depth_height:=480 \
  front_depth_rate:=8
```

### Teleop-only launch (`joy_node` + `controller_teleop`)

```bash
ros2 launch hexapod_teleop controller_teleop.launch.py
```

### Operator launch (foxglove + joystick)

```bash
cd /home/runner/work/hexapod-ros2/hexapod-ros2/operator_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch operator_bringup operator.launch.py
```

### Convenience scripts

- `/home/runner/work/hexapod-ros2/hexapod-ros2/init_operator.sh`
  - sources overlays
  - verifies required packages
  - starts `joy_node` + `foxglove_bridge`
  - opens Foxglove Studio (`ws://localhost:8765`)
- `/home/runner/work/hexapod-ros2/hexapod-ros2/steamdeck/ros2_hexapod_launch.sh`
  - scans local subnet for Foxglove bridge on port `8765`
  - starts `joy_node`
  - opens Foxglove Studio in kiosk mode

---

## Configuration knobs

### Node parameters

- `hardware_interface_fake.publish_rate` (default `100.0`)
- `hardware_interface.publish_rate` (default `100.0`)

### Launch arguments (`hexapod_simulation.launch.py` / `gazebo.launch.py`)

- `bridge_vision` (`false` by default in simulation launch)
- `front_depth_width` (default `320` in simulation launch, passed to gazebo launch)
- `front_depth_height` (default `240` in simulation launch, passed to gazebo launch)
- `front_depth_rate` (default `6` in simulation launch, passed to gazebo launch)

---

## Hardware-facing assumptions in current code

From `hexapod_hardware_interface` handlers:

- Servo packet format built by `ServoHandler::buildPacket()`:
  - `[0xAA][length=72][18 float32 joint angles][xor checksum]`
- Sensor packet parse format expected by `SensorHandler::parseAndPublish()`:
  - header bytes then pitch/roll/current/voltage floats and leg contact bitfield
- `operator_bringup` launch expects joystick at `/dev/input/js1`

> Real transport to embedded hardware is not yet wired in `hardware_interface.cpp` (node currently initializes handlers but does not run I/O loop).

---

## Development workflow

### Docker (robot workspace)

Dev container with desktop + device/X11 passthrough:

```bash
cd /home/runner/work/hexapod-ros2/hexapod-ros2/hexapod_ws
docker compose -f docker-compose.dev.yml up --build
```

Minimal runtime container:

```bash
cd /home/runner/work/hexapod-ros2/hexapod-ros2/hexapod_ws
docker compose up --build
```

### Lint/test

The repository declares `ament_lint_auto` dependencies in package manifests, but there are no dedicated functional test targets in package `CMakeLists.txt` at present.

---

## Troubleshooting

- **`GetCapabilities service not available` warning in teleop nodes**
  - Start `gait_controller` first (e.g., via `hexapod_fake.launch.py` or `hexapod_simulation.launch.py`).
- **No joystick input in operator launch**
  - Confirm joystick device path matches `/dev/input/js1` or adjust `operator.launch.py`.
- **Foxglove cannot connect**
  - Verify bridge is up and port `8765` is reachable.
- **Gazebo cannot find resources/models**
  - `gazebo.launch.py` sets `GZ_SIM_RESOURCE_PATH`; ensure package is built and sourced.

---

## Known limitations / items to confirm

- `hexapod_control` is currently a placeholder package with empty source files.
- `hardware_interface` (non-fake) has no active polling/transport loop yet.
- `KinematicSolverService` is implemented but not launched by default.
- Several package manifests still contain `TODO` description/license fields.
- Operator environment files are inconsistent:
  - scripts/default setup expect **ROS Jazzy**
  - `operator_ws/dockerfile` currently uses **ROS Humble**

