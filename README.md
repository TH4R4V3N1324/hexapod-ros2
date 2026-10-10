# hexapod-ros2

ROS 2 control stack for a custom 6-legged (18-DOF) walking robot, plus a
separate operator workstation stack for remote teleoperation. The robot side
runs on **ROS 2 Jazzy** on a Raspberry Pi 5, drives Waveshare ST3215 serial
bus servos through `ros2_control`, and can also run in **Gazebo Harmonic**
for simulation and gait development. The operator side runs on **ROS 2
Humble** and bridges a game controller to the robot over Foxglove.

> **Note on `ROS2_ARCHITECTURE.md`:** the architecture doc at the repo root
> is out of date and does not reflect the current package layout, topics, or
> services. This README is the current source of truth; the old doc should
> be treated as historical only (or removed).

---

## Table of contents

- [Repository layout](#repository-layout)
- [System overview](#system-overview)
- [Hardware](#hardware)
- [Packages (`hexapod_ws`)](#packages-hexapod_ws)
- [ROS graph reference](#ros-graph-reference)
- [Custom interfaces](#custom-interfaces)
- [Gait controller internals](#gait-controller-internals)
- [Simulation (Gazebo Harmonic)](#simulation-gazebo-harmonic)
- [Operator stack (`operator_ws`)](#operator-stack-operator_ws)
- [Steam Deck client](#steam-deck-client)
- [Running it](#running-it)
- [Known gaps / TODO](#known-gaps--todo)

---

## Repository layout

```
hexapod-ros2/
├── hexapod_ws/              # ROS 2 Jazzy workspace — runs on the robot
│   ├── Dockerfile           # jazzy-ros-base (+ optional dev/sim tools)
│   ├── docker-compose.yml       # production container (robot)
│   ├── docker-compose.dev.yml   # dev container: X11, joystick, sim tools
│   └── src/
│       ├── hexapod_interfaces/        # custom msgs/srvs
│       ├── hexapod_description/       # URDF/xacro, meshes, worlds, RViz, Gazebo bridge
│       ├── hexapod_hardware_interface/ # ST3215 ros2_control plugin, BMI088 + INA226 drivers
│       ├── hexapod_gait_controller/   # kinematics, trajectories, gait state machine
│       ├── hexapod_teleop/            # keyboard + gamepad teleop
│       ├── hexapod_bringup/           # top-level launch files
│       └── hexapod_control/           # placeholder package (currently stubs, see below)
├── operator_ws/              # ROS 2 Humble workspace — runs on the operator PC
│   └── src/operator_bringup/ # joy_node + foxglove_bridge for the operator side
├── steamdeck/                 # Scripts to auto-discover the robot's Foxglove
│   │                           bridge on the LAN and launch Foxglove Studio
│   └── images/                 # Steam Deck / distrobox shortcut icons
├── init_operator.sh           # Native (non-Docker) operator launcher
└── ROS2_ARCHITECTURE.md       # Outdated — ignore, see this README instead
```

## System overview

The project is split into two independent ROS 2 workspaces that talk to each
other only over the network (DDS + a Foxglove WebSocket bridge), not by
sharing a workspace or overlay:

- **`hexapod_ws`** runs *on the robot* (or in simulation on a dev machine).
  It owns the gait controller, kinematics, hardware I/O, and description.
- **`operator_ws`** runs *on the operator's machine* (a laptop, or the Steam
  Deck). It runs `joy_node` to read a gamepad and `foxglove_bridge` so the
  operator can view the robot's Foxglove panels remotely. It does **not**
  run the teleop node itself — `controller_teleop` runs on the robot side
  and subscribes to `/joy`, which is published by the operator's `joy_node`
  and reaches the robot over shared DDS/ROS domain (or Foxglove, depending
  on network setup).

High-level data flow when driving the physical robot:

```
[Gamepad] → operator_ws: joy_node → /joy
                                        │  (DDS, same ROS_DOMAIN_ID)
                                        ▼
hexapod_ws: controller_teleop → /cmd_vel → gait_controller
                                              │
                                              ▼ (IK + trajectory generation)
                                        /joint_commands
                                              │
                                              ▼
                     joint_state_relay → hexapod_joint_controller (ros2_control)
                                              │
                                              ▼
                       ST3215System plugin → serial bus → 18× ST3215 servos

/joint_states, /imu/data, /power/left, /power/right → foxglove_bridge → operator's
                                                                      Foxglove Studio
```

## Hardware

- **6 legs**, 3 joints per leg (`coxa`, `femur`, `tibia`) → **18 DOF total**,
  named `leg{1-6}_{coxa,femur,tibia}_joint`.
- **Raspberry Pi 5** is the only compute. There is no microcontroller in
  between: the Pi talks to the servos and sensors directly.
- **Servos:** 18× Waveshare ST3215 serial bus servos (Feetech STS protocol,
  1 Mbaud) driven straight from the Pi's serial port. Two daughter boards,
  one per side of the body, break the bus out to the legs.
- **Power monitoring:** each daughter board carries an INA226 current/voltage
  sensor on the Pi's I2C bus, giving power draw for each side.
- **IMU:** a BMI088 (accelerometer + gyroscope) on the Pi's I2C bus.
- **Leg contact:** there are no contact switches. Contact is meant to be
  detected from servo load feedback, which the servo plugin publishes as
  joint effort on `/joint_states`.
- Onboard compute runs Foxglove Bridge so a remote operator can visualize
  joint states, IMU, and camera/lidar (in sim) without needing RViz on the
  robot itself.

## Packages (`hexapod_ws`)

| Package | Type | Purpose |
|---|---|---|
| `hexapod_interfaces` | msg/srv | Custom services and the few messages with no standard equivalent. Sensor data uses standard `sensor_msgs` types. |
| `hexapod_description` | C++ / xacro | URDF (visual + collision meshes) and a simplified "primitives" xacro used for Gazebo physics; RViz config; Gazebo Harmonic world; ros_gz bridge config; `joint_state_relay` node. |
| `hexapod_hardware_interface` | C++ | `ST3215System` `ros2_control` plugin for the bus servos, plus `bmi088_node` (IMU) and `ina226_node` (power monitors) I2C drivers. |
| `hexapod_gait_controller` | C++ | The core `gait_controller` node: inverse kinematics, Bezier swing trajectories, gait/mode state machine, and the `leg_axis_tester` diagnostic tool. |
| `hexapod_teleop` | C++ | `keyboard_teleop` (raw-terminal WASD-style control) and `controller_teleop` (gamepad `/joy` → `/cmd_vel`). |
| `hexapod_bringup` | launch only | Top-level launch files for the real robot, mock hardware and simulation. |
| `hexapod_control` | C++ (stub) | Placeholder package for a `behaviour_manager` and `config_manager`; both source files currently exist but are empty — see [Known gaps](#known-gaps--todo). |

### `hexapod_description`

- `urdf/hexapod.urdf.xacro` — full visual model (DAE meshes for body, coxa,
  femur, tibia) used for the real robot, mock hardware and RViz display.
  The `hardware` xacro arg picks the `ros2_control` hardware (`mock` by
  default).
- `urdf/hexapod_primitives.urdf.xacro` — a simplified collision-geometry
  version used for Gazebo physics, with a front depth camera
  (resolution/rate are launch arguments).
- `urdf/hexapod.ros2_control.xacro` — the `ros2_control` block shared by
  both models. `hardware` can be `gazebo` (`gz_ros2_control`), `mock`
  (`mock_components/GenericSystem`) or `st3215` (the real servos). For
  `st3215` it also holds the serial settings and each joint's `servo_id`,
  `direction` and `offset`. Servo IDs are `leg * 10 + joint` (11, 12, 13 for leg 1; 21, 22, 23 for leg 2; …);
  directions and offsets need calibrating on the real robot.
- `config/controllers.yaml` — `ros2_control` config: a
  `joint_state_broadcaster` and a single `hexapod_joint_controller`
  (`position_controllers/JointGroupPositionController`) covering all 18
  joints. It runs at 1000 Hz in Gazebo; `hexapod_robot.launch.py`
  overrides this to 100 Hz for the servo bus.
- `worlds/hexapod_world.sdf` — the Gazebo Harmonic world.
- `src/joint_state_relay.cpp` — bridges the gait controller's
  `sensor_msgs/JointState` on `joint_commands` into the flat
  `Float64MultiArray` that `hexapod_joint_controller` expects on
  `/hexapod_joint_controller/commands`, using `KeepLast(1)` QoS so stale
  commands never queue up under load.

### `hexapod_hardware_interface`

- **`ST3215System`** — a `ros2_control` `SystemInterface` plugin
  (`hexapod_hardware_interface/ST3215System`) for the 18 servos.
  - Each cycle, `write()` sends every goal position in one sync-write packet,
    and `read()` gets position, speed and load back with one sync read.
  - Exposes a position command and position, velocity and effort states per
    joint. Effort is the servo's load reading scaled by the `max_torque`
    param (2.94 N·m, the ST3215's 12 V stall torque). It's an estimate, but
    useful for detecting leg contact.
  - On activation it sets each goal to the servo's current position before
    enabling torque, so the legs don't jump. Deactivating disables torque.
  - Servos that don't reply are logged and skipped rather than stopping the
    controller.
- **`ST3215Bus`** — the serial driver: Feetech STS packets, sync read/write,
  and an `echo` option for adapters that loop TX back to RX.
- **`bmi088_node`** — reads the BMI088 accelerometer and gyroscope and
  publishes `sensor_msgs/Imu` on `imu/data_raw`. The BMI088 has no
  orientation output, so `imu_filter_madgwick` fuses it into orientation on
  `imu/data`.
- **`ina226_node`** — reads both INA226s and publishes `sensor_msgs/BatteryState`
  on `power/left` and `power/right`. Current is negative while drawing power,
  as `BatteryState` specifies. Percentage is a rough estimate from voltage.
- `config/sensors.yaml` — I2C bus, addresses, ranges, shunt resistances and
  battery voltage limits for the sensor nodes.

Both sensor nodes keep retrying if their device is missing, so a loose
connection recovers without a restart.

### `hexapod_gait_controller`

See [Gait controller internals](#gait-controller-internals) below.

### `hexapod_teleop`

- **`keyboard_teleop`** — raw-terminal (non-canonical, no echo) keyboard
  driver. Publishes `/cmd_vel` and drives gait/mode changes via the
  `set_gait`/`set_mode` services, cycling through whatever
  `get_capabilities` reports is available. Controls:

  | Key | Action |
  |---|---|
  | `w` / `s` | Increase / decrease forward linear velocity (`linear.x`) |
  | `a` / `d` | Increase / decrease lateral linear velocity (`linear.y`) |
  | `q` / `e` | Increase / decrease angular (yaw) velocity |
  | `x` | Zero all velocities (stop) |
  | `g` | Cycle to the next available gait (calls `set_gait`) |
  | `m` | Cycle to the next available mode (calls `set_mode`) |
  | `c` | Exit |

- **`controller_teleop`** — subscribes to `sensor_msgs/Joy` on `joy` (from
  `joy_node`, published by the operator side) and maps axis 1 → `linear.x`,
  axis 0 → `linear.y`, axis 3 → `angular.z`. Full stick is the gait
  controller's top speed from `get_capabilities`, which it requests in the
  background until the gait controller answers; joystick input is ignored
  until then. The gait/mode lists aren't used to switch gaits from the gamepad
  yet — see [Known gaps](#known-gaps--todo).

### `hexapod_bringup`

- `hexapod_robot.launch.py` — the real robot. Starts `robot_state_publisher`,
  the `ros2_control` controller manager with both controllers,
  `joint_state_relay`, `gait_controller`, `controller_teleop` and
  `foxglove_bridge`. With `hardware:=st3215` (the default) it also starts
  the BMI088, Madgwick filter and INA226 nodes. Launch args: `hardware`,
  `serial_port` (default `/dev/ttyAMA0`) and `update_rate` (default 100).
- `hexapod_fake.launch.py` — `hexapod_robot.launch.py` with `hardware:=mock`:
  the full control pipeline on mock hardware, for bench-testing gait logic
  without servos or Gazebo.
- `hexapod_simulation.launch.py` — includes `hexapod_description`'s
  `gazebo.launch.py`, then layers on `gait_controller`, `controller_teleop`,
  `foxglove_bridge`, and `rviz2`. Exposes `bridge_vision`,
  `front_depth_width/height/rate` as launch arguments to control whether
  the (bandwidth-heavy) camera/depth topics are bridged out of Gazebo.

### `hexapod_control` (stub)

`behaviour_manager.cpp` and `config_manager.cpp` exist as empty files with
no build target in `CMakeLists.txt`. This package is a placeholder for
future higher-level behaviour/config logic and currently does nothing.

## ROS graph reference

**Topics**

| Topic | Type | Publisher(s) | Subscriber(s) |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `keyboard_teleop`, `controller_teleop` | `gait_controller` |
| `/joint_states` | `sensor_msgs/JointState` (position, velocity, effort) | `joint_state_broadcaster` | `gait_controller`, `robot_state_publisher` |
| `/joint_commands` | `sensor_msgs/JointState` | `gait_controller`, `leg_axis_tester` | `joint_state_relay` |
| `/hexapod_joint_controller/commands` | `std_msgs/Float64MultiArray` | `joint_state_relay` | `hexapod_joint_controller` (ros2_control) |
| `/imu/data_raw` | `sensor_msgs/Imu` (no orientation) | `bmi088_node` | `imu_filter_madgwick` |
| `/imu/data` | `sensor_msgs/Imu` | `imu_filter_madgwick` (robot), Gazebo bridge (sim) | — |
| `/power/left`, `/power/right` | `sensor_msgs/BatteryState` | `ina226_node` | — |
| `/odometry/legs` | `nav_msgs/Odometry` | `gait_controller` | `ekf_filter_node` |
| `/odometry/filtered` | `nav_msgs/Odometry` | `ekf_filter_node` (also publishes TF `odom` → `base_link`) | — |
| `/joy` | `sensor_msgs/Joy` | `joy_node` (operator side) | `controller_teleop` |

**Services**

| Service | Type | Server | Client(s) |
|---|---|---|---|
| `/set_gait` | `hexapod_interfaces/SetGait` | `gait_controller` | `keyboard_teleop` |
| `/set_mode` | `hexapod_interfaces/SetMode` | `gait_controller` | `keyboard_teleop` |
| `/set_height` | `hexapod_interfaces/SetHeight` | `gait_controller` | — |
| `/get_capabilities` | `hexapod_interfaces/GetCapabilities` | `gait_controller` | `keyboard_teleop`, `controller_teleop` |
| `/ik_solver` | `hexapod_interfaces/IKSolver` | `kinematic_solver` (standalone service, not wired into `gait_controller`'s own launch) | — |
| `/fk_solver` | `hexapod_interfaces/FKSolver` | `kinematic_solver` | — |

## Custom interfaces

Defined in `hexapod_interfaces`. Sensor data uses standard `sensor_msgs`
types; these cover only what has no standard equivalent.

**Messages**
- `LocomotionOption` — `string name`, `uint8 id` (used to describe available gaits/modes).
- `HexapodState` — `mode`, `gait`, `body_height`, `gait_phase`, `is_moving`,
  `is_stable`. The file exists but isn't listed in `CMakeLists.txt`, so it
  isn't built.

**Services**
- `SetGait(uint8 gait) → (bool success, string message)` — `0=tripod, 1=ripple, 2=wave`.
- `SetMode(uint8 mode) → (bool success, string message)` — `0=normal, 1=strafe, 2=tilt, 3=config`.
- `SetHeight(float64 height) → (bool success, string message)`.
- `GetCapabilities() → (LocomotionOption[] modes, LocomotionOption[] gaits, float64 max_linear_vel, float64 max_angular_vel)`.
- `IKSolver(uint8 leg_index, geometry_msgs/Point target) → sensor_msgs/JointState`.
- `FKSolver(sensor_msgs/JointState) → geometry_msgs/Point foot_position`.

## Gait controller internals

`gait_controller` (in `hexapod_gait_controller`) is the central node. It
runs a 100 Hz ROS timer (`gaitTimerCallback`) and composes three helper
classes.

Gait timing is in seconds, not ticks. Each tick advances the gait by the time
since the last tick, so the loop rate can change without changing how the
robot walks. The timer follows simulation time when `use_sim_time` is set, as
it is in the Gazebo launch. Parameters:

| Parameter | Default | Meaning |
|---|---|---|
| `loop_rate` | 100.0 Hz | How often joint commands are sent; matches the controller manager's rate |
| `phase_duration` | 1.0 s | Length of one gait phase at low speed |
| `min_phase_duration` | 0.7 s | Shortest phase; sets each gait's top speed (0.17 m/s for tripod) |
| `max_foot_speed` | 1.5 m/s | Cap on how fast a foot target can move |
| `max_joint_speed` | 4.0 rad/s | The gait slows down so no joint exceeds this; keep it below the servos' top speed |
| `idle_return_delay` | 2.0 s | Idle time before the legs return to their start positions |
| `startup_duration` | 1.0 s | Time for the startup move (longer if `max_joint_speed` requires it) |

`/cmd_vel` is in real units (m/s and rad/s). Each phase is planned from the
commanded velocity: stance feet move back at the body's speed, and swing feet
land ahead of neutral by half the stroke the stance will sweep. Once that
stroke would exceed `MAX_STRIDE_LENGTH`, the phases get shorter, down to
`min_phase_duration`. Commands above the current gait's top speed are scaled
down. `get_capabilities` reports the current gait's top linear and angular
speeds. Normal mode doesn't strafe: it turns from `angular.z` (as Nav2 sends
it) plus `linear.y`, which is scaled so the gamepad's full linear range maps to
the full yaw range. Strafe mode uses `linear.y` to strafe and `angular.z` to
turn.

Directions follow ROS conventions for `/cmd_vel`: positive `linear.x` is
forward (towards the cameras), positive `linear.y` is left in strafe mode, and
positive yaw is anticlockwise. Turning rotates each foot about the body centre.
The URDF body frame follows REP 103 as well (+x forward, +y left, +z up). Legs
1–3 are on the right (front, middle, rear) and legs 4–6 on the left (rear,
middle, front).

To check directions:

```bash
# Unit tests of the foot motion maths (no simulation needed)
colcon test --packages-select hexapod_gait_controller && colcon test-result --verbose

# End-to-end in Gazebo: drives each direction and checks how the robot moved
ros2 launch hexapod_bringup hexapod_simulation.launch.py world:=empty_world
ros2 run hexapod_bringup sim_direction_check.py   # in a second terminal
```

Commands take effect every tick, not just at phase boundaries: stance feet move
with the current body velocity, swing feet re-aim their landing target (at up
to `max_target_speed`, 0.5 m/s), and the phase length follows the commanded
speed. A forward-to-backward reversal shows up in the body's motion within
0.06 s (it used to take ~1.9 s). Every foot stays within `MAX_STRIDE_LENGTH`
of its neutral position, and both ends of a swing are inside that circle, so
the whole swing is too. Neighbouring legs can't collide inside those circles:
the smallest gap between their segments is ~2.5 cm (`test_leg_clearance`).

The helper classes:

- **`GaitConfig`** — holds the three supported gaits (tripod / ripple /
  wave) and four modes (normal / strafe / tilt / config), per-leg home
  stance positions (each leg's start position is the same base pose rotated
  ±15° per pair around Z), current body height (default `-0.12` m), and
  gait/mode change requests.
- **`TrajectoryGenerator`** — turns a commanded `/cmd_vel` (linear X/Y +
  angular Z) into per-leg swing and stance targets. Swing trajectories are
  5-control-point Bezier splines (`SwingSpline`) with a configurable lift
  height (`LIFT_HEIGHT = 0.05` m); stance trajectories are straight-line
  splines (`LineSpline`). A `FrameConverter` helper handles the per-leg
  rotation into each leg's local frame.
- **`KinematicSolver`** — inverse kinematics used to convert Cartesian foot
  targets into joint angles for the gait loop, and also exposed directly as
  the standalone `ik_solver`/`fk_solver` services.

State machine, roughly:

1. **Startup** — waits for joint states from all 18 joints, then moves every
   leg in a straight line from its measured position to its start position
   (`startup()`) before gait execution begins.
2. **Walk** — while `/cmd_vel` is non-zero (beyond
   `CMD_VEL_CHANGE_THRESHOLD = 0.05`), continuously regenerates swing/stance
   trajectories per the active gait's leg groupings (tripod/ripple/wave
   phase patterns from `GaitConfig::getGaitConfig`) and steps through them
   (`PerformLegStep`).
3. **Idle return** — when velocity commands drop to zero, legs are eased
   back to a stable stance (`HandleIdleReturn`/`returnToStart`) rather than
   stopping mid-stride.

Each foot stays within `MAX_STRIDE_LENGTH = 0.06 m` of its neutral position.
Top speeds follow from that and `min_phase_duration`, and are what
`get_capabilities` reports back to teleop clients.

`leg_axis_tester` is a small diagnostic executable that shares the same
kinematics/trajectory libraries as `gait_controller`, publishing to
`joint_commands` directly — useful for exercising a single leg's axes
without the full gait state machine.

## Lidar (RPLIDAR A2M12)

- **Driver:** Slamtec's `rplidar_ros`, started by `hexapod_robot.launch.py`
  with the other sensors (`rplidar` section of `sensors.yaml`). The A2M12 runs
  at 256000 baud. Scans are published on `/lidar/scan` in `lidar_link`, the
  same as the simulated lidar.
- **Device name:** install `hexapod_ws/udev/99-rplidar.rules` on the Pi so the
  lidar always appears as `/dev/rplidar`. The robot container bind-mounts the
  Pi's `/dev` so it sees that name.
- **Mount:** `urdf/hexapod.lidar.xacro` sets the lidar's position for both the
  real and Gazebo models; change `lidar_z` when the body changes. `lidar_z` is
  the height of the scan plane above the hip joints, and it must clear the
  legs: the knees reach about 9 cm above the hips at the end of a stride
  (~12 cm with the leg's thickness). It's set to 15 cm. At 5 cm, 20% of each
  scan hit the robot's own legs and SLAM was off by 25 cm.
- **Simulation:** the Gazebo lidar matches the A2M12: 10 Hz, 1600 points per
  scan (0.225°), 0.2–12 m.
- **On first use**, check the scan's orientation in RViz: an object in front of
  the robot should appear at +x. If it's rotated, adjust the yaw in
  `lidar_joint`.

## Odometry

Two stages, both started by `hexapod_robot.launch.py` and `hexapod_simulation.launch.py`:

1. **Leg odometry** (`gait_controller`, published on `/odometry/legs` at 50 Hz).
   On every joint state, each foot's measured position comes from FK, and feet
   that aren't swinging are treated as planted. The body motion is the rigid
   2D transform that best explains how those planted feet moved relative to
   the body (`LegOdometry`, unit-tested in `test_leg_odometry`).
2. **EKF** (`robot_localization`, `hexapod_bringup/config/ekf.yaml`). Fuses the
   leg velocities with the IMU's yaw rate and publishes `/odometry/filtered`
   and the `odom` → `base_link` transform. The feet slip most when turning,
   and the gyro corrects that. IMU orientation isn't fused: without a
   magnetometer its yaw drifts.

In Gazebo, after driving forward, turning 143° and driving forward again,
leg odometry alone was 5.7 cm and 12° off; the fused odometry was 2.4 cm and
0° off.

## SLAM

`slam_toolbox` (online async) builds a 2D map from `/lidar/scan` and publishes
the `map` → `odom` correction, leaving `odom` → `base_link` to the EKF. Enable
it with `slam:=true` on `hexapod_simulation.launch.py` or
`hexapod_robot.launch.py`. Settings are in `hexapod_bringup/config/slam_toolbox.yaml`,
based on the package defaults but adding scans every 0.2 m or 0.2 rad (the
robot is slow) and using the A2M12's 0.2–12 m range.

In the house world, after driving a 0.6 m square, SLAM was 1.8 cm and 0.2°
from the true pose; odometry alone was 15.5 cm off.

## Navigation (Nav2)

Nav2 plans and follows paths on the SLAM map, publishing `/cmd_vel`. Enable it
with `nav:=true` alongside `slam:=true`:

```bash
ros2 launch hexapod_bringup hexapod_simulation.launch.py slam:=true nav:=true
```

Then send goals with RViz's "2D Goal Pose" or the `navigate_to_pose` action.
Settings are in `hexapod_bringup/config/nav2_params.yaml`, based on
nav2_bringup's defaults:

- **Robot:** 0.32 m radius (the legs plus stride), 0.45 m inflation so 1 m
  doorways stay passable.
- **Omnidirectional:** with `nav:=true` the gait starts in strafe mode
  (`initial_mode` parameter), where `/cmd_vel` follows the standard holonomic
  convention, and MPPI uses its `Omni` motion model. The robot walks sideways
  or diagonally instead of turning first, and only turns for the goal heading.
- **Speeds:** MPPI and the velocity smoother are capped just under the gait's
  limits (0.15 m/s forward and sideways, 0.6 rad/s), as are recovery spins.
- **MPPI acceleration limits** are left at Nav2's defaults. MPPI starts each
  cycle from the measured speed, and the gait takes up to ~1 s to respond, so
  low limits stop it ever speeding up; the velocity smoother ramps gently
  instead.
- **Timing:** the EKF stamps its output at the current time, and SLAM stamps
  `map` → `odom` 1 s ahead (more than its 0.5 s scan interval), so Nav2 never
  looks up a transform that hasn't arrived yet.

In the house world, Nav2 reached a goal across the room (ending turned round)
0 cm from target in 21 s, and a goal through a doorway into the next room 4 cm
from target in 32 s, with no recoveries.

## Simulation (Gazebo Harmonic)

`hexapod_description/launch/gazebo.launch.py`:

- Launches Gazebo Harmonic (`gz sim`) with `hexapod_world.sdf`, using the
  **primitives** xacro (simplified collision geometry, not the visual mesh
  model) so physics stays cheap.
- Sets `GZ_SIM_SYSTEM_PLUGIN_PATH` / `GZ_SIM_RESOURCE_PATH` so Gazebo can
  resolve `model://hexapod_description/...` mesh URIs.
- Spawns the robot via `ros_gz_sim`'s `create` (the Harmonic replacement for
  the old `gazebo_ros spawn_entity.py`), then sequences
  `joint_state_broadcaster` → `hexapod_joint_controller` spawning via
  `OnProcessExit` event handlers once the model is actually in the world.
- Bridges core sim topics unconditionally (`/clock`, `/imu` remapped to
  `/imu/data` to match the real robot, `/imu/mag`,
  front depth image/camera_info/points, `/lidar/scan`) and gates the
  higher-bandwidth main camera + full-res front depth image behind the
  `bridge_vision` launch argument, to avoid saturating Foxglove/RViz by
  default.
- `hexapod_simulation.launch.py` wraps this together with `gait_controller`,
  `controller_teleop`, `foxglove_bridge`, and RViz for a one-command sim
  bring-up.

This is a migration from an earlier Gazebo Classic setup (per the
`gazebo.launch.py` comments) — the "explore SLAM, Nav2, Gazebo simulation"
work referenced in the project's broader roadmap builds on this
Harmonic-based sim.

## Operator stack (`operator_ws`)

A separate, **ROS 2 Humble** workspace (note the distro mismatch with
`hexapod_ws`'s Jazzy — see [Known gaps](#known-gaps--todo)) containing a
single package, `operator_bringup`, which launches:

- **`joy_node`** — reads `/dev/input/js1`, 0.05 deadzone, 20 Hz autorepeat.
- **`foxglove_bridge_node`** — lets the operator's Foxglove Studio connect
  back to the robot's own Foxglove bridge (or vice versa, depending on
  which side initiates), for visualizing joint states, IMU, sim camera
  feeds, etc.

Two ways to run it:

- **Docker** (`operator_ws/dockerfile` + `docker-compose.yml`): builds on
  `osrf/ros:humble-desktop-full`, installs `joy` + `foxglove_bridge`, and
  runs `operator.launch.py` on container start. `initHexOperator.sh` brings
  the container up, waits for the Foxglove WebSocket (port 8765) to come
  alive, then opens Foxglove via a deep link (`foxglove://open?...`).
- **Native** (`init_operator.sh`, repo root): sources ROS + both workspaces
  if built locally, verifies `foxglove_bridge`, `joy`, `hexapod_interfaces`,
  and `hexapod_description` are all resolvable in the active overlay
  (installing `foxglove_bridge` via `apt` if missing), then starts
  `joy_node`, `foxglove_bridge`, and `foxglove-studio` directly on the host.

## Steam Deck client

`steamdeck/` scripts turn a Steam Deck (via `distrobox`) into a portable
Foxglove viewing/joystick client:

- **`hexapod_launch.sh`** — the Steam-launchable entry point. Sets up the
  Deck's Wayland/X display environment, then runs `ros2_hexapod_launch.sh`
  inside a `distrobox` Ubuntu container.
- **`ros2_hexapod_launch.sh`** — scans the local `192.168.x.x/24` subnet for
  a host with port `8765` open (i.e. an active `foxglove_bridge`), starts
  `joy_node`, and opens `foxglove-studio` in kiosk mode pointed at the
  discovered bridge over a `foxglove://` deep link.
- `images/` — icon/header art used for the Steam Deck game-mode shortcut.

This gives a no-typing, walk-up client: power on the Deck, launch the
shortcut, and it finds the robot's Foxglove bridge and starts streaming a
joystick automatically.

## Running it

**On the robot / a sim-capable dev machine (`hexapod_ws`):**

```bash
cd hexapod_ws
docker compose -f docker-compose.dev.yml up --build   # dev image w/ Gazebo, joy, terminator
# inside the container:
colcon build --symlink-install
source install/setup.bash

# Bench-test gait logic on mock hardware (no Gazebo, no real servos):
ros2 launch hexapod_bringup hexapod_fake.launch.py

# On the robot, with the servos and sensors connected:
ros2 launch hexapod_bringup hexapod_robot.launch.py serial_port:=/dev/ttyAMA0

# Or run in Gazebo Harmonic:
ros2 launch hexapod_bringup hexapod_simulation.launch.py bridge_vision:=true
```

The plain `docker-compose.yml` (no dev tools, `INSTALL_DEV_TOOLS=false`)
builds a lean runtime image whose default `CMD` runs
`hexapod_fake.launch.py` — intended for the real robot's onboard computer
where Gazebo/RViz/X11 aren't needed. Switch it to `hexapod_robot.launch.py`
once the servos are wired up. The container runs `privileged`, so the serial
port and `/dev/i2c-1` are visible; I2C and the UART still have to be enabled
on the Pi itself.

**On the operator machine (`operator_ws`):**

```bash
cd operator_ws
./initHexOperator.sh     # Docker route
# — or, natively, from the repo root: —
./init_operator.sh
```

Both workspaces need to share a `ROS_DOMAIN_ID` (or otherwise be bridged)
for `/joy` published on the operator side to reach `controller_teleop` on
the robot side.

## Known gaps / TODO

Noted here rather than glossed over, since they affect how far you can take
this out of the box:

- **The real hardware drivers are untested on hardware.** `ST3215System`
  has been run against a simulated servo bus, and the sensor nodes have
  only been run without devices attached.
- **Servo calibration isn't done.** Each joint's `direction` and `offset`
  in `hexapod.ros2_control.xacro` default to 1 and 0.
- **Leg contact detection isn't implemented.** The servo load is published
  as joint effort, but nothing uses it yet.
- **`hexapod_control` is an empty stub.** `behaviour_manager.cpp` and
  `config_manager.cpp` are 0-byte files with no build targets.
- **ROS distro mismatch.** `hexapod_ws` targets Jazzy; `operator_ws`
  targets Humble. This works today because they only interact over
  DDS/network, but it's worth keeping in mind if message definitions ever
  need to cross that boundary directly.
- **Package descriptions/licenses are all `TODO`** in every `package.xml`.
- All `package.xml` maintainer emails are real personal addresses — worth
  reviewing before making the repo public.
