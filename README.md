# hexapod-ros2

ROS 2 control stack for a custom 6-legged (18-DOF) walking robot, plus a
separate operator workstation stack for remote teleoperation. The robot side
runs on **ROS 2 Jazzy** on a Raspberry Pi 5, drives Waveshare ST3215 serial
bus servos through `ros2_control`, and can also run in **Gazebo Harmonic**
for simulation and gait development. The operator side also runs on **ROS 2
Jazzy** and bridges a game controller to the robot over Foxglove.

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
│       ├── hexapod_hardware_interface/ # ST3215 ros2_control plugin, ISM330DHCX + INA226 drivers
│       ├── hexapod_gait_controller/   # kinematics, trajectories, gait state machine
│       ├── hexapod_teleop/            # keyboard + gamepad teleop
│       ├── hexapod_bringup/           # top-level launch files
│       └── hexapod_control/           # placeholder package (currently stubs, see below)
├── operator_ws/              # ROS 2 Jazzy workspace — runs on the operator PC
│   └── src/operator_bringup/ # game_controller_node + foxglove_bridge for the operator side
├── foxglove_panel/            # "Hexapod" Foxglove panel: attitude, feet, power, controls
├── pi/                        # Robot Pi setup: I2C, hotspot, start-at-boot service
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
  Deck). It runs `game_controller_node` to read a gamepad and `foxglove_bridge` so the
  operator can view the robot's Foxglove panels remotely. It does **not**
  run the teleop node itself — `controller_teleop` runs on the robot side
  and subscribes to `/joy`, which is published by the operator's `game_controller_node`
  and reaches the robot over shared DDS/ROS domain (or Foxglove, depending
  on network setup).

High-level data flow when driving the physical robot:

```
[Gamepad] → operator_ws: game_controller_node → /joy
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
- **IMU:** an Adafruit ISM330DHCX board (accelerometer + gyroscope) on the
  Pi's I2C bus, at address 0x6A.
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
| `hexapod_hardware_interface` | C++ | `ST3215System` `ros2_control` plugin for the bus servos, plus `ism330dhcx_node` (IMU) and `ina226_node` (power monitors) I2C drivers. |
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
- **`ism330dhcx_node`** — reads the ISM330DHCX accelerometer and gyroscope
  and publishes `sensor_msgs/Imu` on `imu/data_raw`. The ISM330DHCX has no
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
  `game_controller_node`, published by the operator side) and publishes both `/cmd_vel`
  and `/body_pose`; the gait controller uses whichever suits its mode. The
  face buttons read a feature's current state from the gait controller and
  call its `set_<feature>` service with the opposite, so they never undo a
  change made another way. Full
  stick is the gait controller's limit from `get_capabilities`, which it
  requests in the background until the gait controller answers; joystick
  input is ignored until then. Axis and button numbers are parameters,
  defaulting to the SDL game controller layout that `joy`'s
  `game_controller_node` reports for any supported gamepad (Xbox names
  first, PlayStation in brackets). The operator launch and the Steam Deck
  script run that node: plain `joy_node` reports each controller's raw order,
  which for Xbox-style controls like the Steam Deck's puts the triggers on
  axes 2 and 5 (a DualSense's raw order happens to match):

  | Input | Walking (normal / strafe) | Tilt mode |
  |---|---|---|
  | Left stick (axes 0, 1) | forward/back, left/right | shift the body forward/back, left/right |
  | Right stick X (axis 2) | turn | roll |
  | Right stick Y (axis 3) | — | pitch (forward lowers the nose) |
  | Triggers LT/RT [L2/R2] (axes 4, 5) | — | yaw left / right |
  | D-pad up/down (buttons 11, 12) | — | raise / lower the body |
  | Start [Options] (button 6) | next mode | next mode |
  | Back [Create] (button 4) | next gait | next gait |
  | A [Cross] (button 0) | terrain adaptation on/off | terrain adaptation on/off |
  | B [Circle] (button 1) | walk leveling on/off | walk leveling on/off |
  | X [Square] (button 2) | load balancing on/off | load balancing on/off |
  | Y [Triangle] (button 3) | tilt leveling on/off | tilt leveling on/off |

  Drivers disagree on what a released trigger reads (0, 1 or -1), so each
  trigger's first reading is taken as released. To check another
  controller's numbering, run `ros2 topic echo /joy` and move each control.

### `hexapod_bringup`

- `hexapod_robot.launch.py` — the real robot. Starts `robot_state_publisher`,
  the `ros2_control` controller manager with both controllers,
  `joint_state_relay`, `gait_controller`, `controller_teleop` and
  `foxglove_bridge`. With `hardware:=st3215` (the default) it also starts
  the ISM330DHCX, Madgwick filter and INA226 nodes. Launch args: `hardware`,
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
| `/imu/data_raw` | `sensor_msgs/Imu` (no orientation) | `ism330dhcx_node` | `imu_filter_madgwick` |
| `/imu/data` | `sensor_msgs/Imu` | `imu_filter_madgwick` (robot), Gazebo bridge (sim) | — |
| `/power/left`, `/power/right` | `sensor_msgs/BatteryState` | `ina226_node` | — |
| `/odometry/legs` | `nav_msgs/Odometry` | `gait_controller` | `ekf_filter_node` |
| `/odometry/filtered` | `nav_msgs/Odometry` | `ekf_filter_node` (also publishes TF `odom` → `base_link`) | — |
| `/foot_force/leg1` … `/foot_force/leg6` | `geometry_msgs/WrenchStamped` (ground force on each foot, body frame) | `gait_controller` | — |
| `/terrain_map` | `sensor_msgs/PointCloud2` (height map cells, odom frame) | `gait_controller` | — |
| `/front_depth/points` | `sensor_msgs/PointCloud2` | Gazebo bridge (sim) | `gait_controller`, Nav2 costmaps |
| `/body_pose` | `geometry_msgs/Pose` (body shift and roll/pitch/yaw, used in tilt mode) | `controller_teleop` | `gait_controller` |
| `/joy` | `sensor_msgs/Joy` | `game_controller_node` (operator side) | `controller_teleop` |

**Services**

| Service | Type | Server | Client(s) |
|---|---|---|---|
| `/set_gait` | `hexapod_interfaces/SetGait` | `gait_controller` | `keyboard_teleop`, `controller_teleop` |
| `/set_mode` | `hexapod_interfaces/SetMode` | `gait_controller` | `keyboard_teleop`, `controller_teleop` |
| `/set_height` | `hexapod_interfaces/SetHeight` | `gait_controller` | — |
| `/get_capabilities` | `hexapod_interfaces/GetCapabilities` | `gait_controller` | `keyboard_teleop`, `controller_teleop` |
| `/set_terrain_adaptation`, `/set_terrain_map`, `/set_walk_leveling`, `/set_tilt_leveling`, `/set_tilt_lock_yaw`, `/set_load_balancing` | `std_srvs/SetBool` | `gait_controller` | — |
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
- `SetMode(uint8 mode) → (bool success, string message)` — `0=normal, 1=strafe, 2=tilt`; other values are rejected.
- `SetHeight(float64 height) → (bool success, string message)`. `height` is
  the feet's z below the hips, so negative: `-0.12` for a 12 cm body. The body
  glides there at `height_rate` (0.03 m/s), every foot moving together, whether
  standing, walking or in tilt mode. Heights outside `min_height`–`max_height`
  (0.06–0.16 m) are refused. It used to move the foot targets at once, and
  while standing still only half the feet followed: the others waited, then
  jumped to the new height in the first step of walking.
- `GetCapabilities() → (LocomotionOption[] modes, LocomotionOption[] gaits, float64 max_linear_vel, float64 max_angular_vel, float64 max_tilt_angle, float64 max_tilt_translation, uint8 current_mode, uint8 current_gait)`.
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

A gait change requested with `set_gait` applies at the start of the next step,
when every foot is planted.

### Foot forces, contact and load balancing

On every joint state, each leg's three joint efforts are turned into the force
the ground puts on its foot, through the leg's Jacobian (`KinematicSolver::groundForce`,
`test_foot_force`). Each leg learns its own zero reading mid-swing, when the
foot is certainly in the air, so the leg's weight doesn't count as load. The
forces are published in the body frame on `foot_force/leg1` … `foot_force/leg6`
(`geometry_msgs/WrenchStamped`), and a foot is in contact above
`contact_force_on` (2 N) until it drops below `contact_force_off` (1 N).

With stiff legs on hard ground, sub-millimetre differences decide which feet
carry the weight. In simulation the robot often stood on one tripod after
stopping, with the other three legs carrying nothing. Load balancing
(`load_balancing`, on by default) raises each planted foot carrying more than
its share and lowers each carrying less, by up to `load_balance_max` (2 cm).
It only acts with four or more feet down, since three feet carry whatever the
centre of mass puts on them. Each adjustment has any part that would raise or
tilt the body removed, so it doesn't fight leveling. After stopping, the
spread between the most and least loaded foot dropped from 7–8 N to 1–4 N
(each foot's even share is ~3.2 N).

`load_balance_gain` (m per newton-second) depends on how stiff the legs and
ground are: 0.0001 suits the sim's rigid contacts, and the real robot's sprung
tibias will want more. The contact thresholds come from the sim too and will
need checking against the ST3215's load readings, which are a current estimate
rather than a torque sensor.

### Uneven ground

With `terrain_adaptation` on (the default), each foot remembers how far its
ground is above or below nominal, stays there in stance and aims for it on its
next swing. Where it puts its foot comes from two sources:

- **The front depth camera**, only with `terrain_map` on (off by default, or
  `/set_terrain_map` while running). In the outdoor scenarios it caused more
  early touchdowns (30 against 20) and stumbles (346 against 248, before the
  false stumbles below were fixed) than
  the feet alone, and walked worse on hills: 1.62 m against 2.29 m over the
  15° hill. It only helped on the slabs (1.60 m against 1.53 m). Its points (`depth_topic`, every
  `depth_stride`-th pixel, at most `map_update_rate` frames a second) build a
  height map of 2 cm cells in `map_frame` (odom), kept within `map_radius` of
  the robot and published on `terrain_map` (`sensor_msgs/PointCloud2`) for
  RViz. Points within 6 cm of the robot's own feet are dropped, since the
  front feet come into view at the end of their stride. The camera can't see the
  ground under the front feet, so the map remembers what it saw on the way.
  Each swing:
  - takes its landing height from the map;
  - moves its landing spot up to `foothold_search` (3 cm) to keep
    `foothold_edge` (2.5 cm) of flat ground around it, so feet don't come down
    on an edge;
  - lifts high enough to clear the highest ground on the way by
    `step_clearance`.

  The map only changes a landing height by more than `map_height_deadband`
  (1 cm): smaller differences are mostly the body's pitch skewing the points,
  and following them rocked the body. Nothing is mapped until the startup
  sequence is done: before that TF has the body at the wrong height. Depth
  frames are processed on the gait loop's thread, so each one is kept cheap.
  Processing every frame in full delayed gait ticks by up to 37 ms and made
  the legs stutter.
- **Foot forces**, for ground the camera hasn't seen, such as walking
  sideways or backwards:
  - A swinging foot that lands more than 1 cm above where it was aiming,
    pushing up rather than back for 3 ticks in a row, stops there.
  - One that finishes its swing with no force and is still unloaded 0.15 s
    later reaches down at `seek_speed` until it finds the ground. Without
    the wait, feet that had just landed (before the weight came onto them)
    were pushed down a few mm every step, which slowly tipped the body.
  - One that is hit from the side mid-swing (above `stumble_force` for 3
    ticks, while not bearing weight) is lifted a further `stumble_lift` and
    carries on over the edge. Without those two conditions, normal landings
    (pushed back and up at once) and the filtered push-off just after lift-off
    counted as stumbles, about 38 times in a 2 m walk on flat ground, and each
    one popped the foot up as it landed. Now flat ground gives none, while the
    slabs, rocks and hills still give 7–15 a run.

Ground heights are limited to `max_step_height` (4 cm) either way. The body
rises or falls at `body_height_rate` so the feet's average ground is back at
nominal; slopes are left to leveling. Load balancing only runs while standing
still; once walking starts its adjustments become part of each foot's ground
height. Without effort data (mock hardware) all of this is off.

The swing now lifts first, moves across high, and comes down nearly
vertically. Its three middle control points are all `step_height` (0.06 m)
above the path, so the foot peaks at ~5 cm. The old profile dipped mid-swing
and clipped slab edges. A higher step clears more but walks slower: at full
command the robot made 0.127 m/s with a ~4 cm peak, 0.124 m/s at 0.06 and
0.109 m/s at 0.075.

On the `rough_terrain` world (a 2.5 cm slab under the left feet, a 3 cm slab
under the right, then a 2 cm step), walking 1.4–1.5 m:

| | Adaptation off | On |
|---|---|---|
| Distance walked | 136 cm | 151 cm |
| Sideways drift / turn | +9.1 cm / +7.2° | +0.2 cm / −0.9° |
| Body roll, mean / peak | 0.7° / 5.7° | 0.3° / 2.3° |
| Body pitch, mean / peak | 1.3° / 7.4° | 0.4° / 2.7° |
| Heaviest foot load | 23 N | 16 N |

On flat ground it walks as steadily as with adaptation off (body roll and
pitch varying by ~0.1°).

### Leveling while walking

In normal and strafe mode the IMU keeps the body level on slopes
(`walk_leveling`, on by default). A correction built up from the measured roll
and pitch rotates the body about the ground point under it, which keeps the
body above the middle of its feet and moves the feet less than 7 mm sideways
(`test_leg_clearance` covers stride circles that much wider). The gait itself
still works in the level frame, and leg odometry has the tilt taken out.

| Parameter | Default | |
|---|---|---|
| `walk_leveling` | true | hold the body level while walking |
| `walk_level_gain` | 2.0 /s | slow, to follow slopes rather than each step |
| `walk_max_angle` | 0.15 rad | largest correction; it is also cut back if a foot would be out of reach |

In simulation, on a deck tilted 5.7° (roll or pitch), the body stands at 0.0°
and averages 0.0–0.1° while walking (5.7° with leveling off), and walks as
straight and as far as on level ground. Tilt mode takes over the levelled pose
when it starts and hands it back when it ends, so switching modes on a slope
doesn't jolt the feet.

### Tilt mode

Tilt mode (mode 2) stands still with the feet planted and moves the body
instead: `/body_pose` (`geometry_msgs/Pose`) sets the body's shift and its
roll, pitch and yaw. On entry the legs step back to their start positions;
on leaving, the body eases back to neutral before walking resumes.

With `tilt_leveling` on (the default), the IMU holds the body's roll and pitch
in space: a correction built up from the difference between the requested and
measured angles is added to the pose, so the body stays level (or at the
requested angle) while whatever it stands on tilts underneath it. The IMU
can't measure position, so the shift is always relative to the feet.

Each tick the pose moves at most a small step from the last one, so the feet
never jump. A pose the legs can't reach safely is cut back: yaw eases off
first, then the whole step only goes as far as is safe. Every foot stays
inside its stride circle and within reach (`test_leg_clearance` covers feet
10 cm above and below standing height).

| Parameter | Default | |
|---|---|---|
| `tilt_max_angle` | 0.3 rad | largest roll, pitch or yaw |
| `tilt_max_translation` | 0.04 m | largest shift along each axis |
| `tilt_angle_rate`, `tilt_translation_rate` | 1.0 rad/s, 0.1 m/s | how fast the requested pose is followed |
| `tilt_leveling` | true | hold roll and pitch with the IMU |
| `tilt_level_gain` | 8.0 /s | how fast the leveling responds |
| `tilt_lock_yaw` | false | also hold the heading; off because the IMU's yaw drifts without a magnetometer and holding it uses up the stride |

The tilt parameters can be changed while running (`ros2 param set`).

Each on/off feature also has a `std_srvs/SetBool` service that sets its
parameter, so the two always agree:

```bash
ros2 service call /set_terrain_adaptation std_srvs/srv/SetBool "{data: false}"
```

| Service | Feature |
|---|---|
| `/set_terrain_adaptation` | follow uneven ground with the feet |
| `/set_terrain_map` | use the depth camera's height map for footholds and step heights |
| `/set_walk_leveling` | keep the body level while walking |
| `/set_tilt_leveling` | hold the body's roll and pitch in tilt mode |
| `/set_tilt_lock_yaw` | also hold the heading in tilt mode |
| `/set_load_balancing` | spread the weight evenly while standing |

To try it on a moving platform:

```bash
ros2 launch hexapod_bringup hexapod_simulation.launch.py world:=tilt_platform
# then switch to tilt mode and move the deck (radians)
ros2 service call /set_mode hexapod_interfaces/srv/SetMode "{mode: 2}"
gz topic -t /platform/roll -m gz.msgs.Double -p 'data: 0.15'   # also /platform/pitch, /platform/yaw
```

In simulation the commanded poses are reached accurately (0.15 rad roll or
pitch gives 8.6°, a 3 cm raise gives 3.0 cm), and with the deck rolling ±6.9°
the body stays within 1.5° (gain 8). A deck held at 8.6° is levelled to 0.0°.

The helper classes:

- **`GaitConfig`** — holds the three supported gaits (tripod / ripple /
  wave) and three modes (normal / strafe / tilt), per-leg home
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

## Cameras

Two front-facing cameras, the Arducam ToF below the Pi camera, both mounted in
`urdf/hexapod.cameras.xacro` (shared by the real and Gazebo models):

- **Arducam ToF camera (B0410)** → `front_depth_camera_link`, point cloud on
  `/front_depth/points`. 240×180 depth, 70° diagonal FOV (58.5° horizontal),
  4 m in far mode, tilted 10° down to see low obstacles closer to the feet.
  The simulated camera matches it (15 Hz in sim to keep Gazebo's load down;
  the real camera runs at 30 fps). No ROS 2 driver is packaged for it yet; a
  node publishing `PointCloud2` on the same topic is needed on the real robot.
- **Raspberry Pi Camera Module 3** (standard lens, 66° horizontal FOV) →
  `camera_link`, image on `/camera/image`. Simulated at 640×360.

The front legs stay outside the ToF camera's view (they're ~58° off-axis, the
camera sees ±29°).

## Odometry


Two stages, both started by `hexapod_robot.launch.py` and `hexapod_simulation.launch.py`:

1. **Leg odometry** (`gait_controller`, published on `/odometry/legs` at 50 Hz).
   On every joint state, each foot's measured position comes from FK, and feet
   that aren't swinging and are bearing weight are treated as planted (without
   effort data, as with mock hardware, every non-swinging foot counts). The body motion is the rigid
   2D transform that best explains how those planted feet moved relative to
   the body (`LegOdometry`, unit-tested in `test_leg_odometry`).
   When the net horizontal force on the feet passes `blocked_force_ratio` (0.3)
   of their vertical force, the robot is pushing against something and its feet
   are scrubbing, so only the turn is counted. In simulation, walking into a
   wall for 6 s used to add ~50 cm of travel that never happened; now it adds
   ~2 cm (the time it takes to notice). Sliding along a wall isn't counted
   while blocked.
   Joint states that share a timestamp with the previous one are skipped and
   their motion counted with the next: in simulation ~30% of them did, and
   dropping their motion had made odometry read ~20% short. Walking 50 cm in
   simulation, odometry is now within 4–6%.
2. **EKF** (`robot_localization`, `hexapod_bringup/config/ekf.yaml`). Fuses the
   leg velocities with the IMU's yaw rate and publishes `/odometry/filtered`
   and the `odom` → `base_footprint` transform. The feet slip most when turning,
   and the gyro corrects that. IMU orientation isn't fused: without a
   magnetometer its yaw drifts.

**`base_footprint`** is the point on the floor directly below `base_link`.
The gait controller publishes `base_footprint` → `base_link` every tick: the
body height, plus the body's roll and pitch from the IMU. Odometry, SLAM and
Nav2 all use `base_footprint` as the robot's base. So z = 0 in `odom` is the
floor, which lets the costmaps tell the floor from low obstacles. The tilt
matters: the body rocks by up to ~0.8° while walking, which without it put
floor 3.5 m away 5 cm too high and marked phantom obstacles across the room.

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
- **Omnidirectional, preferring forwards:** with `nav:=true` the gait starts
  in strafe mode (`initial_mode` parameter), where `/cmd_vel` follows the
  standard holonomic convention, and MPPI uses its `Omni` motion model.
  `PathAngleCritic` (forward mode, 30°) and `PreferForwardCritic` make it face
  along the path, so the forward-facing depth camera sees where it's going;
  sideways steps remain for corrections. Walking backwards past low obstacles
  the camera couldn't see is what once got the robot stuck.
- **Global costmap** is a 20 × 20 m rolling window rather than sized to the
  SLAM map. The SLAM map grows as the robot explores, and every resize wiped
  the obstacle layer, so low obstacles vanished once the camera looked away.
- **Speeds:** MPPI and the velocity smoother are capped just under the gait's
  limits (0.15 m/s forward and sideways, 0.6 rad/s), as are recovery spins.
- **MPPI acceleration limits** are left at Nav2's defaults. MPPI starts each
  cycle from the measured speed, and the gait takes up to ~1 s to respond, so
  low limits stop it ever speeding up; the velocity smoother ramps gently
  instead.
- **Low obstacles:** both costmaps take the ToF point cloud as well as the
  lidar, marking anything from 3 cm above the floor to 35 cm (just above the
  lidar's scan plane), and ignoring points within 0.3 m of the camera. Both are
  3D voxel layers, so the lidar, which passes over low obstacles, only clears
  at its own height and doesn't erase what the camera marked below it.
- **Timing:** the EKF stamps its output at the current time, and SLAM stamps
  `map` → `odom` 1 s ahead (more than its 0.5 s scan interval), so Nav2 never
  looks up a transform that hasn't arrived yet.

In the `low_obstacles` world (a walled room with 6, 10, 12 and 20 cm objects,
all below the lidar), Nav2 went through the objects and back, staying at least
0.51 m from each, facing its direction of travel 63–70% of the time, and ending
1–9 cm from the goals.

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
  front depth image/camera_info/points, `/lidar/scan`, and the robot's true
  pose from Gazebo on `/ground_truth/odom`) and gates the
  higher-bandwidth main camera + full-res front depth image behind the
  `bridge_vision` launch argument, to avoid saturating Foxglove/RViz by
  default.
- `hexapod_simulation.launch.py` wraps this together with `gait_controller`,
  `controller_teleop`, `foxglove_bridge`, and RViz for a one-command sim
  bring-up. `gui:=false` runs Gazebo without its window, `rviz:=false`
  skips RViz, and `spawn_x/y/z/yaw` set where the robot starts.

### Outdoor scenarios

Three worlds approximate outdoor ground, generated by
`hexapod_description/scripts/generate_outdoor_worlds.py` (`--seed` changes
the rock field):

- `outdoor_slopes` — hills of 5°, 10°, 15° and 20° (up, across the top, down),
  and 10° and 15° side slopes to walk along.
- `outdoor_rocks` — rocks mostly 1–4 cm high with a few up to 7 cm, over
  gentle bumps.
- `outdoor_low_friction` — flat lanes of dry ground (mu 1.0), wet grass
  (0.6), mud (0.4) and wet tile (0.25), and a 10° hill of mud.

`scenario_runner.py` walks the robot through 16 scenarios across these
worlds, starting a fresh simulation for each, and scores each run against
Gazebo's true pose: progress relative to the flat baseline, sideways drift,
odometry error, body tilt (mean and peak) and falls. Results go to
`/tmp/hexapod_scenarios/results.csv`, with each run's launch log and a
`<name>_trace.csv` of the body's position, roll and pitch beside it.

```bash
ros2 run hexapod_bringup scenario_runner.py                # all, headless (~10 min)
ros2 run hexapod_bringup scenario_runner.py --only hill    # names containing "hill"
ros2 run hexapod_bringup scenario_runner.py --only rocks --gui   # watch in Gazebo
ros2 run hexapod_bringup scenario_runner.py --list         # names and spawn points
```

Soft ground can't be simulated (Gazebo's ground doesn't deform), and the
depth camera and lidar don't model sunlight.

This is a migration from an earlier Gazebo Classic setup (per the
`gazebo.launch.py` comments) — the "explore SLAM, Nav2, Gazebo simulation"
work referenced in the project's broader roadmap builds on this
Harmonic-based sim.

## Operator stack (`operator_ws`)

A separate **ROS 2 Jazzy** workspace (the same distro as `hexapod_ws`)
containing a single package, `operator_bringup`, which launches:

- **`game_controller_node`** (from `joy`) — reads the first gamepad
  (`device_id` 0) in the standard game controller layout, 0.05 deadzone,
  20 Hz autorepeat.
- **`foxglove_bridge_node`** — lets the operator's Foxglove Studio connect
  back to the robot's own Foxglove bridge (or vice versa, depending on
  which side initiates), for visualizing joint states, IMU, sim camera
  feeds, etc.

Two ways to run it:

- **Docker** (`operator_ws/dockerfile` + `docker-compose.yml`): builds on
  `ros:jazzy-ros-base`, installs `joy` + `foxglove_bridge`, and
  runs `operator.launch.py` on container start. `initHexOperator.sh` brings
  the container up, waits for the Foxglove WebSocket (port 8765) to come
  alive, then opens Foxglove via a deep link (`foxglove://open?...`).
- **Native** (`init_operator.sh`, repo root): sources ROS + both workspaces
  if built locally, verifies `foxglove_bridge`, `joy`, `hexapod_interfaces`,
  and `hexapod_description` are all resolvable in the active overlay
  (installing `foxglove_bridge` via `apt` if missing), then starts
  `game_controller_node`, `foxglove_bridge`, and `foxglove-studio` directly
  on the host. Jazzy needs Ubuntu 24.04 (on Windows, a 24.04 WSL distro).

## Foxglove panel

`foxglove_panel/` is a Foxglove extension adding a **Hexapod** panel: body
roll and pitch, which feet are on the ground and how hard, power for each side,
mode, gait and body height, and switches for gait_controller's features (each
showing the node's current parameter). See its README to build and install it.

## Steam Deck client

`steamdeck/` scripts turn a Steam Deck (via `distrobox`) into a portable
Foxglove viewing/joystick client:

- **`setup_distrobox.sh`** — run once on the Deck (in Konsole) to create the
  `jazzy` distrobox: Ubuntu 24.04 with ROS 2 Jazzy, matching the robot, plus
  `joy`, Foxglove Studio and the Hexapod panel. Running it again updates them.
- **`install_panel.sh`** — builds `foxglove_panel/` and installs it into
  Foxglove Studio (called by `setup_distrobox.sh`).
- **`hexapod_launch.sh`** — the Steam-launchable entry point. Sets up the
  Deck's Wayland/X display environment, then runs `ros2_hexapod_launch.sh`
  inside the `jazzy` distrobox.
- **`ros2_hexapod_launch.sh`** — finds the robot's `foxglove_bridge` (port
  `8765`): the hotspot address 192.168.50.1 first, otherwise a scan of the
  local `192.168.x.x/24` subnet. It then starts `game_controller_node` and
  opens `foxglove-studio` in kiosk mode pointed at the bridge over a
  `foxglove://` deep link.
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
port and `/dev/i2c-1` are visible.

**Setting up the robot's Pi (`pi/`):** on a fresh Raspberry Pi OS install,
with the repo cloned to `~/hexapod-ros2` and a GitHub SSH key on the Pi, run
`pi/setup_pi.sh` once and reboot. It:

- turns on I2C at 400 kHz and the servo UART (`/dev/ttyAMA0`), and installs
  `i2c-tools` and `iw`;
- installs the udev rule that names the lidar `/dev/rplidar`;
- names the Pi `hexapod`, in cloud-init's `user-data` too, which otherwise
  renames it at every boot;
- adds a Wi-Fi hotspot, `hexapod`, at 192.168.50.1 for the Steam Deck, so ROS 2
  DDS doesn't depend on a home router passing multicast. It runs on a second
  interface (`uap0`) on the same radio while `wlan0` stays on the home Wi-Fi
  for internet, which it shares with whatever joins the hotspot. The radio
  can only use one channel, and the hotspot can't use 5 GHz radar channels,
  so the home Wi-Fi is kept to 2.4 GHz and a NetworkManager dispatcher script
  moves the hotspot to its channel. The password is generated on first run
  (set `HOTSPOT_PASSWORD=...` to choose or change it);
- adds `hexapod-start.service`, which at every boot pulls the latest code
  (fast-forward only), rebuilds the image with `--pull` (a newer ROS base
  image means a full rebuild, picking up new `foxglove_bridge` and ROS
  releases), and runs `docker compose up -d`. Offline, it starts the image it
  already has. `journalctl -u hexapod-start` shows what it did.

The Steam Deck script tries the hotspot address first, then scans the local
network. Its scan opens and closes a connection on port 8765 at every
address, which `foxglove_bridge` logs as `handshake failed`; that is harmless.

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
- **Contact sensing and terrain adaptation are tuned on the simulation
  only.** The contact, touchdown and stumble thresholds and the load
  balancing gain need checking against the real servos' load readings, and
  the height map against the real ToF camera's noise.
- **`hexapod_control` is an empty stub.** `behaviour_manager.cpp` and
  `config_manager.cpp` are 0-byte files with no build targets.
- **Leg odometry can still drift when feet slip without the robot pushing
  against anything** (e.g. on a slippery or moving surface), since it assumes
  planted feet stay put.
- **The robot creeps on a tilting platform in simulation.** With the deck
  rolling and pitching ±6.9° together, the levelled robot slides several cm
  and turns in 20 s (worse when the deck moves slowly, so it is the angle,
  not the speed). At ±3.4° it barely moves. Neither multi-point feet nor load
  balancing changed it. The sim's legs and contacts are rigid; the real
  robot's rubber tips and sprung tibias should grip better.
- **Package descriptions/licenses are all `TODO`** in every `package.xml`.
- All `package.xml` maintainer emails are real personal addresses — worth
  reviewing before making the repo public.
