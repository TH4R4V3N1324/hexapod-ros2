# hexapod-ros2

ROS 2 control stack for a custom 6-legged (18-DOF) walking robot, plus a
separate operator workstation stack for remote teleoperation. The robot side
runs on **ROS 2 Jazzy**, drives an ESP32/Servo2040-based hardware backend over
a custom serial protocol, and can also run headless in **Gazebo Harmonic**
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
│       ├── hexapod_hardware_interface/ # real + fake hardware backends
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
                                hardware_interface → serial → Servo2040
                                              │
                                              ▼
                          /joint_states, /imu_data, /power_status_data,
                          /leg_contact_data  → foxglove_bridge → operator's
                                                                Foxglove Studio
```

## Hardware

- **6 legs**, 3 joints per leg (`coxa`, `femur`, `tibia`) → **18 DOF total**,
  named `leg{1-6}_{coxa,femur,tibia}_joint`.
- Onboard sensing (per `hexapod_interfaces` and the sensor parsing protocol):
  IMU (pitch/roll), a 2S LiPo power monitor (voltage/current), and 6 binary
  leg-contact sensors.
- A servo/sensor daughterboard (Servo2040-class board, referred to elsewhere
  as the embedded/ESP32 side of the project) talks to `hexapod_hardware_interface`
  over a custom binary serial protocol (see [Custom interfaces](#custom-interfaces)).
- Onboard compute runs Foxglove Bridge so a remote operator can visualize
  joint states, IMU, and camera/lidar (in sim) without needing RViz on the
  robot itself.

## Packages (`hexapod_ws`)

| Package | Type | Purpose |
|---|---|---|
| `hexapod_interfaces` | msg/srv | All custom message and service definitions shared across the stack. |
| `hexapod_description` | C++ / xacro | URDF (visual + collision meshes) and a simplified "primitives" xacro used for Gazebo physics; RViz config; Gazebo Harmonic world; ros_gz bridge config; `joint_state_relay` node. |
| `hexapod_hardware_interface` | C++ | `hardware_interface` (real) and `hardware_interface_fake` (simulated servo response) nodes, built from shared `SensorHandler`/`ServoHandler` classes. |
| `hexapod_gait_controller` | C++ | The core `gait_controller` node: inverse kinematics, Bezier swing trajectories, gait/mode state machine, and the `leg_axis_tester` diagnostic tool. |
| `hexapod_teleop` | C++ | `keyboard_teleop` (raw-terminal WASD-style control) and `controller_teleop` (gamepad `/joy` → `/cmd_vel`). |
| `hexapod_bringup` | launch only | Top-level launch files that compose the packages above for fake-hardware and simulation runs. |
| `hexapod_control` | C++ (stub) | Placeholder package for a `behaviour_manager` and `config_manager`; both source files currently exist but are empty — see [Known gaps](#known-gaps--todo). |

### `hexapod_description`

- `urdf/hexapod.urdf.xacro` — full visual model (DAE meshes for body, coxa,
  femur, tibia) used for RViz display.
- `urdf/hexapod_primitives.urdf.xacro` — a simplified collision-geometry
  version used for Gazebo physics, parameterized with `ros2_control`
  hardware and a front depth camera (resolution/rate are launch arguments).
- `config/controllers.yaml` — `ros2_control` config: a
  `joint_state_broadcaster` and a single `hexapod_joint_controller`
  (`position_controllers/JointGroupPositionController`) covering all 18
  joints, running at 1000 Hz.
- `worlds/hexapod_world.sdf` — the Gazebo Harmonic world.
- `src/joint_state_relay.cpp` — bridges the gait controller's
  `sensor_msgs/JointState` on `joint_commands` into the flat
  `Float64MultiArray` that `hexapod_joint_controller` expects on
  `/hexapod_joint_controller/commands`, using `KeepLast(1)` QoS so stale
  commands never queue up under load.

### `hexapod_hardware_interface`

Two executables share the same `SensorHandler`/`ServoHandler` logic:

- **`hardware_interface`** — intended for the real robot. Constructs the
  handlers but (see [Known gaps](#known-gaps--todo)) does not yet drive an
  actual serial port from this executable.
- **`hardware_interface_fake`** — runs a wall timer at a configurable
  `publish_rate` (default 100 Hz) that simulates servo motion toward the
  last commanded joint positions (exponential smoothing, `alpha = 0.3`) and
  publishes synthetic IMU/power/contact data. This is what
  `hexapod_bringup`'s fake-hardware launch uses, so gait logic can be
  exercised without any physical robot attached.
- **`ServoHandler`** packs 18 commanded joint angles into a
  `[0xAA][length][18×float32][XOR checksum]` binary packet
  (`buildPacket()`) intended for the servo daughterboard, and publishes
  `sensor_msgs/JointState` feedback on `joint_states`.
- **`SensorHandler`** parses a `[0xBB][len][pitch][roll][current][voltage][6-bit contacts]`
  packet from the board into `sensor_msgs/Imu`, `PowerStatus`, and
  `LegContact` messages (`parseAndPublish()`), or synthesizes the same
  messages in fake mode (`publishFakeData()`).

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
  axis 0 → `linear.y`, axis 3 → `angular.z`, all clamped to `[-1, 1]` and
  published on `/cmd_vel`. Also queries `get_capabilities` once at startup
  (gait/mode lists aren't currently used to drive gait switching from the
  gamepad — see [Known gaps](#known-gaps--todo)).

### `hexapod_bringup`

- `hexapod_fake.launch.py` — `robot_state_publisher` + `hardware_interface_fake`
  + `gait_controller` + `foxglove_bridge` + `controller_teleop`. The
  "everything except real servos" launch, useful for bench-testing gait
  logic and the full ROS graph without Gazebo.
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
| `/joint_states` | `sensor_msgs/JointState` | `servo_handler` (in `hardware_interface[_fake]`) | `gait_controller`, `robot_state_publisher` |
| `/joint_commands` | `sensor_msgs/JointState` | `gait_controller`, `leg_axis_tester` | `servo_handler`, `joint_state_relay` (sim) |
| `/hexapod_joint_controller/commands` | `std_msgs/Float64MultiArray` | `joint_state_relay` | `hexapod_joint_controller` (ros2_control, sim only) |
| `/imu_data` | `sensor_msgs/Imu` | `sensor_handler` | — |
| `/power_status_data` | `hexapod_interfaces/PowerStatus` | `sensor_handler` | — |
| `/leg_contact_data` | `hexapod_interfaces/LegContact` | `sensor_handler` | — |
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

Defined in `hexapod_interfaces`:

**Messages**
- `HexapodState` — `mode`, `gait`, `body_height`, `gait_phase`, `is_moving`, `is_stable`.
- `PowerStatus` — `voltage`, `current`, `power`, `battery_percent`, `low_battery_warning`.
- `LegContact` — `header` + `bool[6] contacts`.
- `LocomotionOption` — `string name`, `uint8 id` (used to describe available gaits/modes).
- `ControllerInput` — normalized axes, a `command_id`/`arguments` pair, and a
  `source_id` enum (`ESPNOW`, `VIRTUAL_JOY`, `KEYBOARD`, `GAMEPAD`,
  `AUTONOMY`) intended for a lower-level embedded controller input path;
  not currently published/subscribed anywhere in this workspace.

**Services**
- `SetGait(uint8 gait) → (bool success, string message)` — `0=tripod, 1=ripple, 2=wave`.
- `SetMode(uint8 mode) → (bool success, string message)` — `0=normal, 1=strafe, 2=tilt, 3=config`.
- `SetHeight(float64 height) → (bool success, string message)`.
- `GetCapabilities() → (LocomotionOption[] modes, LocomotionOption[] gaits, float64 max_linear_vel, float64 max_angular_vel)`.
- `IKSolver(uint8 leg_index, geometry_msgs/Point target) → sensor_msgs/JointState`.
- `FKSolver(sensor_msgs/JointState) → geometry_msgs/Point foot_position`.

## Gait controller internals

`gait_controller` (in `hexapod_gait_controller`) is the central node. It
runs a 50 Hz wall timer (`gaitTimerCallback`) and composes three helper
classes:

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

1. **Startup** — on first joint-state message, interpolates every leg from
   its current position to its configured home stance
   (`startup()`/`startup_trajectory`) before gait execution begins.
2. **Walk** — while `/cmd_vel` is non-zero (beyond
   `CMD_VEL_CHANGE_THRESHOLD = 0.05`), continuously regenerates swing/stance
   trajectories per the active gait's leg groupings (tripod/ripple/wave
   phase patterns from `GaitConfig::getGaitConfig`) and steps through them
   (`PerformLegStep`).
3. **Idle return** — when velocity commands drop to zero, legs are eased
   back to a stable stance (`HandleIdleReturn`/`returnToStart`) rather than
   stopping mid-stride.

Motion limits are capped at `MAX_LINEAR_VELOCITY = 0.2 m/s`,
`MAX_ANGULAR_VELOCITY = 2.0 rad/s`, `MAX_STRIDE_LENGTH = 0.06 m` — these are
also what `get_capabilities` reports back to teleop clients.

`leg_axis_tester` is a small diagnostic executable that shares the same
kinematics/trajectory libraries as `gait_controller`, publishing to
`joint_commands` directly — useful for exercising a single leg's axes
without the full gait state machine.

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
- Bridges core sim topics unconditionally (`/clock`, `/imu`, `/imu/mag`,
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

# Bench-test gait logic with fake hardware (no Gazebo, no real servos):
ros2 launch hexapod_bringup hexapod_fake.launch.py

# Or run in Gazebo Harmonic:
ros2 launch hexapod_bringup hexapod_simulation.launch.py bridge_vision:=true
```

The plain `docker-compose.yml` (no dev tools, `INSTALL_DEV_TOOLS=false`)
builds a lean runtime image whose default `CMD` runs
`hexapod_fake.launch.py` — intended for the real robot's onboard computer
where Gazebo/RViz/X11 aren't needed. **A real `hardware_interface` launch
file isn't wired into `hexapod_bringup` yet** — see below.

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

- **`hardware_interface` doesn't send anything over serial yet.** The real
  (non-fake) hardware interface node constructs `SensorHandler`/`ServoHandler`
  but there's no serial port open/write call in `hardware_interface.cpp`
  itself, and no launch file wires it up (`hexapod_bringup` only launches
  `hardware_interface_fake`). `ServoHandler::buildPacket()` exists and is
  presumably meant to be sent to the Servo2040 board, but that transport
  layer isn't implemented in this workspace yet.
- **`hexapod_control` is an empty stub.** `behaviour_manager.cpp` and
  `config_manager.cpp` are 0-byte files with no build targets.
- **ROS distro mismatch.** `hexapod_ws` targets Jazzy; `operator_ws`
  targets Humble. This works today because they only interact over
  DDS/network, but it's worth keeping in mind if message definitions ever
  need to cross that boundary directly.
- **`ControllerInput.msg`** (with its `SOURCE_ESPNOW`/`SOURCE_AUTONOMY`
  etc. constants) isn't published or subscribed anywhere in this workspace
  — it looks like it's meant for a lower-level embedded input path that
  isn't part of `hexapod_ws` yet.
- **Package descriptions/licenses are all `TODO`** in every `package.xml`.
- All `package.xml` maintainer emails are real personal addresses — worth
  reviewing before making the repo public.
