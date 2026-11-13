#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <optional>
#include <poll.h>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "gait_core/kinematic_solver.hpp"
#include "hexapod_hardware_interface/st3215_bus.hpp"
#include "test_common.hpp"

using namespace hardware_tests;
using namespace hexapod_hardware_interface;
using gait_core::JointAngles;
using gait_core::KinematicSolver;
using Eigen::Vector3d;

static const char* USAGE = R"(Drive one leg's foot with the gait controller's IK, and check it gets there.

Usage: test_leg <command> <leg 1-6> [args] [options]
       test_leg [shell] [options]   With no command, an interactive shell: type the commands below at its
                                    prompt, leaving out the leg to use the selected one.

Positions are mm in the leg frame: +x out from the coxa, +y anticlockwise seen from above, +z up. The
stand position is (177, 0, -134): 177 mm out and 134 mm below the hip.

Looking:
  where <leg>               Where the foot is (from the joints) and the joint angles.
  monitor <leg>             The same, live. With the leg limp, move the foot by hand: x should grow as you
                            pull it outward, y as you swing it anticlockwise, z as you lift it. If one goes
                            the wrong way, that joint's direction is wrong.
  limp <leg>                Torque off on the leg's three servos.

Moving (the foot moves in straight lines at --foot-speed; every point is checked against the joint ranges
before anything moves):
  stand <leg>               To the stand position, joint by joint, from wherever the leg is.
  goto <leg> <x> <y> <z>    To a position, in a straight line from where the foot is.
  move <leg> <dx> <dy> <dz> By that much from where the foot is.
  axis <leg> x|y|z [mm]     From stand: +mm, back, -mm, back (default --step). --cycles <n> repeats it.
  swing <leg>               Gait-like swings across --step either side, lifting --lift. --cycles <n>.
  circle <leg> [mm]         Round a circle about the stand position (default radius --step), in --plane
                            xy (level, the default), xz (out/in and up/down) or yz. --cycles <n>.
  speed <leg> [x|y|z]       The axis move (default x) at each of --speeds, reading the joints throughout:
                            the lag behind the target, the error once stopped, the peak load and the
                            fastest joint the move needed, per speed.
  test <leg>                Stand, then out/in, each side, up/down and a swing, checking each point.

Options:
  --foot-speed <mm/s>       How fast the foot moves (default 30).
  --speed <°/s>             How fast 'stand' turns the joints (default 30).
  --step <mm>               Distance for axis, swing, circle, speed and test (default 30).
  --lift <mm>               Height for swing and test's up move (default 40).
  --cycles <n>              Repeats for axis, swing and circle (default 1).
  --speeds <mm/s,...>       Foot speeds for speed (default 25,50,100,200).
  --pause                   test: wait for Enter at each point, to measure it.
  --yes                     Don't ask before the first move.
  --port /dev/ttyAMA0  --baud 1000000  --echo  --debug (print every packet)

Lift the robot so the leg hangs free: at stand the foot is below the body's underside, so on the ground it
pushes the body up. The first move asks you to (when the leg's torque is off).

The numbers come from the servos' own encoders: they show how well the leg tracks, but not a joint turning
the wrong way or an off-centre servo (the encoders agree with the command either way). Watch each move go
the way it says, or use monitor. Joint directions and offsets come from hexapod.ros2_control.xacro, as on
the robot. Torque stays on after a move; run 'limp' to release it.
)";

static const char* SHELL_HELP = R"(
In the shell (the commands above work the same, without 'test_leg'):
  select <leg>              Set the leg commands use when you leave the leg out.
  baud <rate>               Change the rate this end of the bus talks at.
  debug [on|off]            Print every packet sent and byte received.
  echo [on|off]             Discard the bytes the bus adapter echoes back.
  help [command]            This list, or one command's help.
  quit                      Leave (also exit, or Ctrl+D).
The leg can be left out of any command: 'stand' stands the selected leg, 'axis z 20' moves it up and down
20 mm, 'goto 190 0 -120' sends its foot there. A number from 1 to 6 first is still the leg, as on the command
line: 'circle 3' circles leg 3. Options can go on any line ('axis x --foot-speed 60'), and the ones given when
the shell started apply to every line. Ctrl+C stops a running move.
)";

// Options that take no value
static const std::vector<std::string> FLAGS = {"echo", "pause", "yes", "debug"};

// The stand position in the leg frame (+x out from the coxa, z up), as gait_config.hpp's startPos
static const Vector3d STAND(0.177, 0.0, -0.134);
// Joint ranges, as GaitController::reachable and the URDF
static constexpr double MAX_COXA = 1.4;
static constexpr double FEMUR_MIN = -1.6, FEMUR_MAX = 1.4;
static constexpr double TIBIA_MIN = -0.78, TIBIA_MAX = 2.63;
static constexpr double REACH_TOLERANCE = 0.001;  // m
static constexpr double TICK = 0.02;              // s between foot targets on a path
static constexpr int ARRIVED_TOLERANCE = 10;      // steps, about 0.9°
static constexpr double GOOD_FOOT_ERROR = 0.005;  // m
static constexpr double AT_STAND = 0.002;         // m, close enough to stand not to go there first
static constexpr double SERVO_TOP_SPEED = 270.0;  // °/s, ST3215 at 12 V with no load (0.222 s/60°)
static constexpr uint8_t ACCELERATION = 20;

/**
 * @brief How one joint's servo maps to its joint angle, as ST3215System does
*/
struct JointMapping {
    double direction = 1.0;
    double offset = 0.0;  // rad, joint angle at the servo's centre

    int toSteps(double radians) const {
        const double steps = direction * (radians - offset) * ST3215_STEPS_PER_REV / (2.0 * M_PI) + ST3215_CENTER_POSITION;
        return std::clamp(static_cast<int>(std::lround(steps)), 0, ST3215_STEPS_PER_REV - 1);
    }
    double toRadians(int steps) const {
        return direction * (steps - ST3215_CENTER_POSITION) * 2.0 * M_PI / ST3215_STEPS_PER_REV + offset;
    }
};

double degrees(double radians) {
    return radians * 180.0 / M_PI;
}

/**
 * @brief Read the leg's joint directions and offsets from the robot's URDF (hardware:=st3215)
 * @return The coxa, femur and tibia mappings, or nullopt if xacro or the parameters couldn't be read
*/
std::optional<std::array<JointMapping, 3>> loadMappings(int leg) {
    std::string xacro_file;
    try {
        xacro_file = ament_index_cpp::get_package_share_directory("hexapod_description") + "/urdf/hexapod.urdf.xacro";
    } catch (const std::exception&) {
        return std::nullopt;
    }
    FILE* pipe = popen(("xacro '" + xacro_file + "' hardware:=st3215 2>/dev/null").c_str(), "r");
    if (!pipe) {
        return std::nullopt;
    }
    std::string urdf;
    char buffer[4096];
    size_t n;
    while ((n = fread(buffer, 1, sizeof(buffer), pipe)) > 0) {
        urdf.append(buffer, n);
    }
    if (pclose(pipe) != 0) {
        return std::nullopt;
    }

    // The ros2_control joint is the one with a servo_id parameter (the transmission's has none)
    auto param = [](const std::string& block, const std::string& name) -> std::optional<double> {
        const std::string open = "<param name=\"" + name + "\">";
        size_t start = block.find(open);
        if (start == std::string::npos) {
            return std::nullopt;
        }
        start += open.size();
        return std::stod(block.substr(start, block.find('<', start) - start));
    };
    std::array<JointMapping, 3> mappings;
    const char* joints[] = {"coxa", "femur", "tibia"};
    for (int j = 0; j < 3; ++j) {
        const std::string open = format("<joint name=\"leg%d_%s_joint\">", leg, joints[j]);
        bool found = false;
        for (size_t at = urdf.find(open); at != std::string::npos; at = urdf.find(open, at + 1)) {
            const std::string block = urdf.substr(at, urdf.find("</joint>", at) - at);
            if (block.find("servo_id") == std::string::npos) {
                continue;
            }
            auto direction = param(block, "direction");
            auto offset = param(block, "offset");
            if (!direction || !offset) {
                return std::nullopt;
            }
            mappings[j] = {*direction < 0 ? -1.0 : 1.0, *offset};
            found = true;
            break;
        }
        if (!found) {
            return std::nullopt;
        }
    }
    return mappings;
}

/**
 * @brief Whether the leg can put its foot at a target, as GaitController::reachable
*/
bool reachable(KinematicSolver& ik, const Vector3d& target) {
    const JointAngles a = ik.solveIK(target);
    return std::abs(a.coxa) <= MAX_COXA && a.femur >= FEMUR_MIN && a.femur <= FEMUR_MAX &&
        a.tibia >= TIBIA_MIN && a.tibia <= TIBIA_MAX && (ik.solveFK(a) - target).norm() <= REACH_TOLERANCE;
}

/**
 * @brief Wait for Enter, or return false on Ctrl+C or end of input
*/
bool waitForEnter() {
    while (!stop_requested) {
        pollfd input{STDIN_FILENO, POLLIN, 0};
        if (poll(&input, 1, 100) > 0) {
            char line[256];
            if (fgets(line, sizeof(line), stdin)) {
                return true;
            }
            clearerr(stdin);
            return false;
        }
    }
    return false;
}

std::string mm(const Vector3d& point) {
    return format("(%.1f, %.1f, %.1f) mm", point.x() * 1000.0, point.y() * 1000.0, point.z() * 1000.0);
}

/**
 * @brief One leg on the bus: its servos, joint mappings and IK
*/
class Leg {
public:
    Leg(ST3215Bus& bus, int number, const std::array<JointMapping, 3>& mappings)
        : bus_(bus), number_(number), mappings_(mappings) {
        for (int joint = 1; joint <= 3; ++joint) {
            ids_.push_back(static_cast<uint8_t>(number * 10 + joint));
        }
    }

    int number() const { return number_; }
    const std::vector<uint8_t>& ids() const { return ids_; }
    std::string idList() const { return format("%d %d %d", ids_[0], ids_[1], ids_[2]); }

    /**
     * @brief Read the servos
     * @return Each joint's servo position (steps), speed and load (0.1%), or nullopt if one didn't reply
    */
    std::optional<std::array<ST3215State, 3>> read() {
        std::vector<std::optional<ST3215State>> states;
        bus_.readStates(ids_, states);
        std::array<ST3215State, 3> result;
        for (int j = 0; j < 3; ++j) {
            if (j >= static_cast<int>(states.size()) || !states[j]) {
                return std::nullopt;
            }
            result[j] = *states[j];
        }
        return result;
    }

    JointAngles angles(const std::array<ST3215State, 3>& states) const {
        return {mappings_[0].toRadians(states[0].position), mappings_[1].toRadians(states[1].position),
            mappings_[2].toRadians(states[2].position)};
    }

    std::vector<int> steps(const JointAngles& a) const {
        return {mappings_[0].toSteps(a.coxa), mappings_[1].toSteps(a.femur), mappings_[2].toSteps(a.tibia)};
    }

    void send(const JointAngles& a, uint16_t speed) {
        bus_.writeGoalPositions(ids_, steps(a), ACCELERATION, speed);
    }

    /**
     * @brief Whether every joint's torque is on
    */
    bool holding() {
        for (uint8_t id : ids_) {
            std::vector<uint8_t> data;
            uint8_t error = 0;
            if (!bus_.readRegisters(id, ST3215_REG_TORQUE_ENABLE, 1, data, error) || data[0] == 0) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Hold where the joints are now, then turn torque on
    */
    bool hold() {
        auto states = read();
        if (!states) {
            return false;
        }
        bus_.writeGoalPositions(ids_, {states->at(0).position, states->at(1).position, states->at(2).position},
            ACCELERATION, 0);
        return bus_.setTorqueEnabled(ids_, true);
    }

    bool limp() {
        return bus_.setTorqueEnabled(ids_, false);
    }

private:
    ST3215Bus& bus_;
    int number_;
    std::array<JointMapping, 3> mappings_;
    std::vector<uint8_t> ids_;
};

/**
 * @brief Everything a command needs
*/
struct Context {
    Leg& leg;
    KinematicSolver& ik;
    const Args& args;
    double foot_speed;     // m/s
    double step;           // m
    double lift;           // m
    int cycles;
    double joint_speed_dps;
};

std::optional<Vector3d> footNow(Context& c) {
    auto states = c.leg.read();
    if (!states) {
        return std::nullopt;
    }
    return c.ik.solveFK(c.leg.angles(*states));
}

/**
 * @brief Print where the foot is and the joint angles
*/
bool printWhere(Context& c, bool one_line) {
    auto states = c.leg.read();
    if (!states) {
        printf("No reply from leg %d\n", c.leg.number());
        return false;
    }
    const JointAngles a = c.leg.angles(*states);
    const Vector3d foot = c.ik.solveFK(a);
    printf("%sfoot at %-28s joints coxa %+6.1f° femur %+6.1f° tibia %+6.1f°   load %+5.1f%% %+5.1f%% %+5.1f%%%s",
        one_line ? "\r" : "", mm(foot).c_str(), degrees(a.coxa), degrees(a.femur), degrees(a.tibia),
        states->at(0).load / 10.0, states->at(1).load / 10.0, states->at(2).load / 10.0, one_line ? "  " : "\n");
    fflush(stdout);
    return true;
}

/**
 * @brief Before moving: ask for the robot to be lifted if the leg is limp, then hold where it is
*/
bool prepare(Context& c) {
    if (!c.leg.holding() && !c.args.has("yes")) {
        printf("Lift the robot so leg %d hangs free: at stand the foot is %.0f mm below the hip. Press Enter to start "
            "(Ctrl+C cancels).\n", c.leg.number(), -STAND.z() * 1000.0);
        if (!waitForEnter()) {
            printf("Cancelled\n");
            return false;
        }
    }
    if (!c.leg.hold()) {
        printf("No reply from leg %d. Run 'test_servos scan'.\n", c.leg.number());
        return false;
    }
    return true;
}

/**
 * @brief Points along a straight line, spaced for the foot speed
*/
std::vector<Vector3d> line(const Vector3d& from, const Vector3d& to, double speed) {
    const int count = std::max(1, static_cast<int>(std::ceil((to - from).norm() / (speed * TICK))));
    std::vector<Vector3d> points;
    for (int i = 1; i <= count; ++i) {
        points.push_back(from + (to - from) * (static_cast<double>(i) / count));
    }
    return points;
}

/**
 * @brief Points along a swing: across from one side to the other, lifting by lift at the middle
*/
std::vector<Vector3d> arc(const Vector3d& from, const Vector3d& to, double lift, double speed) {
    const double length = (to - from).norm() + 2.0 * lift;  // roughly the arc's length
    const int count = std::max(1, static_cast<int>(std::ceil(length / (speed * TICK))));
    std::vector<Vector3d> points;
    for (int i = 1; i <= count; ++i) {
        const double s = static_cast<double>(i) / count;
        points.push_back(from + (to - from) * s + Vector3d(0.0, 0.0, lift * std::sin(M_PI * s)));
    }
    return points;
}

void append(std::vector<Vector3d>& path, const std::vector<Vector3d>& more) {
    path.insert(path.end(), more.begin(), more.end());
}

/**
 * @brief Check every point of a path is reachable, printing the first one that isn't
*/
bool checkPath(KinematicSolver& ik, const std::vector<Vector3d>& path) {
    for (const auto& point : path) {
        if (!reachable(ik, point)) {
            printf("The leg can't reach %s (outside a joint's range): nothing moved\n", mm(point).c_str());
            return false;
        }
    }
    return true;
}

/**
 * @brief How the leg kept up while following a path
*/
struct Tracking {
    Stats lag;               // m, foot distance behind the target, read every tick
    int peak_load = 0;       // 0.1%
    double peak_joint_speed = 0.0;  // °/s the path asked of the fastest joint
};

/**
 * @brief Move the foot along a path of points, one per tick
 * @param[out] tracking If given, the joints are read every tick to see how far the foot lags
 * @return false if Ctrl+C stopped it or the leg stopped replying
*/
bool follow(Context& c, const std::vector<Vector3d>& path, Tracking* tracking = nullptr) {
    auto next = std::chrono::steady_clock::now();
    std::optional<JointAngles> previous;
    for (const Vector3d& point : path) {
        if (stop_requested) {
            return false;
        }
        const JointAngles target = c.ik.solveIK(point);
        c.leg.send(target, 0);
        if (tracking) {
            if (previous) {
                const double fastest = std::max({std::abs(target.coxa - previous->coxa),
                    std::abs(target.femur - previous->femur), std::abs(target.tibia - previous->tibia)});
                tracking->peak_joint_speed = std::max(tracking->peak_joint_speed, degrees(fastest) / TICK);
            }
            previous = target;
            auto states = c.leg.read();
            if (!states) {
                printf("    no reply from the leg\n");
                return false;
            }
            tracking->lag.add((c.ik.solveFK(c.leg.angles(*states)) - point).norm());
            for (const auto& state : *states) {
                tracking->peak_load = std::max(tracking->peak_load, std::abs(state.load));
            }
        }
        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(TICK));
        std::this_thread::sleep_until(next);
    }
    return true;
}

/**
 * @brief How far the leg got from a target once stopped, from the servos' encoders
*/
struct Check {
    double foot_error;   // m
    double joint_error;  // rad, the largest of the three
};

/**
 * @brief Wait for the leg to settle, then compare where it is with the target
 * @param[in] print Whether to print the foot and joint lines
*/
std::optional<Check> check(Context& c, const Vector3d& target, double settle, bool print = true) {
    std::this_thread::sleep_for(std::chrono::duration<double>(settle));
    auto states = c.leg.read();
    if (!states) {
        printf("    no reply from the leg\n");
        return std::nullopt;
    }
    const JointAngles measured = c.leg.angles(*states);
    const JointAngles wanted = c.ik.solveIK(target);
    const Vector3d foot = c.ik.solveFK(measured);
    const double errors[3] = {measured.coxa - wanted.coxa, measured.femur - wanted.femur, measured.tibia - wanted.tibia};
    double worst = 0.0;
    for (double e : errors) {
        worst = std::max(worst, std::abs(e));
    }
    const Vector3d d = (foot - target) * 1000.0;
    if (print) {
        printf("    foot at %s, %4.1f mm off (%+.1f, %+.1f, %+.1f)\n", mm(foot).c_str(), d.norm(), d.x(), d.y(), d.z());
        printf("    joints %+6.1f° %+6.1f° %+6.1f°, off by %+.1f° %+.1f° %+.1f°, load %+.1f%% %+.1f%% %+.1f%%\n",
            degrees(measured.coxa), degrees(measured.femur), degrees(measured.tibia),
            degrees(errors[0]), degrees(errors[1]), degrees(errors[2]),
            states->at(0).load / 10.0, states->at(1).load / 10.0, states->at(2).load / 10.0);
    }
    return Check{d.norm() / 1000.0, worst};
}

/**
 * @brief Move to the stand position joint by joint, from wherever the leg is
*/
bool goToStand(Context& c) {
    auto start = c.leg.read();
    if (!start) {
        printf("No reply from leg %d\n", c.leg.number());
        return false;
    }
    const Vector3d from = c.ik.solveFK(c.leg.angles(*start));
    if ((from - STAND).norm() <= AT_STAND) {
        return true;
    }
    printf("  to stand %s from %s at %.0f°/s\n", mm(STAND).c_str(), mm(from).c_str(), c.joint_speed_dps);
    const auto speed = static_cast<uint16_t>(std::clamp(c.joint_speed_dps * ST3215_STEPS_PER_REV / 360.0, 1.0, 3000.0));
    const JointAngles stand_angles = c.ik.solveIK(STAND);
    c.leg.send(stand_angles, speed);
    const auto goal = c.leg.steps(stand_angles);
    int largest = 0;
    for (int j = 0; j < 3; ++j) {
        largest = std::max(largest, std::abs(goal[j] - start->at(j).position));
    }
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(largest / static_cast<double>(speed) + 3.0);
    while (!stop_requested && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        auto states = c.leg.read();
        if (states && std::abs(states->at(0).position - goal[0]) <= ARRIVED_TOLERANCE &&
            std::abs(states->at(1).position - goal[1]) <= ARRIVED_TOLERANCE &&
            std::abs(states->at(2).position - goal[2]) <= ARRIVED_TOLERANCE) {
            break;
        }
    }
    return !stop_requested;
}

/**
 * @brief Follow a path and then check where the foot ended up
*/
int run(Context& c, const std::string& description, const std::vector<Vector3d>& path) {
    printf("  %s\n", description.c_str());
    if (!follow(c, path)) {
        printf("Stopped: the leg holds where it got to\n");
        return 1;
    }
    auto result = check(c, path.back(), 0.3);
    if (!result) {
        return 1;
    }
    printf("Torque is still on. Run 'test_leg limp %d' to release it.\n", c.leg.number());
    return result->foot_error <= GOOD_FOOT_ERROR ? 0 : 1;
}

int where(Context& c) {
    return printWhere(c, false) ? 0 : 1;
}

int monitor(Context& c) {
    printf("Leg %d, Ctrl+C to stop. With it limp ('test_leg limp %d'), move the foot by hand: x grows outward, "
        "y anticlockwise seen from above, z upward.\n", c.leg.number(), c.leg.number());
    while (!stop_requested) {
        if (!printWhere(c, true)) {
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("\n");
    return 0;
}

int limp(Context& c) {
    if (!c.leg.limp()) {
        printf("No reply from leg %d\n", c.leg.number());
        return 1;
    }
    printf("Torque off on servos %s\n", c.leg.idList().c_str());
    return 0;
}

int stand(Context& c) {
    if (!prepare(c) || !goToStand(c)) {
        return 1;
    }
    printf("  stand\n");
    auto result = check(c, STAND, 0.3);
    printf("Torque is still on. Run 'test_leg limp %d' to release it.\n", c.leg.number());
    return result && result->foot_error <= GOOD_FOOT_ERROR ? 0 : 1;
}

int goTo(Context& c, const Vector3d& target, const std::string& description) {
    auto from = footNow(c);
    if (!from) {
        printf("No reply from leg %d\n", c.leg.number());
        return 1;
    }
    const auto path = line(*from, target, c.foot_speed);
    if (!checkPath(c.ik, path)) {
        if (reachable(c.ik, target)) {
            printf("The target is reachable but the straight line from %s isn't: run 'stand' first\n", mm(*from).c_str());
        }
        return 1;
    }
    if (!prepare(c)) {
        return 1;
    }
    return run(c, description + " " + mm(target), path);
}

/**
 * @brief Go to stand first if the foot isn't there, ready for a move about it
*/
bool fromStand(Context& c, const std::vector<Vector3d>& path) {
    return checkPath(c.ik, path) && prepare(c) && goToStand(c);
}

std::optional<Vector3d> axisDirection(const std::string& axis) {
    if (axis == "x") return Vector3d::UnitX();
    if (axis == "y") return Vector3d::UnitY();
    if (axis == "z") return Vector3d::UnitZ();
    printf("The axis must be x, y or z\n");
    return std::nullopt;
}

std::vector<Vector3d> axisPath(const Vector3d& direction, double distance, double speed, int cycles) {
    std::vector<Vector3d> path;
    for (int i = 0; i < cycles; ++i) {
        append(path, line(STAND, STAND + direction * distance, speed));
        append(path, line(STAND + direction * distance, STAND, speed));
        append(path, line(STAND, STAND - direction * distance, speed));
        append(path, line(STAND - direction * distance, STAND, speed));
    }
    return path;
}

int axis(Context& c, const std::string& name, double distance) {
    auto direction = axisDirection(name);
    if (!direction) {
        return 1;
    }
    const auto path = axisPath(*direction, distance, c.foot_speed, c.cycles);
    if (!fromStand(c, path)) {
        return 1;
    }
    return run(c, format("along %s ±%.0f mm and back, %d cycle%s", name.c_str(), distance * 1000.0, c.cycles,
        c.cycles == 1 ? "" : "s"), path);
}

int swing(Context& c) {
    const Vector3d right = STAND - Vector3d(0, c.step, 0);
    const Vector3d left = STAND + Vector3d(0, c.step, 0);
    std::vector<Vector3d> path = line(STAND, right, c.foot_speed);
    for (int i = 0; i < c.cycles; ++i) {
        append(path, arc(right, left, c.lift, c.foot_speed));
        append(path, line(left, right, c.foot_speed));  // the stance stroke back, on the ground
    }
    append(path, line(right, STAND, c.foot_speed));
    if (!fromStand(c, path)) {
        return 1;
    }
    return run(c, format("swinging %.0f mm either side, lifting %.0f mm, %d cycle%s", c.step * 1000.0, c.lift * 1000.0,
        c.cycles, c.cycles == 1 ? "" : "s"), path);
}

int circle(Context& c, double radius) {
    const std::string plane = c.args.get("plane", "xy");
    Vector3d u, v;
    if (plane == "xy") {
        u = Vector3d::UnitX(), v = Vector3d::UnitY();
    } else if (plane == "xz") {
        u = Vector3d::UnitX(), v = Vector3d::UnitZ();
    } else if (plane == "yz") {
        u = Vector3d::UnitY(), v = Vector3d::UnitZ();
    } else {
        printf("The plane must be xy, xz or yz\n");
        return 1;
    }
    const Vector3d start = STAND + u * radius;
    std::vector<Vector3d> path = line(STAND, start, c.foot_speed);
    const int count = std::max(8, static_cast<int>(std::ceil(2.0 * M_PI * radius / (c.foot_speed * TICK))));
    for (int i = 0; i < c.cycles; ++i) {
        for (int k = 1; k <= count; ++k) {
            const double angle = 2.0 * M_PI * k / count;
            path.push_back(STAND + u * radius * std::cos(angle) + v * radius * std::sin(angle));
        }
    }
    append(path, line(start, STAND, c.foot_speed));
    if (!fromStand(c, path)) {
        return 1;
    }
    return run(c, format("round a %.0f mm circle in %s, %d cycle%s", radius * 1000.0, plane.c_str(), c.cycles,
        c.cycles == 1 ? "" : "s"), path);
}

/**
 * @brief Run the axis move at several foot speeds, reading the joints throughout
*/
int speed(Context& c, const std::string& name) {
    auto direction = axisDirection(name);
    if (!direction) {
        return 1;
    }
    std::vector<double> speeds;
    std::stringstream list(c.args.get("speeds", "25,50,100,200"));
    for (std::string item; std::getline(list, item, ',');) {
        speeds.push_back(std::stod(item) / 1000.0);
    }
    if (speeds.empty()) {
        printf("Give --speeds as mm/s separated by commas, e.g. 25,50,100\n");
        return 1;
    }
    if (!fromStand(c, axisPath(*direction, c.step, speeds.front(), 1))) {
        return 1;
    }
    printf("  along %s ±%.0f mm and back at each speed\n\n", name.c_str(), c.step * 1000.0);
    printf("  foot speed   lag mean   lag max   error stopped   peak load   fastest joint\n");
    Report report;
    for (double s : speeds) {
        if (stop_requested) {
            break;
        }
        Tracking tracking;
        if (!follow(c, axisPath(*direction, c.step, s, 1), &tracking)) {
            printf("Stopped: the leg holds where it got to\n");
            return 1;
        }
        auto stopped = check(c, STAND, 0.3, false);
        if (!stopped) {
            return 1;
        }
        printf("  %6.0f mm/s  %6.1f mm  %6.1f mm   %8.1f mm     %6.1f%%     %5.0f°/s%s\n", s * 1000.0,
            tracking.lag.mean() * 1000.0, tracking.lag.max() * 1000.0, stopped->foot_error * 1000.0,
            tracking.peak_load / 10.0, tracking.peak_joint_speed,
            tracking.peak_joint_speed > SERVO_TOP_SPEED ? "  (beyond the servos' top speed)" : "");
        if (stopped->foot_error > GOOD_FOOT_ERROR) {
            report.warn(format("At %.0f mm/s the foot stopped %.1f mm off", s * 1000.0, stopped->foot_error * 1000.0));
        }
    }
    printf("\n");
    report.info("Lag is how far the foot was behind the moving target, read every 20 ms; it grows with speed. The "
        "error once stopped should stay small at every speed.");
    report.info(format("Torque is still on. Run 'test_leg limp %d' to release it.", c.leg.number()));
    return report.finish();
}

/**
 * @brief Stand, then out/in, each side, up/down and a swing, checking each point
*/
int test(Context& c) {
    const Vector3d out = STAND + Vector3d(c.step, 0, 0);
    const Vector3d in = STAND - Vector3d(c.step, 0, 0);
    const Vector3d left = STAND + Vector3d(0, c.step, 0);
    const Vector3d right = STAND - Vector3d(0, c.step, 0);
    const Vector3d up = STAND + Vector3d(0, 0, c.lift);
    const Vector3d down = STAND - Vector3d(0, 0, c.step);
    const int step_mm = static_cast<int>(std::lround(c.step * 1000.0));
    const int lift_mm = static_cast<int>(std::lround(c.lift * 1000.0));
    const double v = c.foot_speed;
    const std::vector<std::pair<std::string, std::vector<Vector3d>>> moves = {
        {format("out %d mm, straight away from the body", step_mm), line(STAND, out, v)},
        {"back to stand", line(out, STAND, v)},
        {format("in %d mm, towards the body", step_mm), line(STAND, in, v)},
        {"back to stand", line(in, STAND, v)},
        {format("%d mm sideways, anticlockwise seen from above (the coxa turning positive)", step_mm), line(STAND, left, v)},
        {"back to stand", line(left, STAND, v)},
        {format("%d mm sideways, clockwise seen from above", step_mm), line(STAND, right, v)},
        {"back to stand", line(right, STAND, v)},
        {format("up %d mm, straight up", lift_mm), line(STAND, up, v)},
        {"back to stand", line(up, STAND, v)},
        {format("down %d mm, straight down", step_mm), line(STAND, down, v)},
        {"back to stand", line(down, STAND, v)},
        {"to the clockwise side, ready to swing", line(STAND, right, v)},
        {format("swing across to the anticlockwise side, lifting %d mm in the middle", lift_mm), arc(right, left, c.lift, v)},
        {"back to stand", line(left, STAND, v)},
    };
    std::vector<Vector3d> all;
    for (const auto& move : moves) {
        append(all, move.second);
    }
    if (!fromStand(c, all)) {
        return 1;
    }

    printf("  stand\n");
    auto stand_check = check(c, STAND, 0.3);
    if (!stand_check) {
        return 1;
    }
    double worst_foot = stand_check->foot_error;
    double worst_joint = stand_check->joint_error;
    std::string worst_move = "stand";
    for (const auto& move : moves) {
        if (stop_requested) {
            break;
        }
        printf("  %s\n", move.first.c_str());
        if (!follow(c, move.second)) {
            break;
        }
        auto result = check(c, move.second.back(), 0.3);
        if (!result) {
            return 1;
        }
        if (result->foot_error > worst_foot) {
            worst_foot = result->foot_error;
            worst_move = move.first;
        }
        worst_joint = std::max(worst_joint, result->joint_error);
        if (c.args.has("pause")) {
            printf("    Enter for the next move\n");
            if (!waitForEnter()) {
                break;
            }
        }
    }

    printf("\n");
    Report report;
    if (stop_requested) {
        report.warn("Stopped early: the leg holds where it got to");
    }
    if (worst_foot <= GOOD_FOOT_ERROR) {
        report.pass(format("The foot tracked within %.1f mm (joints within %.1f°)", worst_foot * 1000.0, degrees(worst_joint)));
    } else {
        report.warn(format("The foot was up to %.1f mm off (joints up to %.1f°), on '%s': blocked, a limit, a loose "
            "horn, or too fast (--foot-speed)", worst_foot * 1000.0, degrees(worst_joint), worst_move.c_str()));
    }
    report.info("These come from the servos' encoders: check by eye that each move went the way it said. A move "
        "the wrong way is a joint with the wrong direction; a stand pose that looks wrong is a joint off centre.");
    report.info(format("Torque is still on. Run 'test_leg limp %d' to release it.", c.leg.number()));
    return report.finish();
}

bool isAxis(const std::string& text) {
    return text == "x" || text == "y" || text == "z";
}

/**
 * @brief Whether the arguments after the leg are a form a command takes
 * @return nullopt if there's no such command
*/
std::optional<bool> fitsAfterLeg(const std::string& command, const std::vector<std::string>& rest) {
    const size_t n = rest.size();
    for (const char* name : {"where", "monitor", "limp", "stand", "swing", "test"}) {
        if (command == name) {
            return n == 0;
        }
    }
    if (command == "goto" || command == "move") {
        return n == 3;
    }
    if (command == "axis") {
        return (n == 1 || n == 2) && isAxis(rest[0]);
    }
    if (command == "circle") {
        return n <= 1;
    }
    if (command == "speed") {
        return n == 0 || (n == 1 && isAxis(rest[0]));
    }
    return std::nullopt;
}

/**
 * @brief The open bus, and what's been loaded for each leg
*/
struct Session {
    ST3215Bus bus;
    std::string port;
    int baud = 1000000;
    bool echo = false;
    bool debug = false;
    KinematicSolver ik;
    std::map<int, std::array<JointMapping, 3>> mappings;  // by leg, read from the URDF once
};

/**
 * @brief Run one command on one leg
 * @param[in] pos The positional arguments: the command, the leg, then its arguments
 * @return The command's exit code, or -1 if the arguments weren't a form the command takes
*/
int runCommand(Session& session, const std::vector<std::string>& pos, const Args& args) {
    const std::string& command = pos[0];
    if (pos.size() < 2 || !fitsAfterLeg(command, {pos.begin() + 2, pos.end()}).value_or(false)) {
        return -1;
    }
    if (!isIntegerIn(pos[1], 1, 6)) {
        printf("The leg must be 1-6\n");
        return -1;
    }
    const int leg_number = std::stoi(pos[1]);

    auto mappings = session.mappings.find(leg_number);
    if (mappings == session.mappings.end()) {
        auto loaded = loadMappings(leg_number);
        if (!loaded) {
            loaded = std::array<JointMapping, 3>{};
            printf("Couldn't read the URDF (is the workspace sourced?): using direction 1 and offset 0 for every "
                "joint of leg %d\n", leg_number);
        }
        mappings = session.mappings.emplace(leg_number, *loaded).first;
    }
    Leg leg(session.bus, leg_number, mappings->second);
    for (uint8_t id : leg.ids()) {
        if (!session.bus.ping(id)) {
            printf("Servo %d (leg %d) didn't reply. Run 'test_servos scan'.\n", id, leg_number);
            return 1;
        }
    }

    try {
        Context c{leg, session.ik, args, std::max(args.getDouble("foot-speed", 30.0), 1.0) / 1000.0,
            args.getDouble("step", 30.0) / 1000.0, args.getDouble("lift", 40.0) / 1000.0,
            std::max(args.getInt("cycles", 1), 1), args.getDouble("speed", 30.0)};
        auto at = [&pos](size_t i) { return std::stod(pos[i]) / 1000.0; };

        if (command == "where") {
            return where(c);
        } else if (command == "monitor") {
            return monitor(c);
        } else if (command == "limp") {
            return limp(c);
        } else if (command == "stand") {
            return stand(c);
        } else if (command == "goto") {
            return goTo(c, Vector3d(at(2), at(3), at(4)), "to");
        } else if (command == "move") {
            auto from = footNow(c);
            if (!from) {
                printf("No reply from leg %d\n", leg_number);
                return 1;
            }
            return goTo(c, *from + Vector3d(at(2), at(3), at(4)), "by " + mm(Vector3d(at(2), at(3), at(4))) + ", to");
        } else if (command == "axis") {
            return axis(c, pos[2], pos.size() == 4 ? at(3) : c.step);
        } else if (command == "swing") {
            return swing(c);
        } else if (command == "circle") {
            return circle(c, pos.size() == 3 ? at(2) : c.step);
        } else if (command == "speed") {
            return speed(c, pos.size() == 3 ? pos[2] : "x");
        } else if (command == "test") {
            return test(c);
        }
    } catch (const std::exception&) {
        printf("Invalid number in the arguments\n");
    }
    return -1;
}

/**
 * @brief Put the selected leg into a shell command that left it out
 * @return The command as it would be typed on the command line
*/
std::vector<std::string> withSelectedLeg(std::vector<std::string> pos, int selected) {
    const std::vector<std::string> rest(pos.begin() + 1, pos.end());
    const auto fits = fitsAfterLeg(pos[0], rest);
    if (!fits.has_value()) {
        return pos;
    }
    // A leg number first is the leg, as on the command line
    if (!rest.empty() && isIntegerIn(rest[0], 1, 6) && *fitsAfterLeg(pos[0], {rest.begin() + 1, rest.end()})) {
        return pos;
    }
    if (*fits) {
        pos.insert(pos.begin() + 1, std::to_string(selected));
    }
    return pos;
}

void printLegCommandHelp(std::string command) {
    if (command == "exit") {
        command = "quit";
    }
    printCommandHelp(std::string(USAGE) + SHELL_HELP, command);
}

/**
 * @brief Read commands from the terminal and run them until quit or Ctrl+D
 * @param[in] startup_options The program's own options, put before every line's so the line's win
*/
int shell(Session& session, const std::vector<std::string>& startup_options) {
    int selected = 1;
    printf("Leg shell. Type 'help' for the commands, 'help <command>' for one.\n"
        "Ctrl+C stops a running move, Ctrl+D or 'quit' leaves.\n\n");
    while (true) {
        stop_requested = 0;
        std::vector<std::string> words;
        if (!readShellLine(format("test_leg[%s@%d leg=%d]> ", session.port.c_str(), session.baud, selected), words)) {
            return 0;
        }
        std::vector<std::string> all = startup_options;
        all.insert(all.end(), words.begin(), words.end());
        const Args args(all, FLAGS);
        const auto& pos = args.positional();
        if (pos.empty()) {
            continue;
        }
        const std::string& command = pos[0];

        if (command == "quit" || command == "exit") {
            return 0;
        } else if (command == "help") {
            if (pos.size() > 1) {
                printLegCommandHelp(pos[1]);
            } else {
                printf("%s%s", USAGE, SHELL_HELP);
            }
        } else if (command == "select") {
            if (pos.size() == 2 && isIntegerIn(pos[1], 1, 6)) {
                selected = std::stoi(pos[1]);
            } else {
                printf("The leg must be 1-6\n");
            }
        } else if (command == "baud") {
            const int rate = pos.size() == 2 && isIntegerIn(pos[1], 1, 4000000) ? std::stoi(pos[1]) : 0;
            if (rate > 0 && session.bus.open(session.port, rate, session.echo)) {
                session.baud = rate;
            } else {
                printf("Couldn't open %s at that rate. Use one of 1000000 500000 115200 57600 38400.\n",
                    session.port.c_str());
                session.bus.open(session.port, session.baud, session.echo);
            }
            session.bus.setDebug(session.debug);
        } else if (command == "debug") {
            session.debug = parseSwitch(pos, session.debug);
            session.bus.setDebug(session.debug);
            printf("Debug %s\n", session.debug ? "on" : "off");
        } else if (command == "echo") {
            session.echo = parseSwitch(pos, session.echo);
            session.bus.open(session.port, session.baud, session.echo);
            session.bus.setDebug(session.debug);
            printf("Discarding the adapter's echo: %s\n", session.echo ? "on" : "off");
        } else {
            if (runCommand(session, withSelectedLeg(pos, selected), args) < 0) {
                printLegCommandHelp(command);
            }
            printf("\n");
        }
    }
}

/**
 * @brief Open the servo bus and run a command, or the shell if there's none
 * @return 0 if the command succeeded, 1 otherwise
*/
int main(int argc, char** argv) {
    Args args(argc, argv, FLAGS);
    const auto& pos = args.positional();
    if (args.has("help")) {
        printf("%s%s", USAGE, SHELL_HELP);
        return 0;
    }
    const bool shell_mode = pos.empty() || (pos[0] == "shell" && pos.size() == 1);
    if (!shell_mode && (pos.size() < 2 || !fitsAfterLeg(pos[0], {pos.begin() + 2, pos.end()}).value_or(false))) {
        printf("%s", USAGE);
        return 1;
    }

    Session session;
    session.port = args.get("port", "/dev/ttyAMA0");
    session.baud = args.getInt("baud", 1000000);
    session.echo = args.has("echo");
    session.debug = args.has("debug");
    catchCtrlCWithoutRestart();
    if (!session.bus.open(session.port, session.baud, session.echo)) {
        printf("Failed to open %s at %d baud\n", session.port.c_str(), session.baud);
        printf("Is the robot running with hardware:=st3215? It holds the port; stop it or use hardware:=mock.\n");
        return 1;
    }
    session.bus.setDebug(session.debug);

    if (shell_mode) {
        // The options alone, in order, so they can go in front of every line
        std::vector<std::string> startup_options;
        for (int i = 1; i < argc; ++i) {
            const std::string word = argv[i];
            if (word.rfind("--", 0) != 0) {
                continue;
            }
            startup_options.push_back(word);
            const bool flag = word == "--help" ||
                std::find(FLAGS.begin(), FLAGS.end(), word.substr(2)) != FLAGS.end();
            if (!flag && i + 1 < argc) {
                startup_options.push_back(argv[++i]);
            }
        }
        return shell(session, startup_options);
    }
    const int result = runCommand(session, pos, args);
    if (result < 0) {
        printf("\n%s", USAGE);
        return 1;
    }
    return result;
}
