// Golden trace test: runs the whole gait controller node through a scripted drive and checks every
// joint command against a recording made from known-good code.
//
// It exists to catch behaviour changes while the gait logic moves into hexapod-gait-core. Nothing
// here is physically realistic; what matters is that the same inputs give the same 18 angles on
// every tick.
//
// - Time: sim time, set by hand to advance exactly TICK_DT per tick, so dt never varies.
// - Servos: perfect. Each command is fed straight back as the joint state for the next tick.
// - Foot forces (some sections): the lowest feet carry the weight evenly, turned into joint
//   efforts with the leg Jacobian, so contact, terrain adaptation and load balancing run.
// - IMU (some sections): a fixed tilt, so walk and tilt leveling run.
//
// Re-record after an intended behaviour change:
//   GAIT_GOLDEN_RECORD=1 colcon test --packages-select hexapod_gait_controller --ctest-args -R golden

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"

#include "gait_core/kinematic_solver.hpp"
#include "hexapod_gait_controller/gait_controller.hpp"

namespace {

constexpr int64_t TICK_NS = 10'000'000;   // 10 ms, the node's default loop_rate of 100 Hz
constexpr int64_t START_NS = 1'000'000'000'000;  // sim time starts at 1000 s
constexpr int CMD_VEL_EVERY = 5;          // ticks between cmd_vel messages (20 Hz, like teleop)
constexpr double WEIGHT = 25.0;           // N, shared by the feet in contact when forces are on
constexpr double CONTACT_BAND = 0.005;    // m, feet this close to the lowest foot carry weight
constexpr double TOLERANCE = 1e-7;        // rad

const std::vector<std::string>& jointNames() {
    static const std::vector<std::string> names = [] {
        std::vector<std::string> n;
        for (int leg = 1; leg <= 6; ++leg) {
            for (const char* joint : {"coxa", "femur", "tibia"}) {
                n.push_back("leg" + std::to_string(leg) + "_" + joint + "_joint");
            }
        }
        return n;
    }();
    return names;
}

struct Row {
    int tick;
    std::string section;
    int published;
    std::array<double, 18> q;
};

/**
 * @brief Drives a GaitController with a fixed clock, perfect servos and optional fake sensors
*/
class Harness {
public:
    Harness() {
        node = std::make_shared<GaitController>();
        rcl_clock_t* clock = node->get_clock()->get_clock_handle();
        bool enabled = false;
        check(rcl_is_enabled_ros_time_override(clock, &enabled), "read the clock's time override");
        if (!enabled) {
            check(rcl_enable_ros_time_override(clock), "enable the clock's time override");
        }
        setTime();

        node->joint_command_observer = [this](const JointState& msg) {
            for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
                command[msg.name[i]] = msg.position[i];
            }
            published = true;
        };

        // Legs laid out flat at their zeros, as the simulation starts
        for (const auto& name : jointNames()) {
            command[name] = 0.0;
        }
        sendJointStates();
    }

    std::shared_ptr<GaitController> node;
    std::vector<Row> rows;

    // Fake sensors, switched on and off by the script
    bool forces = false;
    bool imu = false;
    double imuRoll = 0.0;
    double imuPitch = 0.0;
    double imuYaw = 0.0;

    void cmdVel(double vx, double vy, double wz) {
        cmd = {vx, vy, wz};
    }

    void setGait(uint8_t gait) {
        auto request = std::make_shared<hexapod_interfaces::srv::SetGait::Request>();
        request->gait = gait;
        node->setGaitCallback(request, std::make_shared<hexapod_interfaces::srv::SetGait::Response>());
    }

    void setMode(uint8_t mode) {
        auto request = std::make_shared<hexapod_interfaces::srv::SetMode::Request>();
        request->mode = mode;
        node->setModeCallback(request, std::make_shared<hexapod_interfaces::srv::SetMode::Response>());
    }

    void setHeight(float height) {
        auto request = std::make_shared<hexapod_interfaces::srv::SetHeight::Request>();
        request->height = height;
        node->setHeightCallback(request, std::make_shared<hexapod_interfaces::srv::SetHeight::Response>());
    }

    void sit(bool down) {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = down;
        node->sitCallback(request, std::make_shared<std_srvs::srv::SetBool::Response>());
    }

    void bodyPose(double roll, double pitch, double yaw, double x, double y, double z) {
        auto msg = std::make_shared<geometry_msgs::msg::Pose>();
        msg->position.x = x;
        msg->position.y = y;
        msg->position.z = z;
        quaternion(roll, pitch, yaw, msg->orientation);
        node->bodyPoseCallback(msg);
    }

    /**
     * @brief Run the node for a while, recording every tick under a section name
     * @param[in] section Name shown when a tick in this stretch doesn't match
     * @param[in] seconds How long to run (s)
    */
    void run(const std::string& section, double seconds) {
        const int ticks = static_cast<int>(std::lround(seconds * 1e9 / TICK_NS));
        for (int i = 0; i < ticks; ++i) {
            tick(section);
        }
    }

private:
    int64_t now_ns = START_NS;
    int tickCount = 0;
    std::map<std::string, double> command;
    bool published = false;
    std::array<double, 3> cmd{0.0, 0.0, 0.0};
    gait_core::KinematicSolver ik;

    static void check(rcl_ret_t ret, const char* what) {
        if (ret != RCL_RET_OK) {
            throw std::runtime_error(std::string("failed to ") + what);
        }
    }

    void setTime() {
        check(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(), now_ns), "set the clock");
    }

    rclcpp::Time stamp() const {
        return rclcpp::Time(now_ns, RCL_ROS_TIME);
    }

    static void quaternion(double roll, double pitch, double yaw, geometry_msgs::msg::Quaternion& q) {
        const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
        const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
        const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
    }

    void tick(const std::string& section) {
        now_ns += TICK_NS;
        setTime();

        if (tickCount % CMD_VEL_EVERY == 0) {
            auto msg = std::make_shared<geometry_msgs::msg::Twist>();
            msg->linear.x = cmd[0];
            msg->linear.y = cmd[1];
            msg->angular.z = cmd[2];
            node->cmdVelCallback(msg);
        }
        if (imu) {
            auto msg = std::make_shared<sensor_msgs::msg::Imu>();
            msg->header.stamp = stamp();
            quaternion(imuRoll, imuPitch, imuYaw, msg->orientation);
            node->imuCallback(msg);
        }

        published = false;
        node->gaitTimerCallback();

        Row row{tickCount, section, published ? 1 : 0, {}};
        for (size_t i = 0; i < jointNames().size(); ++i) {
            row.q[i] = command.at(jointNames()[i]);
        }
        rows.push_back(row);

        sendJointStates();
        ++tickCount;
    }

    /**
     * @brief Report the last commanded angles as the measured joint states, with efforts if forces are on
    */
    void sendJointStates() {
        auto msg = std::make_shared<JointState>();
        msg->header.stamp = stamp();
        for (const auto& name : jointNames()) {
            msg->name.push_back(name);
            msg->position.push_back(command.at(name));
        }

        if (forces) {
            std::array<gait_core::JointAngles, 7> angles;
            std::array<double, 7> footZ{};
            double lowest = 1e9;
            for (int leg = 1; leg <= 6; ++leg) {
                const size_t i = (leg - 1) * 3;
                angles[leg] = {msg->position[i], msg->position[i + 1], msg->position[i + 2]};
                footZ[leg] = ik.solveFK(angles[leg]).z();
                lowest = std::min(lowest, footZ[leg]);
            }
            int loaded = 0;
            for (int leg = 1; leg <= 6; ++leg) {
                loaded += footZ[leg] <= lowest + CONTACT_BAND ? 1 : 0;
            }
            for (int leg = 1; leg <= 6; ++leg) {
                const double fz = footZ[leg] <= lowest + CONTACT_BAND ? WEIGHT / loaded : 0.0;
                // The servos hold the leg still against the ground's push: torques = -J^T force
                const Eigen::Vector3d torques = -ik.jacobian(angles[leg]).transpose() * Eigen::Vector3d(0.0, 0.0, fz);
                for (int j = 0; j < 3; ++j) {
                    msg->effort.push_back(torques[j]);
                }
            }
        }

        node->jointStateCallback(msg);
    }
};

/**
 * @brief The scripted drive: every mode, gait and feature the controller has, in turn
*/
std::vector<Row> runScript() {
    Harness h;

    h.run("startup", 3.0);
    h.sit(false);
    h.run("stand up", 5.0);

    h.cmdVel(0.10, 0.0, 0.0);
    h.run("tripod forward", 4.0);
    h.cmdVel(-0.10, 0.0, 0.0);
    h.run("tripod backward", 3.0);
    h.cmdVel(0.0, 0.0, 0.0);
    h.run("stop and idle return", 3.5);

    h.cmdVel(0.06, 0.05, 0.0);
    h.run("normal mode turn from linear.y", 3.0);
    h.cmdVel(0.0, 0.0, 0.4);
    h.run("normal mode turn on the spot", 3.0);

    h.setMode(1);
    h.cmdVel(0.0, 0.08, 0.0);
    h.run("strafe left", 3.0);
    h.cmdVel(0.07, -0.05, 0.3);
    h.run("strafe combined", 3.0);
    h.cmdVel(0.0, 0.0, 0.0);
    h.run("strafe stop", 1.0);
    h.setMode(0);

    h.setGait(1);
    h.cmdVel(0.08, 0.0, 0.0);
    h.run("ripple forward", 4.0);
    h.setGait(2);
    h.run("wave forward", 6.0);
    h.setGait(0);
    h.run("back to tripod", 3.0);
    h.cmdVel(0.0, 0.0, 0.0);
    h.run("stop", 1.0);

    h.setHeight(-0.10f);
    h.run("lower body", 2.0);
    h.cmdVel(0.08, 0.0, 0.0);
    h.run("walk low", 3.0);
    h.cmdVel(0.0, 0.0, 0.0);
    h.setHeight(-0.134f);
    h.run("raise body", 2.5);

    h.forces = true;
    h.run("forces standing", 1.5);
    h.cmdVel(0.10, 0.0, 0.0);
    h.run("forces walking", 4.0);
    h.cmdVel(0.0, 0.0, 0.0);
    h.run("forces stop", 3.0);

    h.imu = true;
    h.imuRoll = 0.05;
    h.imuPitch = -0.04;
    h.run("walk leveling standing", 2.0);
    h.cmdVel(0.08, 0.0, 0.0);
    h.run("walk leveling walking", 3.0);
    h.cmdVel(0.0, 0.0, 0.0);
    h.run("walk leveling stop", 1.0);

    h.setMode(2);
    h.run("enter tilt", 1.0);
    h.bodyPose(0.10, 0.05, 0.08, 0.02, -0.01, 0.01);
    h.run("tilt pose", 2.5);
    h.imuRoll = 0.08;
    h.imuYaw = 0.1;
    h.run("tilt leveling", 2.0);
    h.bodyPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    h.run("tilt neutral", 1.5);
    h.setMode(0);
    h.run("exit tilt", 2.0);

    h.imu = false;
    h.forces = false;
    h.run("sensors off", 1.0);

    h.sit(true);
    h.run("sit down", 6.0);
    h.sit(false);
    h.run("stand up again", 6.0);

    return std::move(h.rows);
}

void writeTrace(const std::string& path, const std::vector<Row>& rows) {
    std::ofstream out(path);
    ASSERT_TRUE(out) << "can't write " << path;
    out << "tick,section,published";
    for (const auto& name : jointNames()) {
        out << "," << name;
    }
    out << "\n";
    char buffer[32];
    for (const auto& row : rows) {
        out << row.tick << "," << row.section << "," << row.published;
        for (double q : row.q) {
            std::snprintf(buffer, sizeof(buffer), ",%.12g", q);
            out << buffer;
        }
        out << "\n";
    }
}

std::vector<Row> readTrace(const std::string& path) {
    std::vector<Row> rows;
    std::ifstream in(path);
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
        std::stringstream fields(line);
        std::string field;
        Row row{};
        std::getline(fields, field, ',');
        row.tick = std::stoi(field);
        std::getline(fields, row.section, ',');
        std::getline(fields, field, ',');
        row.published = std::stoi(field);
        for (double& q : row.q) {
            std::getline(fields, field, ',');
            q = std::stod(field);
        }
        rows.push_back(row);
    }
    return rows;
}

class GoldenTrace : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        // use_sim_time so the node's clock follows the time the harness sets
        const char* argv[] = {"test_golden_trace", "--ros-args", "-p", "use_sim_time:=true"};
        rclcpp::init(4, argv);
    }
    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }
};

}  // namespace

TEST_F(GoldenTrace, JointCommandsMatchTheRecording) {
    const std::vector<Row> actual = runScript();
    ASSERT_GT(actual.size(), 0u);

    const char* record = std::getenv("GAIT_GOLDEN_RECORD");
    if (record != nullptr && std::string(record) == "1") {
        writeTrace(GOLDEN_TRACE_PATH, actual);
        std::printf("Recorded %zu ticks to %s\n", actual.size(), GOLDEN_TRACE_PATH);
        return;
    }

    const std::vector<Row> expected = readTrace(GOLDEN_TRACE_PATH);
    ASSERT_FALSE(expected.empty()) << "No recording at " << GOLDEN_TRACE_PATH
                                   << ": run once with GAIT_GOLDEN_RECORD=1 on known-good code";
    ASSERT_EQ(actual.size(), expected.size()) << "the script changed: re-record the trace";

    int mismatchedTicks = 0;
    for (size_t t = 0; t < actual.size(); ++t) {
        const Row& a = actual[t];
        const Row& e = expected[t];
        ASSERT_EQ(a.section, e.section) << "the script changed: re-record the trace";
        bool tickOk = a.published == e.published;
        size_t worst = 0;
        double worstError = 0.0;
        for (size_t j = 0; j < a.q.size(); ++j) {
            const double error = std::abs(a.q[j] - e.q[j]);
            if (error > worstError) {
                worstError = error;
                worst = j;
            }
        }
        tickOk = tickOk && worstError <= TOLERANCE;
        if (!tickOk) {
            // Report the first divergence in full; later ticks usually just follow from it
            if (mismatchedTicks == 0) {
                ADD_FAILURE() << "First difference at tick " << a.tick << " (" << a.section << ", t = "
                              << a.tick * TICK_NS / 1e9 << " s): " << jointNames()[worst]
                              << " is " << a.q[worst] << " rad, recorded " << e.q[worst]
                              << " (off by " << worstError << ")"
                              << (a.published != e.published ? "; published flag differs" : "");
            }
            ++mismatchedTicks;
        }
    }
    EXPECT_EQ(mismatchedTicks, 0) << mismatchedTicks << " of " << actual.size() << " ticks differ";
}
