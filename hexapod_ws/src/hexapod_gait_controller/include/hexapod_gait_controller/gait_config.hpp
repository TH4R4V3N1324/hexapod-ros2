#ifndef HEXAPOD_GAIT_CONTROLLER_GAIT_CONFIG_HPP
#define HEXAPOD_GAIT_CONTROLLER_GAIT_CONFIG_HPP

#include <cstdint>
#include <Eigen/Dense>
#include <map>

using Eigen::Vector3d;
using Eigen::AngleAxisd;
using Eigen::Matrix3d;

namespace hexapod_gait_controller {

static constexpr int MAX_LEGS = 6;
static constexpr int MAX_RESOLUTION = 50;
static const double MAX_LINEAR_VELOCITY = 0.2; // m/s
static const double MAX_STRIDE_LENGTH = 0.06; // m
static const double MAX_ANGULAR_VELOCITY = 2.0; // rad/s
static const double LIFT_HEIGHT = 0.05;

struct LocomotionOption {
    std::string name;
    uint8_t id;
};

enum Gait : uint8_t {
    GAIT_TRIPOD,
    GAIT_RIPPLE,
    GAIT_WAVE,
    NUM_GAITS
};

enum Mode : uint8_t {
    MODE_NORMAL,
    MODE_STRAFE,
    MODE_TILT,
    MODE_CONFIG,
    NUM_MODES
};

class GaitConfig {
private:
    const std::vector<LocomotionOption> gaits = {
        {"tripod", GAIT_TRIPOD},
        {"ripple", GAIT_RIPPLE},
        {"wave", GAIT_WAVE}
    };
    const std::vector<LocomotionOption> modes = {
        {"normal", MODE_NORMAL},
        {"strafe", MODE_STRAFE},
        {"tilt", MODE_TILT},
        {"config", MODE_CONFIG}
    };
    Vector3d rotateZ(const Vector3d& v, double degrees);
public:
    const std::vector<LocomotionOption>& getGaits() const { return gaits; }
    const std::vector<LocomotionOption>& getModes() const { return modes; }
    double currentHeight = -0.12;
    Vector3d homePos {0.15, 0, 0};
    Vector3d startPos {0.15, 0, currentHeight};
    std::map<int, Vector3d> startPosition{
        {1, rotateZ(startPos, 15)},
        {2, startPos},
        {3, rotateZ(startPos, -15)},
        {4, rotateZ(startPos, 15)},
        {5, startPos},
        {6, rotateZ(startPos, -15)}
    };
    Gait currentGait = GAIT_TRIPOD;
    Mode currentMode = MODE_NORMAL;
    bool gaitChangeRequested = false;
    Gait pendingGait;
    std::vector<std::vector<int>> getGaitConfig(Gait gait);
    void cycleGait();
    void setGait(Gait gait);
    void cycleMode();
    void setMode(Mode mode);
    void setHeight(double height);
};

}  // namespace hexapod_gait_controller

#endif // HEXAPOD_GAIT_CONTROLLER_GAIT_CONFIG_HPP