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
static constexpr int MAX_RESOLUTION = 50 + 1; // +1 for inclusive endpoint

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
    Vector3d homePos {0.15, 0, 0};  // X outward, Y lateral, Z vertical (meters)
    Vector3d startPos {0.13, 0, -currentHeight};  // X outward reach (meters)
    Vector3d rotateZ(const Vector3d& v, double degrees);
public:
    std::map<int, Vector3d> startPosition{
        {1, rotateZ(startPos, -15)},
        {2, startPos},
        {3, rotateZ(startPos, 15)},
        {4, rotateZ(startPos, 15)},
        {5, startPos},
        {6, rotateZ(startPos, -15)}
    };
    Gait currentGait = GAIT_TRIPOD;
    Mode currentMode = MODE_NORMAL;
    bool gaitChangeRequested = false;
    Gait pendingGait;
    double currentHeight = 0.10;  // 10cm default height
    double max_velocity = 0.2; // m/s
    double max_stride_length = 0.06; // m
    double max_angular_velocity = 2.0; // rad/s
    std::vector<std::vector<int>> getGaitConfig(Gait gait);
    void cycleGait();
    void setGait(Gait gait);
    void cycleMode();
    void setMode(Mode mode);
    void setHeight(double height);
};

}  // namespace hexapod_gait_controller

#endif // HEXAPOD_GAIT_CONTROLLER_GAIT_CONFIG_HPP