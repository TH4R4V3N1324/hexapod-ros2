#ifndef HEXAPOD_GAIT_CONTROLLER_GAIT_CONFIG_HPP
#define HEXAPOD_GAIT_CONTROLLER_GAIT_CONFIG_HPP

#include <cstdint>
#include <Eigen/Dense>
#include <map>

using Eigen::Vector3d;
using Eigen::AngleAxisd;
using Eigen::Matrix3d;

namespace hexapod_gait_controller {

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
    Vector3d homePos {0, 150, 0};
    Vector3d startPos {0, 130, -static_cast<double>(currentHeight)};
    std::map<int, Vector3d> startPosition{
        {1, rotateZ(startPos, -15)},
        {2, startPos},
        {3, rotateZ(startPos, 15)},
        {4, rotateZ(startPos, 15)},
        {5, startPos},
        {6, rotateZ(startPos, -15)}
    };
    Gait currentGait = GAIT_TRIPOD;
    Gait pendingGait;
    bool gaitChangeRequested = false;
    Mode currentMode = MODE_NORMAL;
    double currentHeight = 0.0;
    Vector3d rotateZ(const Vector3d& v, double degrees);
public:
    std::vector<std::vector<int>> getGaitConfig(Gait gait);
    void cycleGait();
    void setGait(Gait gait);
    void cycleMode();
    void setMode(Mode mode);
    void setHeight(double height);
};

}  // namespace hexapod_gait_controller

#endif // HEXAPOD_GAIT_CONTROLLER_GAIT_CONFIG_HPP