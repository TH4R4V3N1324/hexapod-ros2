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
static const double MAX_STRIDE_LENGTH = 0.06; // m, furthest a foot moves from its neutral position
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
        {"tilt", MODE_TILT}
    };
public:
    /**
     * @brief Get the supported gaits
     * @return The list of supported gaits
    */
    const std::vector<LocomotionOption>& getGaits() const { return gaits; }
    /**
     * @brief Get the supported modes
     * @return The list of supported modes
    */
    const std::vector<LocomotionOption>& getModes() const { return modes; }
    // Neutral stance from the Hexapod_V6.1 stand pose: each foot straight out from its coxa
    // (so the corner feet sit at 45 degrees), 0.177 m out and 0.134 m below the hips
    double currentHeight = -0.134;
    Vector3d homePos {0.177, 0, 0};
    Vector3d startPos {0.177, 0, currentHeight};
    std::map<int, Vector3d> startPosition{
        {1, startPos},
        {2, startPos},
        {3, startPos},
        {4, startPos},
        {5, startPos},
        {6, startPos}
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
    void setReach(double reach);
};

}  // namespace hexapod_gait_controller

#endif // HEXAPOD_GAIT_CONTROLLER_GAIT_CONFIG_HPP