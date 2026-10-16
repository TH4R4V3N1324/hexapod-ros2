#include "hexapod_gait_controller/gait_config.hpp"

namespace hexapod_gait_controller {

/**
 * @brief Get the leg configuration based on the specified gait
 * @param[in] gait The gait type
 * @return A vector of vectors representing the leg configuration
*/
std::vector<std::vector<int>> GaitConfig::getGaitConfig(Gait gait){
    switch (gait){
        case GAIT_TRIPOD:
            return {{1, 3, 5}, {2, 4, 6}};
            break;
        case GAIT_RIPPLE:
            return {{3, 6}, {2, 4}, {1, 5}};
            break;
        case GAIT_WAVE:
            return {{3}, {2}, {1}, {4}, {5}, {6}};
            break;
        default:
            return {};
            break;
    }
}

/**
 * @brief Changes to the next gait when called
*/
void GaitConfig::cycleGait(){
    pendingGait = static_cast<Gait>((currentGait + 1) % NUM_GAITS);
    gaitChangeRequested = true;
}

/**
 * @brief Sets the current gait to the specified gait
 * @param[in] gait The gait to switch to
*/
void GaitConfig::setGait(Gait gait) {
    pendingGait = gait;
    gaitChangeRequested = true;
}

/**
 * @brief Changes to the next mode when called
*/
void GaitConfig::cycleMode() {
    currentMode = static_cast<Mode>((currentMode + 1) % NUM_MODES);
}

/**
 * @brief Sets the current mode to the specified mode
 * @param[in] mode The mode to switch to
*/
void GaitConfig::setMode(Mode mode) {
    currentMode = mode;
}

/**
 * @brief Set the height of the hexapod and update start positions accordingly
 * @param[in] height The new height to set
*/
void GaitConfig::setHeight(double height) {
    currentHeight = height;
    startPos.z() = currentHeight;
    for (auto& [leg, position] : startPosition) {
        position = startPos;
    }
}

};