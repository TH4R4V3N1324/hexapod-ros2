#include "hexapod_gait_controller/gait_config.hpp"

namespace hexapod_gait_controller {

// Helper function to rotate a Vector3d around Z axis by degrees
Vector3d GaitConfig::rotateZ(const Vector3d& v, double degrees) {
    double radians = degrees * M_PI / 180.0;
    AngleAxisd rotation(radians, Vector3d::UnitZ());
    return rotation * v;
}

/*
@brief Get the leg configuration based on the specified gait
@param gait The gait type
@return A vector of vectors representing the leg configuration
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

/*
@brief Changes to the next gait when called
*/
void GaitConfig::cycleGait(){
    pendingGait = static_cast<Gait>((currentGait + 1) % NUM_GAITS);
    gaitChangeRequested = true;
}

/*
@brief Sets the current gait to the specified gait
*/
void GaitConfig::setGait(Gait gait) {
    pendingGait = gait;
    gaitChangeRequested = true;
}

/*
@brief Changes to the next mode when called
*/
void GaitConfig::cycleMode() {
    currentMode = static_cast<Mode>((currentMode + 1) % NUM_MODES);
}

/*
@brief Sets the current mode to the specified mode
*/
void GaitConfig::setMode(Mode mode) {
    currentMode = mode;
}

/*
@brief Set the height of the hexapod and update start positions accordingly
@param height The new height to set
*/
void GaitConfig::setHeight(double height) {
    currentHeight = height;
    startPos.z() = -static_cast<double>(currentHeight);
    startPosition = {
        {1, rotateZ(startPos, 15)},
        {2, startPos},
        {3, rotateZ(startPos, -15)},
        {4, rotateZ(startPos, 15)},
        {5, startPos},
        {6, rotateZ(startPos, -15)}
    };
}

};