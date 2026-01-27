#ifndef HEXAPOD_GAIT_CONTROLLER__FRAME_CONVERTER_HPP
#define HEXAPOD_GAIT_CONTROLLER__FRAME_CONVERTER_HPP

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <map>

namespace hexapod_gait_controller {

using Eigen::Vector3d;

struct LegConfig {
    double mounting_angle;      // radians - rotation around Z-axis
    bool mirrored;              // if true, invert Y and angle for mirrored legs
    Vector3d translation_offset; // mm - position of coxa in body frame
};

/**
 * Simple frame converter for leg <-> body transformations
 * Uses only translation (no rotation matrices needed)
 */
class FrameConverter {
private:
    std::map<int, LegConfig> legConfigs;
public:
    FrameConverter() {
        legConfigs[1] = {-M_PI / 4.0, false, Vector3d(67.33, 84.6, 0)};
        legConfigs[2] = {0, false, Vector3d(90, 0, 0)};
        legConfigs[3] = {M_PI / 4.0, false, Vector3d(67.33, -84.6, 0)};
        legConfigs[4] = {3.0 * M_PI / 4.0, true, Vector3d(-67.33, -84.6, 0)};
        legConfigs[5] = {M_PI, true, Vector3d(-90, 0, 0)};
        legConfigs[6] = {-3.0 * M_PI / 4.0, true, Vector3d(-67.33, 84.6, 0)};
    }
    
    /**
     * Convert position from leg frame to body frame
     * @param position Position in leg frame (mm)
     * @param leg_num Leg number (1-6)
     * @return Position in body frame (mm)
     */
    Vector3d legToBodyFrame(const Vector3d& position, int leg_num) {
        return position + legConfigs.at(leg_num).translation_offset;
    }
    
    /**
     * Convert position from body frame to leg frame
     * @param position Position in body frame (mm)
     * @param leg_num Leg number (1-6)
     * @return Position in leg frame (mm)
     */
    Vector3d bodyToLegFrame(const Vector3d& position, int leg_num) {
        return position - legConfigs.at(leg_num).translation_offset;
    }
    
    /**
     * Get the leg configuration for a specific leg
     * @param leg_num Leg number (1-6)
     * @return Reference to leg configuration
     */
    const LegConfig& getLegConfig(int leg_num) const {
        return legConfigs.at(leg_num);
    }
    
    /**
     * Get coxa attachment point in body frame
     * @param leg_num Leg number (1-6)
     * @return Position of coxa joint in body frame (mm)
     */
    Vector3d getCoxaPosition(int leg_num) const {
        return legConfigs.at(leg_num).translation_offset;
    }
};

}  // namespace hexapod_gait_controller

#endif  // HEXAPOD_GAIT_CONTROLLER__FRAME_CONVERTER_HPP
