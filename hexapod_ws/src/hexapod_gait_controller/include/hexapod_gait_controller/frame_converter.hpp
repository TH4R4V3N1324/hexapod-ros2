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
public:
    std::map<int, LegConfig> legConfigs;
    FrameConverter() {
        // Matching embedded config: mounting_angle (rad), mirrored, offset (meters)
        legConfigs[1] = {M_PI / 6.0, false, Vector3d(0.06733, 0.0846, 0)};    // 30°
        legConfigs[2] = {0, false, Vector3d(0.09, 0, 0)};                      // 0°
        legConfigs[3] = {-M_PI / 6.0, false, Vector3d(0.06733, -0.0846, 0)};  // -30°
        legConfigs[4] = {-M_PI / 6.0, true, Vector3d(-0.06733, -0.0846, 0)};  // -30°, mirrored
        legConfigs[5] = {0, true, Vector3d(-0.09, 0, 0)};                      // 0°, mirrored
        legConfigs[6] = {M_PI / 6.0, true, Vector3d(-0.06733, 0.0846, 0)};    // 30°, mirrored
    }
    
    /**
     * Convert position from leg frame to body frame
     * @param position Position in leg frame (m)
     * @param leg_num Leg number (1-6)
     * @return Position in body frame (m)
     */
    Vector3d legToBodyFrame(const Vector3d& position, int leg_num) {
        const auto& config = legConfigs.at(leg_num);
        double angle = config.mounting_angle;
        
        // Rotate from leg frame to body frame (positive rotation)
        double x_body = position.x() * cos(angle) - position.y() * sin(angle);
        double y_body = position.x() * sin(angle) + position.y() * cos(angle);
        
        // Then translate by the coxa offset
        return Vector3d(x_body, y_body, position.z()) + config.translation_offset;
    }
    
    /**
     * Convert position from body frame to leg frame
     * @param position Position in body frame (m)
     * @param leg_num Leg number (1-6)
     * @return Position in leg frame (m)
     */
    Vector3d bodyToLegFrame(const Vector3d& position, int leg_num) {
        const auto& config = legConfigs.at(leg_num);
        double angle = config.mounting_angle;
        
        // First translate to remove coxa offset
        Vector3d translated = position - config.translation_offset;
        
        // Rotate from body frame to leg frame (negative rotation)
        double x_leg = translated.x() * cos(-angle) - translated.y() * sin(-angle);
        double y_leg = translated.x() * sin(-angle) + translated.y() * cos(-angle);
        
        return Vector3d(x_leg, y_leg, translated.z());
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
