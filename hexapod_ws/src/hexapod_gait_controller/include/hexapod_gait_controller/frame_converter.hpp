#ifndef HEXAPOD_GAIT_CONTROLLER__FRAME_CONVERTER_HPP
#define HEXAPOD_GAIT_CONTROLLER__FRAME_CONVERTER_HPP

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <map>

namespace hexapod_gait_controller {

using Eigen::Vector3d;

struct LegConfig {
    double yaw;                  // rad, rotation of the leg frame about the body z axis
    Vector3d translation_offset; // m, position of the coxa joint in the body frame
};

/**
 * @brief Converts positions between each leg's frame and the body frame
 * @note The body frame follows REP 103 (+x forward, +y left, +z up) and matches the URDF.
 *       Each leg frame has +x pointing out along the leg from its coxa joint.
*/
class FrameConverter {
public:
    std::map<int, LegConfig> legConfigs;
    /**
     * @brief Initialise the mounting configuration of each leg
    */
    FrameConverter() {
        // Coxa joint yaw (rad) and position (m), matching the URDF: the middle legs 0.11836 m out
        // to the side, the corner legs at 45 degrees from (0.11916, 0.08952) on the slimmer body
        const double m = 0.11836;
        const double cx = 0.11916;
        const double cy = 0.08952;
        legConfigs[1] = {-M_PI / 4.0, Vector3d(cx, -cy, 0)};          // front right
        legConfigs[2] = {-M_PI / 2.0, Vector3d(0.0, -m, 0)};          // middle right
        legConfigs[3] = {-3.0 * M_PI / 4.0, Vector3d(-cx, -cy, 0)};   // rear right
        legConfigs[4] = {3.0 * M_PI / 4.0, Vector3d(-cx, cy, 0)};     // rear left
        legConfigs[5] = {M_PI / 2.0, Vector3d(0.0, m, 0)};            // middle left
        legConfigs[6] = {M_PI / 4.0, Vector3d(cx, cy, 0)};            // front left
    }
    
    /**
     * @brief Convert position from leg frame to body frame
     * @param[in] position Position in leg frame (m)
     * @param[in] leg_num Leg number (1-6)
     * @return Position in body frame (m)
    */
    Vector3d legToBodyFrame(const Vector3d& position, int leg_num) {
        const auto& config = legConfigs.at(leg_num);
        double angle = config.yaw;
        
        // Rotate from leg frame to body frame (positive rotation)
        double x_body = position.x() * cos(angle) - position.y() * sin(angle);
        double y_body = position.x() * sin(angle) + position.y() * cos(angle);
        
        // Then translate by the coxa offset
        return Vector3d(x_body, y_body, position.z()) + config.translation_offset;
    }
    
    /**
     * @brief Convert position from body frame to leg frame
     * @param[in] position Position in body frame (m)
     * @param[in] leg_num Leg number (1-6)
     * @return Position in leg frame (m)
    */
    Vector3d bodyToLegFrame(const Vector3d& position, int leg_num) {
        const auto& config = legConfigs.at(leg_num);
        double angle = config.yaw;
        
        // First translate to remove coxa offset
        Vector3d translated = position - config.translation_offset;
        
        // Rotate from body frame to leg frame (negative rotation)
        double x_leg = translated.x() * cos(-angle) - translated.y() * sin(-angle);
        double y_leg = translated.x() * sin(-angle) + translated.y() * cos(-angle);
        
        return Vector3d(x_leg, y_leg, translated.z());
    }
    
    /**
     * @brief Get the leg configuration for a specific leg
     * @param[in] leg_num Leg number (1-6)
     * @return Reference to leg configuration
    */
    const LegConfig& getLegConfig(int leg_num) const {
        return legConfigs.at(leg_num);
    }
    
    /**
     * @brief Get coxa attachment point in body frame
     * @param[in] leg_num Leg number (1-6)
     * @return Position of coxa joint in body frame (m)
    */
    Vector3d getCoxaPosition(int leg_num) const {
        return legConfigs.at(leg_num).translation_offset;
    }
};

}  // namespace hexapod_gait_controller

#endif  // HEXAPOD_GAIT_CONTROLLER__FRAME_CONVERTER_HPP
