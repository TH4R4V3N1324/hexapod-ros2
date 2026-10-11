#ifndef HEXAPOD_GAIT_CONTROLLER__HEIGHT_MAP_HPP
#define HEXAPOD_GAIT_CONTROLLER__HEIGHT_MAP_HPP

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

namespace hexapod_gait_controller {

using Eigen::Vector2d;
using Eigen::Vector3d;

/**
 * @brief Ground heights on a 2D grid, built from depth camera points
 * @note Cells are keyed by their integer coordinates in a fixed frame (odom), so the map doesn't
 *       move with the robot; cells far from it are dropped with prune(). Each cell keeps a
 *       running estimate of its height that follows new measurements.
*/
class HeightMap {
public:
    explicit HeightMap(double resolution = 0.02);

    void insert(const std::vector<Vector3d>& points);
    void prune(const Vector2d& centre, double radius);
    void clear();

    std::optional<double> height(const Vector2d& point) const;
    std::optional<double> maxAlong(const Vector2d& from, const Vector2d& to, double radius) const;
    std::optional<double> roughness(const Vector2d& point, double radius) const;
    std::optional<Vector2d> flatFoothold(const Vector2d& point, double searchRadius, double edgeRadius,
                                         double maxStep) const;

    double resolution() const { return resolution_; }
    size_t size() const { return cells_.size(); }
    std::vector<Vector3d> cellCentres() const;

private:
    struct Cell {
        double height;
        int hits;
    };
    int64_t key(int ix, int iy) const;
    int index(double v) const;
    const Cell* cell(int ix, int iy) const;

    double resolution_;
    std::unordered_map<int64_t, Cell> cells_;
};

}  // namespace hexapod_gait_controller

#endif  // HEXAPOD_GAIT_CONTROLLER__HEIGHT_MAP_HPP
