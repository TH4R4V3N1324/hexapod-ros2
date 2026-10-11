#include "hexapod_gait_controller/height_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace hexapod_gait_controller {

/**
 * @brief Create an empty map
 * @param[in] resolution Cell size (m)
*/
HeightMap::HeightMap(double resolution) : resolution_(resolution) {}

/**
 * @brief Hash key of a cell
 * @param[in] ix The cell's x index
 * @param[in] iy The cell's y index
 * @return The key
*/
int64_t HeightMap::key(int ix, int iy) const {
    return (static_cast<int64_t>(ix) << 32) ^ static_cast<int64_t>(static_cast<uint32_t>(iy));
}

/**
 * @brief Cell index of a coordinate
 * @param[in] v The coordinate (m)
 * @return The index
*/
int HeightMap::index(double v) const {
    return static_cast<int>(std::floor(v / resolution_));
}

/**
 * @brief Look up a cell
 * @param[in] ix The cell's x index
 * @param[in] iy The cell's y index
 * @return The cell, or nullptr if nothing has been seen there
*/
const HeightMap::Cell* HeightMap::cell(int ix, int iy) const {
    auto it = cells_.find(key(ix, iy));
    return it == cells_.end() ? nullptr : &it->second;
}

/**
 * @brief Add one frame of points
 * @param[in] points Points on surfaces (x, y in the map frame, z = height, m)
 * @note The points in each cell are averaged, then the cell moves halfway towards that average,
 *       so a single noisy frame can't wreck it but changes show up within a few frames.
*/
void HeightMap::insert(const std::vector<Vector3d>& points) {
    std::unordered_map<int64_t, std::pair<double, int>> frame;
    for (const auto& p : points) {
        auto& acc = frame[key(index(p.x()), index(p.y()))];
        acc.first += p.z();
        acc.second += 1;
    }
    for (const auto& [k, acc] : frame) {
        const double mean = acc.first / acc.second;
        auto it = cells_.find(k);
        if (it == cells_.end()) {
            cells_[k] = Cell{mean, 1};
        } else {
            it->second.height += 0.5 * (mean - it->second.height);
            it->second.hits += 1;
        }
    }
}

/**
 * @brief Drop cells further than radius from a point
 * @param[in] centre The point to keep cells around (m)
 * @param[in] radius How far to keep (m)
*/
void HeightMap::prune(const Vector2d& centre, double radius) {
    const double r2 = radius * radius;
    for (auto it = cells_.begin(); it != cells_.end();) {
        const int ix = static_cast<int>(it->first >> 32);
        const int iy = static_cast<int>(static_cast<int32_t>(it->first & 0xffffffff));
        const Vector2d c((ix + 0.5) * resolution_, (iy + 0.5) * resolution_);
        it = (c - centre).squaredNorm() > r2 ? cells_.erase(it) : std::next(it);
    }
}

/**
 * @brief Forget everything
*/
void HeightMap::clear() {
    cells_.clear();
}

/**
 * @brief Ground height at a point
 * @param[in] point The point (m)
 * @return The height of its cell, or nothing if it hasn't been seen
*/
std::optional<double> HeightMap::height(const Vector2d& point) const {
    const Cell* c = cell(index(point.x()), index(point.y()));
    if (!c) return std::nullopt;
    return c->height;
}

/**
 * @brief Highest ground along a straight path
 * @param[in] from Start of the path (m)
 * @param[in] to End of the path (m)
 * @param[in] radius How far either side of the path to look (m), e.g. the foot's size
 * @return The highest seen cell, or nothing if no cell along the path has been seen
*/
std::optional<double> HeightMap::maxAlong(const Vector2d& from, const Vector2d& to, double radius) const {
    const int steps = std::max(1, static_cast<int>(std::ceil((to - from).norm() / (0.5 * resolution_))));
    const int reach = static_cast<int>(std::ceil(radius / resolution_));
    std::optional<double> best;
    for (int i = 0; i <= steps; ++i) {
        const Vector2d p = from + (to - from) * (static_cast<double>(i) / steps);
        const int cx = index(p.x()), cy = index(p.y());
        for (int dx = -reach; dx <= reach; ++dx) {
            for (int dy = -reach; dy <= reach; ++dy) {
                if (const Cell* c = cell(cx + dx, cy + dy)) {
                    best = best ? std::max(*best, c->height) : c->height;
                }
            }
        }
    }
    return best;
}

/**
 * @brief How uneven the ground is around a point
 * @param[in] point The point (m)
 * @param[in] radius How far around it to look (m)
 * @return Highest minus lowest seen cell within radius, or nothing if fewer than 3 cells are known
*/
std::optional<double> HeightMap::roughness(const Vector2d& point, double radius) const {
    const int reach = static_cast<int>(std::ceil(radius / resolution_));
    const int cx = index(point.x()), cy = index(point.y());
    double low = std::numeric_limits<double>::max(), high = std::numeric_limits<double>::lowest();
    int known = 0;
    for (int dx = -reach; dx <= reach; ++dx) {
        for (int dy = -reach; dy <= reach; ++dy) {
            if (std::hypot(dx, dy) * resolution_ > radius + 1e-9) continue;
            if (const Cell* c = cell(cx + dx, cy + dy)) {
                low = std::min(low, c->height);
                high = std::max(high, c->height);
                ++known;
            }
        }
    }
    if (known < 3) return std::nullopt;
    return high - low;
}

/**
 * @brief Nearest point to put a foot down that isn't next to an edge
 * @param[in] point Where the foot would go (m)
 * @param[in] searchRadius How far it may move (m)
 * @param[in] edgeRadius How much flat ground it needs around it (m)
 * @param[in] maxStep Largest height difference within edgeRadius that still counts as flat (m)
 * @return point itself if it is flat, else the nearest flat cell centre within searchRadius, or
 *         nothing if there isn't one (or the ground there hasn't been seen)
*/
std::optional<Vector2d> HeightMap::flatFoothold(const Vector2d& point, double searchRadius, double edgeRadius,
                                                double maxStep) const {
    auto flat = [&](const Vector2d& p) {
        auto r = roughness(p, edgeRadius);
        return r && *r <= maxStep;
    };
    if (flat(point)) return point;
    const int reach = static_cast<int>(std::ceil(searchRadius / resolution_));
    const int cx = index(point.x()), cy = index(point.y());
    std::optional<Vector2d> best;
    double bestDistance = std::numeric_limits<double>::max();
    for (int dx = -reach; dx <= reach; ++dx) {
        for (int dy = -reach; dy <= reach; ++dy) {
            const Vector2d c((cx + dx + 0.5) * resolution_, (cy + dy + 0.5) * resolution_);
            const double d = (c - point).norm();
            if (d > searchRadius || d >= bestDistance) continue;
            if (flat(c)) {
                best = c;
                bestDistance = d;
            }
        }
    }
    return best;
}

/**
 * @brief Every known cell, for display
 * @return Each cell's centre (x, y) and height (z)
*/
std::vector<Vector3d> HeightMap::cellCentres() const {
    std::vector<Vector3d> out;
    out.reserve(cells_.size());
    for (const auto& [k, c] : cells_) {
        const int ix = static_cast<int>(k >> 32);
        const int iy = static_cast<int>(static_cast<int32_t>(k & 0xffffffff));
        out.emplace_back((ix + 0.5) * resolution_, (iy + 0.5) * resolution_, c.height);
    }
    return out;
}

}  // namespace hexapod_gait_controller
