#pragma once
// Uniform-grid broad phase.  Every body is binned into all cells covered by
// its bounding box (radius + margin/2), so two bodies become a candidate pair
// whenever |dx| and |dy| are both <= r1 + r2 + margin, whatever their sizes.
// Each pair is reported exactly once (from the cell holding the min corner of
// the two boxes' intersection), in a deterministic order.

#include "physics_object.h"

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

class SpatialHash {
public:
    using Pair = std::pair<std::uint32_t, std::uint32_t>;

    // skip_gas_pairs: omit gas–gas pairs (they never collide mechanically).
    void build(const std::vector<std::unique_ptr<PhysicsObject>>& objects,
               double margin, double cell_size, double world_width, double world_height,
               bool skip_gas_pairs = false);

    const std::vector<Pair>& pairs() const { return pairs_; }

private:
    struct Box { double x0, y0, x1, y1; int cx0, cy0, cx1, cy1; bool gas; };

    int cell_x(double x) const {
        const double c = (x - origin_x_) * inv_cell_;
        return c <= 0.0 ? 0 : (c >= nx_ - 1 ? nx_ - 1 : int(c));
    }
    int cell_y(double y) const {
        const double c = (y - origin_y_) * inv_cell_;
        return c <= 0.0 ? 0 : (c >= ny_ - 1 ? ny_ - 1 : int(c));
    }

    double cell_ = 16.0, inv_cell_ = 1.0 / 16.0, origin_x_ = 0.0, origin_y_ = 0.0;
    int nx_ = 1, ny_ = 1;
    std::vector<Box> boxes_;
    std::vector<std::uint32_t> cell_start_;
    std::vector<std::uint32_t> entries_;
    std::vector<std::uint32_t> fill_;
    std::vector<std::uint32_t> cell_split_;   // first skippable-gas entry of each cell
    std::vector<Pair> pairs_;
};
