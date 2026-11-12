#pragma once
// Uniform-grid broad phase in 3D.  Every body is binned into all cells covered
// by its bounding box (grown by margin/2), so two bodies become a candidate
// pair whenever their boxes come within 'margin' of each other on every axis,
// whatever their sizes.  Each pair is reported exactly once (from the cell
// holding the min corner of the two boxes' intersection), in a deterministic
// order.
//
// Gas–gas and liquid–liquid pairs are never reported: gas molecules do not
// collide mechanically (the thermostat models that) and the liquid solver
// keeps its own neighbour list.

#include "physics_object.h"

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

class SpatialHash {
public:
    using Pair = std::pair<std::uint32_t, std::uint32_t>;

    void build(const std::vector<std::unique_ptr<PhysicsObject>>& objects,
               double margin, double cell_size, const Vec3& world_size);

    const std::vector<Pair>& pairs() const { return pairs_; }

private:
    struct Box { double lo[3], hi[3]; int c0[3], c1[3]; int cls; };   // cls: 0 solid, 1 gas, 2 liquid

    int cell(double v, int axis) const {
        const double c = (v - origin_) * inv_cell_;
        return c <= 0.0 ? 0 : (c >= n_[axis] - 1 ? n_[axis] - 1 : int(c));
    }

    double inv_cell_ = 1.0 / 20.0, origin_ = 0.0;
    int n_[3] = {1, 1, 1};
    std::vector<Box> boxes_;
    std::vector<std::uint32_t> cell_start_;
    std::vector<std::uint32_t> entries_;
    std::vector<std::uint32_t> fill_;
    std::vector<std::uint32_t> gas_start_;      // first gas entry of each cell
    std::vector<std::uint32_t> liquid_start_;   // first liquid entry of each cell
    std::vector<Pair> pairs_;
};
