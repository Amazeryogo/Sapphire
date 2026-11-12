#include "spatial_hash.h"

#include <algorithm>
#include <cmath>

void SpatialHash::build(const std::vector<std::unique_ptr<PhysicsObject>>& objects,
                        double margin, double cell_size, const Vec3& world_size) {
    pairs_.clear();
    const std::size_t n = objects.size();
    if (n < 2) return;

    // The grid covers the room plus a border; anything further out is
    // clamped into the edge cells, which stays correct (just slower).
    constexpr double kBorder = 40.0;
    inv_cell_ = 1.0 / cell_size;
    origin_ = -kBorder;
    for (int a = 0; a < 3; ++a)
        n_[a] = std::max(1, int(std::ceil((std::max(world_size[a], 1.0) + 2.0 * kBorder) / cell_size)));
    const std::size_t nx = std::size_t(n_[0]), ny = std::size_t(n_[1]), nz = std::size_t(n_[2]);
    auto cell_index = [&](int cx, int cy, int cz) { return (std::size_t(cz) * ny + std::size_t(cy)) * nx + std::size_t(cx); };

    boxes_.resize(n);
    cell_start_.assign(nx * ny * nz + 1, 0);
    const double half_margin = 0.5 * margin;
    for (std::size_t i = 0; i < n; ++i) {
        const PhysicsObject& o = *objects[i];
        Box& b = boxes_[i];
        b.cls = o.is_gas ? 1 : (o.is_fluid ? 2 : 0);
        Vec3 ext(o.radius, o.radius, o.radius);
        if (o.shape_type == ShapeType::BOX && !o.orientation.is_identity()) {
            ext = Vec3(o.support(Vec3(1, 0, 0)), o.support(Vec3(0, 1, 0)), o.support(Vec3(0, 0, 1)));
        } else if (o.shape_type == ShapeType::BOX) {
            ext = o.half_extents;
        }
        bool ok = is_valid(o.position) && is_valid(ext);
        for (int a = 0; a < 3 && ok; ++a) {
            b.lo[a] = o.position[a] - ext[a] - half_margin;
            b.hi[a] = o.position[a] + ext[a] + half_margin;
            b.c0[a] = cell(b.lo[a], a);
            b.c1[a] = cell(b.hi[a], a);
        }
        if (!ok) {
            for (int a = 0; a < 3; ++a) { b.c0[a] = 0; b.c1[a] = -1; b.lo[a] = 1.0; b.hi[a] = 0.0; }   // never paired
            continue;
        }
        for (int cz = b.c0[2]; cz <= b.c1[2]; ++cz)
            for (int cy = b.c0[1]; cy <= b.c1[1]; ++cy)
                for (int cx = b.c0[0]; cx <= b.c1[0]; ++cx)
                    ++cell_start_[cell_index(cx, cy, cz) + 1];
    }
    for (std::size_t c = 1; c < cell_start_.size(); ++c) cell_start_[c] += cell_start_[c - 1];
    entries_.resize(cell_start_.back());
    fill_.assign(cell_start_.begin(), cell_start_.end() - 1);
    // Solids first, then gas, then liquid, so that same-class particle pairs
    // can be left out without being enumerated.
    for (int pass = 0; pass < 3; ++pass) {
        if (pass == 1) gas_start_.assign(fill_.begin(), fill_.end());
        if (pass == 2) liquid_start_.assign(fill_.begin(), fill_.end());
        for (std::size_t i = 0; i < n; ++i) {
            const Box& b = boxes_[i];
            if (b.cls != pass) continue;
            for (int cz = b.c0[2]; cz <= b.c1[2]; ++cz)
                for (int cy = b.c0[1]; cy <= b.c1[1]; ++cy)
                    for (int cx = b.c0[0]; cx <= b.c1[0]; ++cx)
                        entries_[fill_[cell_index(cx, cy, cz)]++] = std::uint32_t(i);
        }
    }

    for (int cz = 0; cz < n_[2]; ++cz) {
        for (int cy = 0; cy < n_[1]; ++cy) {
            for (int cx = 0; cx < n_[0]; ++cx) {
                const std::size_t c = cell_index(cx, cy, cz);
                const std::uint32_t begin = cell_start_[c], end = cell_start_[c + 1];
                if (end - begin < 2) continue;
                const std::uint32_t gas = gas_start_[c], liquid = liquid_start_[c];
                auto consider = [&](std::uint32_t i, std::uint32_t j) {
                    const Box& a = boxes_[i];
                    const Box& b = boxes_[j];
                    // Report only from the cell owning the overlap's min corner
                    // (cell indices are monotonic, so max of indices works).
                    if (std::max(a.c0[0], b.c0[0]) != cx || std::max(a.c0[1], b.c0[1]) != cy ||
                        std::max(a.c0[2], b.c0[2]) != cz) return;
                    if (a.hi[0] < b.lo[0] || b.hi[0] < a.lo[0] || a.hi[1] < b.lo[1] || b.hi[1] < a.lo[1] ||
                        a.hi[2] < b.lo[2] || b.hi[2] < a.lo[2]) return;
                    pairs_.emplace_back(std::min(i, j), std::max(i, j));
                };
                for (std::uint32_t p = begin; p < gas; ++p)            // solid × everything after it
                    for (std::uint32_t q = p + 1; q < end; ++q) consider(entries_[p], entries_[q]);
                for (std::uint32_t p = gas; p < liquid; ++p)           // gas × liquid
                    for (std::uint32_t q = liquid; q < end; ++q) consider(entries_[p], entries_[q]);
            }
        }
    }
}
