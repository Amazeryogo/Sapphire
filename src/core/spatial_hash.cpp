#include "spatial_hash.h"

#include <algorithm>
#include <cmath>

void SpatialHash::build(const std::vector<std::unique_ptr<PhysicsObject>>& objects,
                        double margin, double cell_size, double world_width, double world_height,
                        bool skip_gas_pairs) {
    pairs_.clear();
    const std::size_t n = objects.size();
    if (n < 2) return;

    // The grid covers the arena plus a border; anything further out is
    // clamped into the edge cells, which stays correct (just slower).
    constexpr double kBorder = 64.0;
    cell_ = cell_size;
    inv_cell_ = 1.0 / cell_size;
    origin_x_ = -kBorder;
    origin_y_ = -kBorder;
    nx_ = std::max(1, int(std::ceil((std::max(world_width, 1.0) + 2.0 * kBorder) / cell_)));
    ny_ = std::max(1, int(std::ceil((std::max(world_height, 1.0) + 2.0 * kBorder) / cell_)));

    boxes_.resize(n);
    cell_start_.assign(std::size_t(nx_) * ny_ + 1, 0);
    const double half_margin = 0.5 * margin;
    for (std::size_t i = 0; i < n; ++i) {
        const PhysicsObject& o = *objects[i];
        const double ext = o.radius + half_margin;
        Box& b = boxes_[i];
        b.gas = skip_gas_pairs && o.is_gas;
        b.x0 = o.position.x - ext; b.x1 = o.position.x + ext;
        b.y0 = o.position.y - ext; b.y1 = o.position.y + ext;
        if (!is_valid(b.x0) || !is_valid(b.y0) || !is_valid(b.x1) || !is_valid(b.y1)) {
            b.cx0 = 0; b.cx1 = -1; b.cy0 = 0; b.cy1 = -1;     // never paired
            continue;
        }
        b.cx0 = cell_x(b.x0); b.cx1 = cell_x(b.x1);
        b.cy0 = cell_y(b.y0); b.cy1 = cell_y(b.y1);
        for (int cy = b.cy0; cy <= b.cy1; ++cy)
            for (int cx = b.cx0; cx <= b.cx1; ++cx)
                ++cell_start_[std::size_t(cy) * nx_ + cx + 1];
    }
    for (std::size_t c = 1; c < cell_start_.size(); ++c) cell_start_[c] += cell_start_[c - 1];
    entries_.resize(cell_start_.back());
    fill_.assign(cell_start_.begin(), cell_start_.end() - 1);
    // Non-skippable bodies first, then skippable gas, so that within each
    // cell gas–gas candidates can be left out without being enumerated.
    for (int pass = 0; pass < 2; ++pass) {
        for (std::size_t i = 0; i < n; ++i) {
            const Box& b = boxes_[i];
            if (b.gas != (pass == 1)) continue;
            for (int cy = b.cy0; cy <= b.cy1; ++cy)
                for (int cx = b.cx0; cx <= b.cx1; ++cx)
                    entries_[fill_[std::size_t(cy) * nx_ + cx]++] = std::uint32_t(i);
        }
        if (pass == 0) cell_split_.assign(fill_.begin(), fill_.end());
    }

    for (int cy = 0; cy < ny_; ++cy) {
        for (int cx = 0; cx < nx_; ++cx) {
            const std::size_t cell = std::size_t(cy) * nx_ + cx;
            const std::uint32_t begin = cell_start_[cell], end = cell_start_[cell + 1];
            const std::uint32_t split = cell_split_[cell];
            for (std::uint32_t p = begin; p < split; ++p) {
                const std::uint32_t i = entries_[p];
                const Box& a = boxes_[i];
                for (std::uint32_t q = p + 1; q < end; ++q) {
                    const std::uint32_t j = entries_[q];
                    const Box& b = boxes_[j];
                    // Report only from the cell owning the overlap's min corner
                    // (cell indices are monotonic in x, so max of indices works).
                    if (std::max(a.cx0, b.cx0) != cx || std::max(a.cy0, b.cy0) != cy) continue;
                    if (a.x1 < b.x0 || b.x1 < a.x0 || a.y1 < b.y0 || b.y1 < a.y0) continue;
                    pairs_.emplace_back(std::min(i, j), std::max(i, j));
                }
            }
        }
    }
}
