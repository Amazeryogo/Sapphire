// Liquid water: Position Based Fluids (Macklin & Müller 2013) in 3D.
//
// Each substep the particles' density is measured with an SPH kernel and the
// constraint rho_i <= rho_0 is projected (one Jacobi sweep; the small substep
// does the rest, as in Macklin et al. 2019, "Small Steps in Physics
// Simulation").  The constraint is one-sided: water resists compression, but
// a free surface is not sucked together by negative pressure; cohesion does
// that job as surface tension.  Viscosity is XSPH smoothing, and a parcel
// warmer or colder than its surroundings rises or sinks (Boussinesq).
//
// The solver works on flat arrays and a neighbour-pair list that is built
// once per frame with a skin, so the inner loops stay cache-friendly.

#include "world.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kH = World::kFluidH;
constexpr double kH2 = kH * kH;
const double kPoly6 = 315.0 / (64.0 * kPi * std::pow(kH, 9));
const double kSpiky = -45.0 / (kPi * std::pow(kH, 6));
constexpr double kMaxFluidCorrection = 0.15 * World::kFluidSpacing;   // per projection, cm

inline double poly6(double r2) {
    const double d = kH2 - r2;
    return d > 0.0 ? kPoly6 * d * d * d : 0.0;
}
// Gradient of the spiky kernel with respect to particle i, for d = x_i - x_j.
inline Vec3 spiky_gradient(const Vec3& d, double r) {
    const double k = kH - r;
    return d * (kSpiky * k * k / r);
}
} // namespace

void World::init_fluid_constants() {
    // Rest density = kernel sum over the cubic lattice the Water tool spawns.
    double density = 0.0, gradient_sq = 0.0;
    Vec3 gradient_sum;
    for (int i = -3; i <= 3; ++i) {
        for (int j = -3; j <= 3; ++j) {
            for (int k = -3; k <= 3; ++k) {
                const Vec3 r(i * kFluidSpacing, j * kFluidSpacing, k * kFluidSpacing);
                const double len = r.length();
                density += poly6(r.length_sq());
                if (len <= 0.0 || len >= kH) continue;
                const Vec3 g = spiky_gradient(r, len);
                gradient_sum += g;
                gradient_sq += g.length_sq();
            }
        }
    }
    fluid_rest_density_ = density;
    fluid_gradient_norm_ = (gradient_sq + gradient_sum.length_sq()) / (density * density);
}

// Per frame: gathers the liquid particles and their neighbour pairs.
void World::build_fluid() {
    fluid_.clear();
    fluid_index_.clear();
    pair_i_.clear();
    pair_j_.clear();
    double speed_sq = 0.0;
    for (std::size_t i = 0; i < objects.size(); ++i) {
        PhysicsObject* o = objects[i].get();
        if (!o->is_fluid || !is_valid(o->position)) continue;
        fluid_.push_back(o);
        fluid_index_.push_back(std::uint32_t(i));
        speed_sq += o->velocity.length_sq();
    }
    const std::size_t n = fluid_.size();
    f_pos_.resize(n); f_delta_.resize(n); f_gsum_.resize(n); f_vel_.resize(n); f_visc_.resize(n); f_coh_.resize(n);
    f_rho_.resize(n); f_g2_.resize(n); f_lambda_.resize(n); f_rest_.resize(n); f_viscosity_.resize(n);
    f_wsum_.resize(n); f_tsum_.resize(n);
    if (n == 0) { pair_w_.clear(); pair_g_.clear(); return; }

    // Temperature-dependent rest density (the real density curve of water,
    // densest at 4 °C) and viscosity (Vogel equation).
    static const double reference_density = thermo::water_density(20.0);
    for (std::size_t k = 0; k < n; ++k) {
        f_pos_[k] = fluid_[k]->position;
        f_rest_[k] = fluid_rest_density_ * thermo::water_density(fluid_[k]->temperature) / reference_density;
        f_viscosity_[k] = thermo::water_viscosity_ratio(fluid_[k]->temperature);
    }

    // The skin covers how far neighbours can approach during the frame.
    const double rms_speed = std::sqrt(speed_sq / double(n));
    const double skin = clampd(3.0 + 2.0 * rms_speed, 3.0, 12.0);
    const double reach = kH + skin, reach2 = reach * reach;

    // Counting-sort the particles into a grid with cells of size 'reach'.
    Vec3 lo = f_pos_[0], hi = f_pos_[0];
    for (const Vec3& p : f_pos_) {
        lo = Vec3(std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z));
        hi = Vec3(std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z));
    }
    int dims[3];
    for (int a = 0; a < 3; ++a) dims[a] = std::clamp(int((hi[a] - lo[a]) / reach) + 1, 1, 256);
    auto cell_of = [&](const Vec3& p, int c[3]) {
        for (int a = 0; a < 3; ++a) c[a] = std::clamp(int((p[a] - lo[a]) / reach), 0, dims[a] - 1);
    };
    auto cell_id = [&](int x, int y, int z) { return (std::size_t(z) * std::size_t(dims[1]) + std::size_t(y)) * std::size_t(dims[0]) + std::size_t(x); };
    static thread_local std::vector<std::uint32_t> start, order, cursor;
    start.assign(std::size_t(dims[0]) * std::size_t(dims[1]) * std::size_t(dims[2]) + 1, 0);
    for (std::size_t k = 0; k < n; ++k) {
        int c[3]; cell_of(f_pos_[k], c);
        ++start[cell_id(c[0], c[1], c[2]) + 1];
    }
    for (std::size_t c = 1; c < start.size(); ++c) start[c] += start[c - 1];
    cursor.assign(start.begin(), start.end() - 1);
    order.resize(n);
    for (std::size_t k = 0; k < n; ++k) {
        int c[3]; cell_of(f_pos_[k], c);
        order[cursor[cell_id(c[0], c[1], c[2])]++] = std::uint32_t(k);
    }

    for (std::size_t k = 0; k < n; ++k) {
        int c[3]; cell_of(f_pos_[k], c);
        const int container = fluid_[k]->container_index;
        for (int dz = -1; dz <= 1; ++dz) {
            const int z = c[2] + dz;
            if (z < 0 || z >= dims[2]) continue;
            for (int dy = -1; dy <= 1; ++dy) {
                const int y = c[1] + dy;
                if (y < 0 || y >= dims[1]) continue;
                for (int dx = -1; dx <= 1; ++dx) {
                    const int x = c[0] + dx;
                    if (x < 0 || x >= dims[0]) continue;
                    const std::size_t cell = cell_id(x, y, z);
                    for (std::uint32_t q = start[cell]; q < start[cell + 1]; ++q) {
                        const std::uint32_t j = order[q];
                        if (j <= k) continue;
                        // Liquid on opposite sides of a vessel wall does not interact.
                        if (fluid_[j]->container_index != container) continue;
                        if ((f_pos_[k] - f_pos_[j]).length_sq() < reach2) {
                            pair_i_.push_back(std::uint32_t(k));
                            pair_j_.push_back(j);
                        }
                    }
                }
            }
        }
    }
    pair_w_.assign(pair_i_.size(), 0.0);
    pair_g_.assign(pair_i_.size(), Vec3());
}

// Per substep: one projection of the incompressibility constraint.
void World::solve_fluid_density() {
    const std::size_t n = fluid_.size();
    if (n == 0) return;
    const std::size_t pairs = pair_i_.size();
    for (std::size_t k = 0; k < n; ++k) f_pos_[k] = fluid_[k]->position;

    const double w0 = poly6(0.0);
    const double w_dq = poly6(0.04 * kH2);                  // Δq = 0.2 h
    const double relaxation = kFluidRelaxation * fluid_gradient_norm_;
    const double anti_cluster = kFluidAntiCluster / fluid_gradient_norm_;
    const int iterations = n <= 1200 ? 2 : 1;

    for (int iteration = 0; iteration < iterations; ++iteration) {
        std::fill(f_rho_.begin(), f_rho_.end(), w0);
        std::fill(f_g2_.begin(), f_g2_.end(), 0.0);
        std::fill(f_gsum_.begin(), f_gsum_.end(), Vec3());
        for (std::size_t p = 0; p < pairs; ++p) {
            const std::uint32_t i = pair_i_[p], j = pair_j_[p];
            Vec3 d = f_pos_[i] - f_pos_[j];
            double r2 = d.length_sq();
            if (r2 >= kH2) { pair_w_[p] = 0.0; continue; }
            if (r2 < 1e-12) { d = Vec3(0.0, 1e-4, 0.0); r2 = 1e-8; }        // coincident: separate vertically
            const double w = poly6(r2);
            const Vec3 g = spiky_gradient(d, std::sqrt(r2));
            pair_w_[p] = w;
            pair_g_[p] = g;
            f_rho_[i] += w; f_rho_[j] += w;
            f_gsum_[i] += g; f_gsum_[j] -= g;
            const double g2 = g.length_sq();
            f_g2_[i] += g2; f_g2_[j] += g2;
        }
        for (std::size_t k = 0; k < n; ++k) {
            const double rest = f_rest_[k];
            const double ratio = f_rho_[k] / rest;
            fluid_[k]->fluid_density = ratio;
            // One-sided: liquid resists compression, a free surface is not pulled together.
            const double constraint = std::max(0.0, ratio - 1.0);
            f_lambda_[k] = -constraint / ((f_g2_[k] + f_gsum_[k].length_sq()) / (rest * rest) + relaxation);
            f_delta_[k] = Vec3();
        }
        for (std::size_t p = 0; p < pairs; ++p) {
            const double w = pair_w_[p];
            if (w == 0.0) continue;
            const std::uint32_t i = pair_i_[p], j = pair_j_[p];
            double ratio = w / w_dq;
            ratio *= ratio;
            const double s = f_lambda_[i] + f_lambda_[j] - anti_cluster * ratio * ratio;
            if (s == 0.0) continue;
            const Vec3 dp = pair_g_[p] * s;
            f_delta_[i] += dp / f_rest_[i];
            f_delta_[j] -= dp / f_rest_[j];
        }
        for (std::size_t k = 0; k < n; ++k) {
            if (fluid_[k]->is_dragged) continue;
            // A particle squeezed hard (under a body that slams into the
            // water) is let out over a few substeps rather than shot out.
            const double len2 = f_delta_[k].length_sq();
            if (len2 > kMaxFluidCorrection * kMaxFluidCorrection) f_delta_[k] *= kMaxFluidCorrection / std::sqrt(len2);
            f_pos_[k] += f_delta_[k];
        }
    }
    for (std::size_t k = 0; k < n; ++k) fluid_[k]->position = f_pos_[k];
}

// Per substep, after velocities are derived: viscosity (XSPH), thermal
// buoyancy and surface tension.
void World::fluid_velocity_pass(double h) {
    const std::size_t n = fluid_.size();
    if (n == 0) return;
    const std::size_t pairs = pair_i_.size();
    for (std::size_t k = 0; k < n; ++k) {
        f_vel_[k] = fluid_[k]->velocity;
        f_visc_[k] = Vec3();
        f_coh_[k] = Vec3();
        f_wsum_[k] = 0.0;
        f_tsum_[k] = 0.0;
    }
    const double h6 = std::pow(kH, 6);
    for (std::size_t p = 0; p < pairs; ++p) {
        const double w = pair_w_[p];
        if (w == 0.0) continue;
        const std::uint32_t i = pair_i_[p], j = pair_j_[p];
        const Vec3 dv = (f_vel_[j] - f_vel_[i]) * w;
        f_visc_[i] += dv; f_visc_[j] -= dv;
        f_wsum_[i] += w; f_wsum_[j] += w;
        f_tsum_[i] += fluid_[j]->temperature * w;
        f_tsum_[j] += fluid_[i]->temperature * w;

        // Akinci cohesion: repulsive very close, attractive out to h.
        const Vec3 d = f_pos_[i] - f_pos_[j];
        const double r = d.length();
        if (r > 1e-9 && r < kH) {
            const double k = kH - r;
            const double a = k * k * k * r * r * r * 64.0 / h6;
            const double shape = (2.0 * r > kH) ? a : 2.0 * a - 1.0;
            // Weaker in the bulk, stronger at the surface.
            const double correction = 2.0 / std::max(0.2, fluid_[i]->fluid_density + fluid_[j]->fluid_density);
            const Vec3 pull = d * (shape * correction / r);
            f_coh_[i] -= pull;
            f_coh_[j] += pull;
        }
    }
    for (std::size_t k = 0; k < n; ++k) {
        PhysicsObject* o = fluid_[k];
        if (o->is_dragged) continue;
        Vec3 v = f_vel_[k];
        if (f_wsum_[k] > 0.0) {
            // Viscosity follows the real temperature dependence of water.
            const double mu = std::min(0.5, fluid_viscosity * f_viscosity_[k]);
            v += f_visc_[k] * ((1.0 - std::pow(1.0 - mu, h)) / f_wsum_[k]);

            // Boussinesq buoyancy: a parcel lighter than the surrounding
            // water (hotter, or below 4 °C) rises; denser parcels sink.
            const double surrounding = thermo::water_density(f_tsum_[k] / f_wsum_[k]);
            const double own = thermo::water_density(o->temperature);
            v -= gravity * ((surrounding - own) / surrounding * h);
        }
        // Surface tension weakens as water heats up.
        v += f_coh_[k] * (kFluidCohesion * thermo::water_surface_tension_ratio(o->temperature) * h);
        o->velocity = v;
    }
}
