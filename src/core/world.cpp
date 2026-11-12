// The frame step and the mechanical solver.  Heat lives in thermal.cpp, the
// liquid in fluid.cpp, editing and spawning in scene.cpp.

#include "world.h"

#include "collision.h"
#include "world_detail.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <unordered_map>

using namespace detail;

namespace {
// Extra separation speed (cm/frame) a contact may add when it pushes apart
// bodies that were already overlapping (e.g. freshly spawned clusters).
constexpr double kPopLimit = 1.5;
// Corners this close to a wall (cm) share in the friction of a box resting on it.
constexpr double kContactSlop = 0.3;
} // namespace

// ============================================================
// Frame step
// ============================================================

void World::step() {
    if (fluid_rest_density_ <= 0.0) init_fluid_constants();
    const Vec3 size(width, height, depth);

    update_containers();
    hash_.build(objects, kFramePairMargin, kCellSize, size);
    build_fluid();
    update_thermal_environment();
    conduct_heat();
    mix_gas_temperatures();
    for (auto& p : pistons) p.update_gas(ambient_temperature);
    if (process_phase_events()) build_fluid();
    build_water_surface();
    measure_immersion();
    update_ice_waterlines();
    thermostat_gases();

    const double h = 1.0 / kSubsteps;
    h_ = h;
    if (dragged_) drag_start_ = dragged_->position;
    for (int s = 0; s < kSubsteps; ++s) {
        compute_forces(h);
        integrate(h, double(s + 1) / kSubsteps);
        hash_.build(objects, kSubstepPairMargin, kCellSize, size);
        solve_positions();
        derive_velocities(h);
        solve_velocities(h);
    }

    std::vector<PhysicsObject*> to_delete;
    for (auto& ptr : objects) {
        PhysicsObject* o = ptr.get();
        if (!is_valid(o->position) || !is_valid(o->velocity) || !is_valid(o->temperature) ||
            !is_valid(o->orientation) || !is_valid(o->angular_velocity)) {
            to_delete.push_back(o);   // never let one bad body poison the scene
            continue;
        }
        if (o->fuse > 0 && --o->fuse == 0) {
            detonate_bomb(o);
            to_delete.push_back(o);
        }
    }
    for (PhysicsObject* o : to_delete)
        if (index_of(o) >= 0) remove_object(o);
    break_ice_bonds();
}

// ============================================================
// Substep: forces and integration
// ============================================================

void World::compute_forces(double h) {
    const double air = kAirDensity * air_density_scale;
    for (auto& ptr : objects) {
        PhysicsObject* o = ptr.get();
        o->acceleration = Vec3();
        o->angular_acceleration = Vec3();
        o->on_axle = false;
        if (o->is_static || o->is_dragged) continue;
        if (!o->is_gas) o->acceleration += gravity;
        // Aerodynamic drag on rigid bodies: F = ½ ρ C_d A v².
        if (air > 0.0 && !o->is_particle() && !o->is_container_piston) {
            const double speed = o->velocity.length();
            if (speed > 1e-6) {
                const double cd = o->shape_type == ShapeType::BOX ? 1.05 : 0.47;
                const double rate = 0.5 * air * cd * o->cross_section() * speed / o->mass;   // per frame
                o->acceleration -= o->velocity * std::min(rate, 0.5 / h);
            }
        }
    }
    for (const auto& s : springs) s.apply_forces();
    for (const auto& p : pistons) p.apply_forces();
    for (const auto& a : axles) a.drive(kMotorAcceleration * h);

    // Vessel pistons: gas pressure below vs. atmosphere above, plus a
    // guide-friction dashpot.  Their own weight is the gravity above.
    for (auto& c : containers) {
        PhysicsObject* face = c.piston;
        if (!face || face->is_dragged) continue;
        const double pressure_force = (c.pressure - thermo::kAtmosphere) * c.bore_area() * kPistonForceScale;
        face->acceleration.y += pressure_force / face->mass - face->velocity.y * kPistonDamping;
    }

    apply_hydrostatics(h);
}

// Per frame: a height map of the water surface (the highest water particle in
// each 10 cm column), kept separately for open water and each vessel.
void World::build_water_surface() {
    surface_cols_ = std::max(1, int(std::ceil(width / kSurfaceColumn)));
    surface_rows_ = std::max(1, int(std::ceil(depth / kSurfaceColumn)));
    surface_tops_.assign(containers.size() + 1, std::vector<double>());
    for (const PhysicsObject* w : fluid_) {
        if (!is_valid(w->position)) continue;
        const std::size_t slot = std::size_t(w->container_index + 1);
        if (slot >= surface_tops_.size()) continue;
        auto& tops = surface_tops_[slot];
        if (tops.empty()) tops.assign(std::size_t(surface_cols_) * std::size_t(surface_rows_), -1e18);
        const int c = std::clamp(int(w->position.x / kSurfaceColumn), 0, surface_cols_ - 1);
        const int r = std::clamp(int(w->position.z / kSurfaceColumn), 0, surface_rows_ - 1);
        double& top = tops[std::size_t(r) * std::size_t(surface_cols_) + std::size_t(c)];
        top = std::max(top, w->position.y);
    }
}

// Level (y of the top of the water) just beside the footprint [x0, x1] × [z0, z1]:
// the median surface height of the columns in a band 14 to 54 cm outside it.
// Measuring beside the body rather than at it avoids the dip a floating body
// presses into the water next to itself.  If there is no water in the band
// (a body that nearly fills its tank) the columns under the body are used.
// NaN if there is no water at all.
double World::water_level_beside(int container, double x0, double x1, double z0, double z1) const {
    const std::size_t slot = std::size_t(container + 1);
    if (slot >= surface_tops_.size() || surface_tops_[slot].empty()) return std::nan("");
    const auto& tops = surface_tops_[slot];
    constexpr double kGap = 14.0, kReach = 40.0;
    auto col = [&](double x) { return std::clamp(int(std::floor(x / kSurfaceColumn)), 0, surface_cols_ - 1); };
    auto row = [&](double z) { return std::clamp(int(std::floor(z / kSurfaceColumn)), 0, surface_rows_ - 1); };
    const int ic0 = col(x0 - kGap), ic1 = col(x1 + kGap), ir0 = row(z0 - kGap), ir1 = row(z1 + kGap);
    const int oc0 = col(x0 - kGap - kReach), oc1 = col(x1 + kGap + kReach);
    const int or0 = row(z0 - kGap - kReach), or1 = row(z1 + kGap + kReach);

    static thread_local std::vector<double> found;
    found.clear();
    for (int r = or0; r <= or1; ++r) {
        for (int c = oc0; c <= oc1; ++c) {
            if (r >= ir0 && r <= ir1 && c >= ic0 && c <= ic1) continue;      // under / right next to the body
            const double top = tops[std::size_t(r) * std::size_t(surface_cols_) + std::size_t(c)];
            if (top > -1e17) found.push_back(top);
        }
    }
    if (found.empty()) {
        for (int r = ir0; r <= ir1; ++r)
            for (int c = ic0; c <= ic1; ++c) {
                const double top = tops[std::size_t(r) * std::size_t(surface_cols_) + std::size_t(c)];
                if (top > -1e17) found.push_back(top);
            }
    }
    if (found.empty()) return std::nan("");
    std::nth_element(found.begin(), found.begin() + std::ptrdiff_t(found.size() / 2), found.end());
    return found[found.size() / 2] + kWaterContactRadius;           // top of the water, not particle centres
}

// Per frame: which water touches each floating body (from the frame's broad
// phase, whose margin covers the ring), its waterline and mean velocity.
void World::measure_immersion() {
    immersion_.assign(objects.size(), Immersion());
    immersed_pairs_.clear();
    for (const auto& [i, j] : hash_.pairs()) {
        PhysicsObject* a = objects[i].get();
        PhysicsObject* b = objects[j].get();
        if (a->is_fluid == b->is_fluid) continue;
        const bool a_is_water = a->is_fluid;
        const PhysicsObject* water = a_is_water ? a : b;
        const PhysicsObject* solid = a_is_water ? b : a;
        if (!feels_hydrostatics(*solid) || solid->is_ice) continue;
        if (water->container_index != solid->container_index) continue;
        double gap;
        if (solid->shape_type == ShapeType::BOX)
            gap = (water->position - closest_point_on_box(water->position, box_of(*solid))).length();
        else
            gap = (water->position - solid->position).length() - solid->radius;
        if (gap > kHydroRingWidth) continue;
        const std::uint32_t wi = a_is_water ? i : j, si = a_is_water ? j : i;
        Immersion& im = immersion_[si];
        ++im.count;
        im.water_velocity += water->velocity;
        im.viscosity += thermo::water_viscosity_ratio(water->temperature);
        immersed_pairs_.emplace_back(si, wi);
    }
    for (std::size_t s = 0; s < immersion_.size(); ++s) {
        PhysicsObject& body = *objects[s];
        if (!feels_hydrostatics(body) || body.is_ice) continue;
        const double ex = body.support(Vec3(1, 0, 0)), ez = body.support(Vec3(0, 0, 1));
        const double measured = water_level_beside(body.container_index, body.position.x - ex, body.position.x + ex,
                                                   body.position.z - ez, body.position.z + ez);
        // The level is low-pass filtered (~1/3 s).  A body's own bobbing
        // sends ripples past the measuring window; following them instantly
        // would let the body pump energy into itself and never settle.
        if (!is_valid(measured)) body.has_water_level = false;
        else if (!body.has_water_level) { body.water_level = measured; body.has_water_level = true; }
        else body.water_level += (measured - body.water_level) * kWaterLevelSmoothing;
        immersion_[s].level = body.has_water_level ? body.water_level : std::nan("");
    }
}

// Archimedes and drag for rigid bodies in water.  A rigid body pushes water
// out of its way one-sidedly (water never props it up through contacts, which
// would let the particle lattice jam and hold up even a steel ball), and
// instead receives the hydrostatic force explicitly:
//     F = rho_water · V_submerged · g   (upwards)
// V_submerged is the part of the body below the water level just beside it
// (see water_level_beside), so the force varies smoothly as it bobs.  A box
// is split into eight blocks that are each weighed separately, which gives
// the righting torque that makes it float flat.  Drag relaxes the body's
// velocity towards the surrounding water's, and the opposite momentum goes
// to that water.
void World::apply_hydrostatics(double h) {
    // Ice: Archimedes from the floe's waterline (see update_ice_waterlines).
    // A fully submerged ice particle displaces 1/kIceRelativeDensity of its
    // own mass of water.
    for (std::size_t s = 0; s < ice_submersion_.size() && s < objects.size(); ++s) {
        if (ice_submersion_[s] <= 0.0) continue;
        PhysicsObject* ice = objects[s].get();
        if (!ice->is_ice || ice->is_dragged) continue;
        ice->acceleration -= gravity * (ice_submersion_[s] / kIceRelativeDensity);
        // Form drag on the submerged part of the floe (water is pushed aside
        // by the floe, so this is what slows it down).
        const double drag = (kIceDrag + kIceFormDrag * ice->velocity.length()) * ice_submersion_[s];
        ice->velocity *= 1.0 - std::min(0.5, drag * h);
    }
    if (immersed_pairs_.empty()) return;

    const double viscosity_scale = fluid_viscosity / 0.05;
    for (std::size_t s = 0; s < immersion_.size() && s < objects.size(); ++s) {
        Immersion& im = immersion_[s];
        if (im.count == 0) continue;
        PhysicsObject* solid = objects[s].get();
        if (!feels_hydrostatics(*solid)) continue;

        double fraction = 0.0;
        if (is_valid(im.level)) {
            if (solid->shape_type == ShapeType::BOX) {
                const Vec3 quarter = solid->half_extents * 0.5;
                const double e = 0.5 * solid->support(Vec3(0, 1, 0));
                const double block = PhysicsObject::kWaterDensity * solid->volume() / 8.0;
                for (int k = 0; k < 8; ++k) {
                    const Vec3 offset = solid->orientation.rotate(Vec3((k & 1) ? quarter.x : -quarter.x,
                                                                       (k & 2) ? quarter.y : -quarter.y,
                                                                       (k & 4) ? quarter.z : -quarter.z));
                    const double f = clampd((im.level - (solid->position.y + offset.y - e)) / (2.0 * e), 0.0, 1.0);
                    if (f <= 0.0) continue;
                    solid->apply_force_at(gravity * (-block * f), offset);
                    fraction += f / 8.0;
                }
            } else {
                const double r = solid->radius;
                const double d = clampd(im.level - (solid->position.y - r), 0.0, 2.0 * r);
                const double wet = kPi * d * d * (3.0 * r - d) / 3.0;       // spherical cap
                fraction = wet / solid->volume();
                solid->acceleration -= gravity * (PhysicsObject::kWaterDensity * wet / solid->mass);
            }
        }

        const Vec3 water_velocity = im.water_velocity / double(im.count);
        const double visc = clampd(viscosity_scale * im.viscosity / double(im.count), 0.0, 10.0);
        // Linear (viscous) plus quadratic (form) drag: F = ½ ρ C_d A v².
        const double speed = (water_velocity - solid->velocity).length();
        const double per_frame = (kHydroLinearDrag * (0.5 + 0.5 * visc)
                                  + 0.5 * PhysicsObject::kWaterDensity * kHydroFormDrag * speed)
                               * solid->cross_section() / solid->mass;
        const double rate = std::min(0.5, per_frame * fraction * h);
        const Vec3 dv = (water_velocity - solid->velocity) * rate;
        solid->velocity += dv;
        solid->angular_velocity *= 1.0 - rate;
        im.reaction = dv * (-solid->mass / double(im.count));
    }
    // The drag's reaction goes to the water around each body.
    for (const auto& [si, wi] : immersed_pairs_) {
        if (si >= immersion_.size() || wi >= objects.size()) continue;
        PhysicsObject& water = *objects[wi];
        water.velocity += immersion_[si].reaction * water.inv_mass();
    }
}

// Ice floes (sets of ice particles joined by ice bonds) float on the level of
// the water beside them.  Each ice particle below that waterline gets the
// buoyancy of the water it displaces.  Working per floe matters: the inside of
// a floe never touches water, yet it is submerged all the same.
void World::update_ice_waterlines() {
    const std::size_t n = objects.size();
    ice_submersion_.assign(n, 0.0);
    std::vector<int> parent;
    std::unordered_map<const PhysicsObject*, int> index;
    for (std::size_t i = 0; i < n; ++i) {
        if (!objects[i]->is_ice) continue;
        index[objects[i].get()] = int(parent.size());
        parent.push_back(int(parent.size()));
    }
    if (parent.empty()) return;
    std::function<int(int)> find = [&](int x) {
        while (parent[std::size_t(x)] != x) { parent[std::size_t(x)] = parent[std::size_t(parent[std::size_t(x)])]; x = parent[std::size_t(x)]; }
        return x;
    };
    for (const auto& l : rigid_links) {
        if (!l.ice_bond) continue;
        auto a = index.find(l.obj_a), b = index.find(l.obj_b);
        if (a == index.end() || b == index.end()) continue;
        parent[std::size_t(find(a->second))] = find(b->second);
    }

    struct Floe { double x0 = 1e18, x1 = -1e18, z0 = 1e18, z1 = -1e18; int container = -1; double level = 0.0; };
    std::unordered_map<int, Floe> floes;
    std::vector<int> floe_of(n, -1);
    for (std::size_t i = 0; i < n; ++i) {
        const PhysicsObject& o = *objects[i];
        if (!o.is_ice) continue;
        const int root = find(index[&o]);
        floe_of[i] = root;
        Floe& f = floes[root];
        f.x0 = std::min(f.x0, o.position.x); f.x1 = std::max(f.x1, o.position.x);
        f.z0 = std::min(f.z0, o.position.z); f.z1 = std::max(f.z1, o.position.z);
        f.container = o.container_index;
    }
    const double r = kWaterContactRadius;
    for (auto& [root, f] : floes)
        f.level = water_level_beside(f.container, f.x0 - r, f.x1 + r, f.z0 - r, f.z1 + r);
    for (std::size_t i = 0; i < n; ++i) {
        if (floe_of[i] < 0) continue;
        const double level = floes[floe_of[i]].level;
        if (!is_valid(level)) continue;                  // no water beside it
        ice_submersion_[i] = clampd((level - (objects[i]->position.y - r)) / (2.0 * r), 0.0, 1.0);
    }
}

void World::integrate(double h, double drag_fraction) {
    for (auto& ptr : objects) {
        PhysicsObject* o = ptr.get();
        o->previous_position = o->position;
        o->previous_orientation = o->orientation;
        if (o->is_dragged) {
            // Kinematic: follow the mouse along a straight path over the frame.
            // (Static bodies such as pins and heaters can be moved this way too.)
            o->position = drag_start_ + (drag_target_ - drag_start_) * drag_fraction;
            o->pre_velocity = (o->position - o->previous_position) / h;
            o->angular_velocity = o->pre_angular_velocity = Vec3();
            continue;
        }
        if (o->is_static) {
            o->velocity = o->pre_velocity = Vec3();
            o->angular_velocity = o->pre_angular_velocity = Vec3();
            continue;
        }

        o->velocity += o->acceleration * h;
        const double speed = o->velocity.length();
        if (speed > kMaxSpeed) o->velocity *= kMaxSpeed / speed;
        if (o->is_container_piston) o->velocity.x = o->velocity.z = 0.0;
        o->pre_velocity = o->velocity;
        o->position += o->velocity * h;

        if (!o->can_rotate()) {
            o->angular_velocity = o->pre_angular_velocity = Vec3();
            continue;
        }
        Vec3 w = o->angular_velocity + o->angular_acceleration * h;
        if (o->shape_type == ShapeType::BOX) {
            // Gyroscopic term of Euler's equations, in the body frame: I·ẇ = -w × (I·w).
            const Vec3& ii = o->inv_inertia_body;
            if (ii.x > 0.0 && ii.y > 0.0 && ii.z > 0.0) {
                Vec3 wb = o->orientation.inv_rotate(w);
                const Vec3 momentum(wb.x / ii.x, wb.y / ii.y, wb.z / ii.z);
                wb -= wb.cross(momentum).scaled(ii) * h;
                w = o->orientation.rotate(wb);
            }
        }
        if (o->is_motor && !o->on_axle) {
            // A free-standing motor spins about a fixed axis in the room.
            const double step = kMotorAcceleration * h;
            w += o->motor_axis * clampd(o->motor_speed - w.dot(o->motor_axis), -step, step);
        }
        const double spin = w.length();
        if (spin > kMaxSpin) w *= kMaxSpin / spin;
        o->angular_velocity = o->pre_angular_velocity = w;
        o->orientation.integrate(w * h);
    }
}

// ============================================================
// Substep: position solve
// ============================================================

void World::solve_positions() {
    contacts_.clear();
    patches_.clear();
    slams_.clear();
    for (int it = 0; it < kLinkIterations; ++it) {
        for (const auto& l : rigid_links) l.solve();
        for (const auto& p : pulleys) p.solve();
        for (const auto& p : pistons) p.solve_stops();
    }
    for (auto& c : clusters) c.solve();
    solve_collisions();
    solve_fluid_density();
    solve_container_boundaries();
    solve_room_walls();
    apply_patch_friction();
    // Joints get the last word over contacts, so a crowd pressing on a rope,
    // a shape or an ice floe cannot pull it apart.
    for (const auto& l : rigid_links) l.solve();
    for (auto& c : clusters) c.solve();
    for (auto& c : containers) {
        if (!c.piston) continue;
        c.piston->position.x = 0.5 * (c.left + c.right);
        c.piston->position.z = 0.5 * (c.back + c.front);
        c.piston->position.y = clampd(c.piston->position.y, c.piston_min_y, c.piston_max_y);
    }
}

double World::contact_radius(const PhysicsObject& self, const PhysicsObject& other) const {
    if (self.is_fluid) return kWaterContactRadius;
    // Ice packs like the water it froze from (its lattice spacing comes from
    // its bonds), so it touches water and unbonded ice at about water spacing
    // and no water can squeeze into a floe.
    if (self.is_ice && other.is_fluid) return kWaterContactRadius;
    if (self.is_ice && other.is_ice) return 0.6 * self.radius;
    return self.radius;
}

void World::add_contact(PhysicsObject* a, PhysicsObject* b, const Vec3& n, const Vec3& pa, const Vec3& pb,
                        double depth, bool b_fixed) {
    if (depth <= 0.0) return;
    const bool b_moves = b && !b_fixed;
    Vec3 ra = pa - a->position;
    Vec3 rb = b ? pb - b->position : Vec3();
    const double wa = a->generalized_inv_mass(ra, n);
    const double wb = b_moves ? b->generalized_inv_mass(rb, n) : 0.0;
    if (wa + wb == 0.0) return;
    const double lambda = depth / (wa + wb);

    // Liquid: frictionless and inelastic, so the push-out is all there is.
    if (a->is_fluid || (b && b->is_fluid)) {
        a->position += n * (lambda * a->inv_mass());
        if (wb > 0.0) b->apply_position_impulse(n * -lambda, rb);
        // A floating body shoves water aside one-sidedly; the momentum it
        // gives the water is taken from it in the velocity pass.
        if (b && b_fixed && a->is_fluid && b->inv_mass() > 0.0) slams_.push_back({a, b, n, rb, depth});
        return;
    }

    const double mu = b ? std::sqrt(a->friction() * b->friction()) : a->friction();
    const bool a_turns = a->rotates_now(), b_turns = b && b->rotates_now();
    const Vec3 la = a_turns ? a->orientation.inv_rotate(ra) : ra;
    const Vec3 lb = b_turns ? b->orientation.inv_rotate(rb) : rb;

    a->apply_position_impulse(n * lambda, ra);
    if (wb > 0.0) b->apply_position_impulse(n * -lambda, rb);

    if (mu > 0.0) {
        // Static friction (Müller et al. 2020): the contact points' sideways
        // slip over this substep is undone if that takes less than μ·λ.
        if (a_turns) ra = a->orientation.rotate(la);
        if (b_turns) rb = b->orientation.rotate(lb);
        Vec3 slip = (a->position + ra) - (a->previous_position + (a_turns ? a->previous_orientation.rotate(la) : la));
        if (b) slip -= (b->position + rb) - (b->previous_position + (b_turns ? b->previous_orientation.rotate(lb) : lb));
        slip -= n * slip.dot(n);
        const double len = slip.length();
        if (len > 1e-10) {
            const Vec3 t = slip / len;
            const double wt = a->generalized_inv_mass(ra, t) + (b_moves ? b->generalized_inv_mass(rb, t) : 0.0);
            if (wt > 0.0) {
                const double lt = len / wt;
                if (lt <= mu * lambda) {
                    a->apply_position_impulse(t * -lt, ra);
                    if (b_moves) b->apply_position_impulse(t * lt, rb);
                }
            }
        }
    }

    Contact c;
    c.a = a; c.b = b; c.normal = n;
    c.ra = ra; c.rb = rb;
    c.b_fixed = !b_moves;
    c.vn_pre = (a->pre_velocity_at(ra) - (b ? b->pre_velocity_at(rb) : Vec3())).dot(n);
    c.restitution = b ? collision_restitution(*a, *b) : a->effective_restitution();
    c.friction = mu;
    c.lambda = lambda;
    c.friction_used = 0.0;
    contacts_.push_back(c);
}

// A set of contact points sharing one normal (a box face on another box, or
// a box's corners on a wall).  Solving the points one after another as if
// each stood alone lets a resting box creep and rock: the first corner takes
// the whole load and its friction turns the box.  So the normal corrections
// are applied first (re-measuring each point as the earlier ones move the
// body), then the total load is shared equally between the points for
// friction, which is what a flat, evenly loaded face does.
void World::solve_manifold(const Manifold& m) {
    PhysicsObject* a = m.a;
    PhysicsObject* b = m.b;
    const Vec3& n = m.normal;
    const int count = std::min(m.count, 8);
    double lambda[8] = {0.0};
    Vec3 ra[8], rb[8];
    auto measure = [&](int k) {
        ra[k] = a->orientation.rotate(m.la[k]);
        const Vec3 pa = a->position + ra[k];
        if (!b) return m.plane_offset - n.dot(pa);
        rb[k] = b->orientation.rotate(m.lb[k]);
        return (b->position + rb[k] - pa).dot(n);
    };

    double total = 0.0;
    for (int pass = 0; pass < 3; ++pass) {
        for (int k = 0; k < count; ++k) {
            const double depth = measure(k);
            if (depth <= 0.0) continue;
            const double w = a->generalized_inv_mass(ra[k], n) + (b ? b->generalized_inv_mass(rb[k], n) : 0.0);
            if (w == 0.0) continue;
            const double dl = depth / w;
            a->apply_position_impulse(n * dl, ra[k]);
            if (b) b->apply_position_impulse(n * -dl, rb[k]);
            lambda[k] += dl;
            total += dl;
        }
    }
    if (total <= 0.0) return;
    const double share = total / count;
    const double mu = b ? std::sqrt(a->friction() * b->friction()) : a->friction();

    // Friction for the patch is applied once every body has been pushed out
    // of every other (see apply_patch_friction).
    PatchFriction patch;
    patch.a = a; patch.b = b; patch.normal = n;
    for (int k = 0; k < count; ++k) { patch.la += m.la[k]; patch.lb += m.lb[k]; }
    patch.la = patch.la / double(count);
    patch.lb = patch.lb / double(count);
    patch.load = total;
    patch.friction = mu;
    patch.used = 0.0;
    patch.first_contact = contacts_.size();
    patch.count = count;
    patches_.push_back(patch);

    const double restitution = b ? collision_restitution(*a, *b) : a->effective_restitution();
    for (int k = 0; k < count; ++k) {
        measure(k);
        Contact c;
        c.a = a; c.b = b; c.normal = n;
        c.ra = ra[k]; c.rb = b ? rb[k] : Vec3();
        c.b_fixed = !b;
        c.vn_pre = (a->pre_velocity_at(ra[k]) - (b ? b->pre_velocity_at(rb[k]) : Vec3())).dot(n);
        c.restitution = restitution;
        c.friction = mu;
        c.lambda = share;
        c.friction_used = 0.0;
        contacts_.push_back(c);
    }
}

// Friction at position level for each contact patch (a box face on another
// box or on a wall): the sideways slip of the patch's centre over this
// substep is undone, up to μ times the load (Coulomb's cone).  Inside the
// cone that is static friction, and a resting box or stack does not creep;
// at the limit it is kinetic friction, and what was spent here is taken off
// the velocity pass's budget.
//
// Two things keep stacks still.  The slip is measured only after all the
// push-outs of the substep: while they are in progress a box is briefly
// turned by the load above it before the floor turns it back, and friction
// must not chase that.  And the correction is a pure translation: on a flat
// face the torque of friction is carried by a shift of the normal load, not
// by turning the box.  (The velocity pass applies friction per point, so a
// box that is really sliding is still tipped by it.)
void World::apply_patch_friction() {
    // A few sweeps: in a stack, holding one face still nudges the faces below and above it.
    for (int sweep = 0; sweep < 3; ++sweep) {
        for (PatchFriction& patch : patches_) {
            PhysicsObject* a = patch.a;
            PhysicsObject* b = patch.b;
            const double room = patch.friction * patch.load - patch.used;
            if (room <= 0.0) continue;
            Vec3 slip = (a->position + a->orientation.rotate(patch.la))
                      - (a->previous_position + a->previous_orientation.rotate(patch.la));
            if (b) slip -= (b->position + b->orientation.rotate(patch.lb))
                         - (b->previous_position + b->previous_orientation.rotate(patch.lb));
            slip -= patch.normal * slip.dot(patch.normal);
            const double len = slip.length();
            const double wa = a->inv_mass(), wb = b ? b->inv_mass() : 0.0;
            if (len < 1e-12 || wa + wb <= 0.0) continue;
            const double step = std::min(len / (wa + wb), room);
            const Vec3 push = slip * (step / len);
            a->position -= push * wa;
            if (b) b->position += push * wb;
            patch.used += step;
        }
    }
    for (const PatchFriction& patch : patches_)
        for (int k = 0; k < patch.count; ++k)
            contacts_[patch.first_contact + std::size_t(k)].friction_used = patch.used / patch.count;
}

void World::collide_pair(PhysicsObject* a, PhysicsObject* b) {
    if (a->group != 0 && a->group == b->group) return;
    // Vessel interiors are sealed analytically (solve_container_boundaries):
    // a body inside a chamber touches nothing outside it, walls included.
    if (a->container_index != b->container_index) return;
    if (a->inv_mass() + b->inv_mass() == 0.0) return;
    if (b->is_fluid) std::swap(a, b);       // a liquid particle, if any, is 'a'
    const bool a_box = a->shape_type == ShapeType::BOX, b_box = b->shape_type == ShapeType::BOX;

    if (a_box && b_box) {
        const double reach = a->radius + b->radius;
        if ((a->position - b->position).length_sq() >= reach * reach) return;
        Vec3 n;
        ManifoldPoint points[8];
        const int count = collide_box_box(box_of(*a), box_of(*b), n, points);
        if (count == 0) return;
        Manifold m;
        m.a = a; m.b = b; m.normal = n; m.count = count;
        for (int k = 0; k < count; ++k) {
            m.la[k] = a->orientation.inv_rotate(points[k].on_a - a->position);
            m.lb[k] = b->orientation.inv_rotate(points[k].on_b - b->position);
        }
        solve_manifold(m);
        return;
    }

    Vec3 n, pa, pb;
    double depth = 0.0;
    if (!a_box && !b_box) {
        const Vec3 delta = a->position - b->position;
        const double ra = contact_radius(*a, *b), rb = contact_radius(*b, *a);
        const double d2 = delta.length_sq();
        if (d2 >= (ra + rb) * (ra + rb)) return;
        const double distance = std::sqrt(d2);
        n = distance > 1e-9 ? delta / distance : Vec3(0, 1, 0);
        pa = a->position - n * ra;
        pb = b->position + n * rb;
        depth = ra + rb - distance;
    } else if (!a_box) {
        const double ra = contact_radius(*a, *b);
        if (!collide_sphere_box(a->position, ra, box_of(*b), n, pb, depth)) return;
        pa = a->position - n * ra;
    } else {
        const double rb = contact_radius(*b, *a);
        if (!collide_sphere_box(b->position, rb, box_of(*a), n, pa, depth)) return;
        n = -n;                              // from the sphere (b) towards the box (a)
        pb = b->position + n * rb;
    }
    // Water is pushed out of floating bodies one-sidedly; they feel the
    // water through apply_hydrostatics() instead.
    add_contact(a, b, n, pa, pb, depth, a->is_fluid && feels_hydrostatics(*b));
}

void World::solve_collisions() {
    for (const auto& [i, j] : hash_.pairs()) collide_pair(objects[i].get(), objects[j].get());
}

void World::solve_container_boundaries() {
    for (std::size_t i = 0; i < containers.size(); ++i) {
        GasContainer& c = containers[i];
        if (!c.piston) continue;
        PhysicsObject* face = c.piston;
        const double top = c.top();

        for (const auto& ptr : objects) {
            PhysicsObject* o = ptr.get();
            if (o->is_static || o->is_container_piston || o->is_dragged) continue;
            if (o->container_index != int(i)) {
                // Anything from outside that tunnelled into the sealed chamber is sent back.
                if (o->container_index < 0 && c.in_chamber(o->position) && !c.in_chamber(o->previous_position))
                    o->position = o->previous_position;
                continue;
            }
            // Bodies inside the sealed chamber are hard-constrained to it.
            const bool round = o->shape_type != ShapeType::BOX;
            const double r = o->is_fluid ? kWaterContactRadius : o->radius;
            const double ex = round ? r : o->support(Vec3(1, 0, 0));
            const double ey = round ? r : o->support(Vec3(0, 1, 0));
            const double ez = round ? r : o->support(Vec3(0, 0, 1));
            const Vec3 p = o->position;
            if (p.x < c.left + ex) add_contact(o, nullptr, Vec3(1, 0, 0), p - Vec3(ex, 0, 0), Vec3(), c.left + ex - p.x);
            else if (p.x > c.right - ex) add_contact(o, nullptr, Vec3(-1, 0, 0), p + Vec3(ex, 0, 0), Vec3(), p.x - (c.right - ex));
            if (p.z < c.back + ez) add_contact(o, nullptr, Vec3(0, 0, 1), p - Vec3(0, 0, ez), Vec3(), c.back + ez - p.z);
            else if (p.z > c.front - ez) add_contact(o, nullptr, Vec3(0, 0, -1), p + Vec3(0, 0, ez), Vec3(), p.z - (c.front - ez));

            double min_y = c.bottom + ey, max_y = top - ey;
            if (min_y > max_y) min_y = max_y = 0.5 * (min_y + max_y);
            if (o->position.y < min_y)
                add_contact(o, nullptr, Vec3(0, 1, 0), o->position - Vec3(0, ey, 0), Vec3(), min_y - o->position.y);
            if (o->position.y > max_y) {
                // Gas pressure on the piston is applied analytically, so gas
                // sees the piston as kinematic; liquids and solids push it.
                const bool fixed = o->is_gas || face->is_dragged;
                add_contact(o, face, Vec3(0, -1, 0), o->position + Vec3(0, ey, 0),
                            Vec3(o->position.x, top, o->position.z), o->position.y - max_y, fixed);
            }
        }
    }
}

void World::solve_room_walls() {
    for (auto& ptr : objects) {
        PhysicsObject* o = ptr.get();
        if (o->is_static || o->is_dragged || o->is_container_piston) continue;
        if (o->shape_type == ShapeType::BOX) {
            const double r = o->radius;
            const Vec3& c = o->position;
            if (c.x - r >= 0.0 && c.x + r <= width && c.y - r >= 0.0 && c.y + r <= height &&
                c.z - r >= 0.0 && c.z + r <= depth) continue;
            // Each wall the box reaches gets one manifold of its corners.
            const struct { Vec3 n; double offset; } planes[6] = {
                {Vec3(0, 1, 0), 0.0}, {Vec3(0, -1, 0), -height}, {Vec3(1, 0, 0), 0.0},
                {Vec3(-1, 0, 0), -width}, {Vec3(0, 0, 1), 0.0}, {Vec3(0, 0, -1), -depth}};
            const Vec3& hx = o->half_extents;
            for (const auto& plane : planes) {
                if (plane.n.dot(o->position) - o->support(plane.n) - plane.offset > kContactSlop) continue;
                Manifold m;
                m.a = o; m.b = nullptr; m.normal = plane.n; m.plane_offset = plane.offset; m.count = 0;
                for (int k = 0; k < 8; ++k) {
                    const Vec3 local((k & 1) ? hx.x : -hx.x, (k & 2) ? hx.y : -hx.y, (k & 4) ? hx.z : -hx.z);
                    const Vec3 p = o->position + o->orientation.rotate(local);
                    if (plane.n.dot(p) - plane.offset < kContactSlop) m.la[m.count++] = local;
                }
                if (m.count > 0) solve_manifold(m);
            }
            continue;
        }
        const double r = o->is_fluid ? kWaterContactRadius : o->radius;
        if (o->position.y - r < 0.0)
            add_contact(o, nullptr, Vec3(0, 1, 0), o->position - Vec3(0, r, 0), Vec3(), r - o->position.y);
        else if (o->position.y + r > height)
            add_contact(o, nullptr, Vec3(0, -1, 0), o->position + Vec3(0, r, 0), Vec3(), o->position.y + r - height);
        if (o->position.x - r < 0.0)
            add_contact(o, nullptr, Vec3(1, 0, 0), o->position - Vec3(r, 0, 0), Vec3(), r - o->position.x);
        else if (o->position.x + r > width)
            add_contact(o, nullptr, Vec3(-1, 0, 0), o->position + Vec3(r, 0, 0), Vec3(), o->position.x + r - width);
        if (o->position.z - r < 0.0)
            add_contact(o, nullptr, Vec3(0, 0, 1), o->position - Vec3(0, 0, r), Vec3(), r - o->position.z);
        else if (o->position.z + r > depth)
            add_contact(o, nullptr, Vec3(0, 0, -1), o->position + Vec3(0, 0, r), Vec3(), o->position.z + r - depth);
    }
}

// ============================================================
// Substep: velocities
// ============================================================

void World::derive_velocities(double h) {
    for (auto& ptr : objects) {
        PhysicsObject* o = ptr.get();
        if (o->is_static && !o->is_dragged) continue;
        o->velocity = (o->position - o->previous_position) / h;
        if (o->is_container_piston) o->velocity.x = o->velocity.z = 0.0;
        if (!o->rotates_now()) continue;
        const Quat dq = o->orientation * o->previous_orientation.conjugate();
        const double sign = dq.w >= 0.0 ? 2.0 : -2.0;
        o->angular_velocity = Vec3(dq.x, dq.y, dq.z) * (sign / h);
    }
}

void World::solve_velocities(double h) {
    // Bounces slower than this are treated as resting contact (no jitter).
    const double rest_threshold = 2.0 * gravity.length() * h + 1e-3;
    auto roll = [](PhysicsObject* o, double normal_impulse) {
        // Rolling (and drilling) resistance of a ball: a torque c_rr · N · r against its spin.
        if (o->shape_type != ShapeType::SPHERE || !o->rotates_now()) return;
        const double spin = o->angular_velocity.length();
        if (spin < 1e-9) return;
        const double dw = kRollingResistance * normal_impulse * o->radius * o->inv_inertia_body.x;
        o->angular_velocity *= std::max(0.0, 1.0 - dw / spin);
    };

    // Slamming: a body that hits water gives it momentum (the water was
    // pushed out of its way in the position pass) and loses as much itself.
    // Only the body's own approach counts, so water that is merely resting
    // against a floating body does not hold it up: that is Archimedes' job.
    for (const Slam& s : slams_) {
        const double approach = (s.body->pre_velocity_at(s.r) - s.water->pre_velocity).dot(s.normal);
        if (approach <= 0.0) continue;
        const double w = s.water->inv_mass() + s.body->generalized_inv_mass(s.r, s.normal);
        if (w <= 0.0) continue;
        const double impulse = std::min(s.water->mass * s.depth / h, approach / w);
        s.body->apply_velocity_impulse(s.normal * -impulse, s.r);
    }

    for (const Contact& c : contacts_) {
        PhysicsObject* a = c.a;
        PhysicsObject* b = c.b;
        const Vec3& n = c.normal;
        const bool b_moves = b && !c.b_fixed;
        const double w = a->generalized_inv_mass(c.ra, n) + (b_moves ? b->generalized_inv_mass(c.rb, n) : 0.0);
        if (w == 0.0) continue;

        // ---- Restitution ----
        const double vn = (a->velocity_at(c.ra) - (b ? b->velocity_at(c.rb) : Vec3())).dot(n);
        double target;
        if (c.vn_pre < -rest_threshold) target = -c.restitution * c.vn_pre;
        else if (c.vn_pre < 0.0) target = 0.0;
        else target = std::min(vn, c.vn_pre + kPopLimit);   // only limit pop-out speed
        const double jn = (target - vn) / w;
        a->apply_velocity_impulse(n * jn, c.ra);
        if (b_moves) b->apply_velocity_impulse(n * -jn, c.rb);

        // ---- Dynamic Coulomb friction, bounded by the normal impulse ----
        const double normal_impulse = c.lambda / h + jn;
        if (normal_impulse <= 0.0) continue;
        if (c.friction > 0.0) {
            const Vec3 vrel = a->velocity_at(c.ra) - (b ? b->velocity_at(c.rb) : Vec3());
            const Vec3 vt = vrel - n * vrel.dot(n);
            const double vt_len = vt.length();
            if (vt_len > 1e-9) {
                const Vec3 t = vt / vt_len;
                const double kt = a->generalized_inv_mass(c.ra, t) + (b_moves ? b->generalized_inv_mass(c.rb, t) : 0.0);
                if (kt > 0.0) {
                    const double budget = std::max(0.0, c.friction * normal_impulse - c.friction_used / h);
                    const double jt = std::min(budget, vt_len / kt);
                    a->apply_velocity_impulse(t * -jt, c.ra);
                    if (b_moves) b->apply_velocity_impulse(t * jt, c.rb);
                }
            }
            roll(a, normal_impulse);
            if (b_moves) roll(b, normal_impulse);
        }
    }

    for (const auto& a : axles) a.solve_velocity();
    for (auto& c : containers)
        if (c.piston) c.piston->velocity.x = c.piston->velocity.z = 0.0;
    fluid_velocity_pass(h);
}

double World::collision_restitution(const PhysicsObject& first, const PhysicsObject& second) {
    if (first.is_fluid || second.is_fluid) return 0.04;
    // Each body's own temperature sets its restitution; the softer,
    // lossier body dominates the impact.
    return std::min(first.effective_restitution(), second.effective_restitution());
}

EnergyReport World::energy() const {
    EnergyReport e;
    for (const auto& ptr : objects) {
        const PhysicsObject& o = *ptr;
        if (o.is_static || o.is_gas) continue;
        e.kinetic += o.kinetic_energy();
        e.potential -= o.mass * gravity.dot(o.position);
    }
    return e;
}
