#include "world.h"

#include "random.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <unordered_map>

namespace {
constexpr double kCellSize = 16.0;
constexpr double kPi = 3.14159265358979323846;
// Extra separation speed (px/frame) a contact may add when it pushes apart
// bodies that were already overlapping (e.g. freshly spawned clusters).
constexpr double kPopLimit = 1.5;
// Water core: radius (× particle radius) and the fraction of an overlap
// removed per substep.
constexpr double kWaterCoreRadius = 1.0;
constexpr double kWaterCoreStiffness = 0.35;

double water_expansion(double celsius) {
    static const double reference = thermo::water_density(20.0);
    return std::cbrt(reference / thermo::water_density(celsius));
}

bool is_solid(const PhysicsObject& o) { return !o.is_fluid && !o.is_gas; }
// Bodies that float or sink by explicit buoyancy (rigid bodies and ice).
bool feels_hydrostatics(const PhysicsObject& o) {
    return is_solid(o) && !o.is_static && !o.is_dragged && !o.is_container_piston;
}
constexpr double kHydroRingWidth = 14.0;  // water this far outside a body counts as touching it

// Area of a body (disc or square of half-size r) below a horizontal waterline
// that lies 'depth' above the body's lowest point.
double submerged_area(const PhysicsObject& body, double depth) {
    const double r = body.radius;
    const double h = clampd(depth, 0.0, 2.0 * r);
    if (body.shape_type == ShapeType::SQUARE) return 2.0 * r * h;
    return r * r * std::acos((r - h) / r) - (r - h) * std::sqrt(std::max(0.0, 2.0 * r * h - h * h));
}
} // namespace

// ============================================================
// Frame step
// ============================================================

void World::step() {
    update_containers();
    hash_.build(objects, kFramePairMargin, kCellSize, width, height);
    update_thermal_environment();
    conduct_heat();
    for (auto& p : pistons) p.update_gas(ambient_temperature);
    expansion_.resize(objects.size());
    viscosity_.resize(objects.size());
    for (std::size_t i = 0; i < objects.size(); ++i) {
        const bool fluid = objects[i]->is_fluid;
        expansion_[i] = fluid ? water_expansion(objects[i]->temperature) : 1.0;
        viscosity_[i] = fluid ? thermo::water_viscosity_ratio(objects[i]->temperature) : 1.0;
    }
    process_phase_events();
    build_water_surface();
    measure_immersion();
    update_ice_waterlines();
    thermostat_gases();

    const double h = 1.0 / kSubsteps;
    if (dragged_) drag_start_ = dragged_->position;
    for (int s = 0; s < kSubsteps; ++s) {
        compute_forces(h);
        integrate(h, double(s + 1) / kSubsteps);
        hash_.build(objects, kSubstepPairMargin, kCellSize, width, height, /*skip_gas_pairs=*/true);
        solve_positions();
        derive_velocities(h);
        solve_velocities(h);
    }

    std::vector<PhysicsObject*> to_delete;
    for (auto& ptr : objects) {
        PhysicsObject* o = ptr.get();
        if (!is_valid(o->position) || !is_valid(o->velocity) || !is_valid(o->temperature)) {
            to_delete.push_back(o);   // never let one bad body poison the scene
            continue;
        }
        if (!o->is_fluid && !o->is_gas && !o->is_ice && !o->is_static) o->update_trail();
        if (o->fuse > 0 && --o->fuse == 0) {
            detonate_bomb(o);
            to_delete.push_back(o);
        }
    }
    for (PhysicsObject* o : to_delete) remove_object(o);
    break_ice_bonds();
}

// ============================================================
// Substep: forces and integration
// ============================================================

void World::compute_forces(double h) {
    for (auto& ptr : objects) {
        PhysicsObject* o = ptr.get();
        o->acceleration = Vector();
        if (!o->is_static && !o->is_dragged && !o->is_gas) o->acceleration += gravity;
    }
    for (const auto& s : springs) s.apply_forces();
    for (const auto& p : pistons) p.apply_forces();

    // Vessel pistons: gas pressure below vs. atmosphere above, plus a
    // guide-friction dashpot.  Their own weight is the gravity above.
    for (auto& c : containers) {
        PhysicsObject* face = c.piston;
        if (!face || face->is_dragged) continue;
        const double pressure_force = (thermo::kAtmosphere - c.pressure) * c.width() * kPistonForceScale;
        face->acceleration.y += pressure_force / face->mass - face->velocity.y * kPistonDamping;
    }

    // Liquid water (SPH-like).  Thermal expansion follows the real density
    // curve (densest at 4 °C); viscosity the Vogel equation.
    for (const auto& [i, j] : hash_.pairs()) {
        PhysicsObject* a = objects[i].get();
        PhysicsObject* b = objects[j].get();
        if (!a->is_fluid || !b->is_fluid) continue;
        if (a->container_index != b->container_index) continue;
        const Vector delta = a->position - b->position;
        const double expansion = 0.5 * (expansion_[i] + expansion_[j]);
        const double smoothing_radius = a->liquid_radius * 2.5 * expansion;
        const double d2 = delta.length_sq();
        if (d2 >= smoothing_radius * smoothing_radius || d2 < 1e-18) continue;
        const double distance = std::sqrt(d2);
        const Vector normal = delta / distance;
        const double q = 1.0 - distance / smoothing_radius;
        const double pressure = q * q * 2.0 / expansion;
        a->acceleration += normal * (pressure / a->mass);
        b->acceleration -= normal * (pressure / b->mass);

        const double viscosity = std::min(0.45, fluid_viscosity * 0.5 * (viscosity_[i] + viscosity_[j]));
        const Vector blend = (b->velocity - a->velocity) * (viscosity * q * h);
        a->velocity += blend;
        b->velocity -= blend;
    }

    apply_hydrostatics(h);
}

// Per frame: a height map of the water surface (the highest water particle in
// each 8 px column), kept separately for open water and each vessel.
void World::build_water_surface() {
    const int columns = std::max(1, int(std::ceil(width / kSurfaceColumn)));
    surface_tops_.assign(containers.size() + 1, std::vector<double>());
    for (const auto& ptr : objects) {
        const PhysicsObject& w = *ptr;
        if (!w.is_fluid || !is_valid(w.position)) continue;
        auto& tops = surface_tops_[std::size_t(w.container_index + 1)];
        if (tops.empty()) tops.assign(std::size_t(columns), 1e18);
        const int c = std::clamp(int(w.position.x / kSurfaceColumn), 0, columns - 1);
        tops[std::size_t(c)] = std::min(tops[std::size_t(c)], w.position.y);
    }
}

// Level (y of the top of the water) just beside the span [x_left, x_right]:
// the median surface height of the columns up to 40 px out on either side.
// Measuring beside the body rather than at it avoids the dip a floating body
// presses into the water next to itself.  NaN if there is no water there.
double World::water_level_beside(int container, double x_left, double x_right) const {
    const std::size_t slot = std::size_t(container + 1);
    if (slot >= surface_tops_.size() || surface_tops_[slot].empty()) return std::nan("");
    const auto& tops = surface_tops_[slot];
    const int columns = int(tops.size());
    constexpr double kGap = 16.0, kReach = 40.0;
    double found[16];
    int n = 0;
    auto take = [&](double from, double to) {
        const int c0 = std::clamp(int(std::floor(from / kSurfaceColumn)), 0, columns - 1);
        const int c1 = std::clamp(int(std::floor(to / kSurfaceColumn)), 0, columns - 1);
        for (int c = c0; c <= c1 && n < 16; ++c)
            if (tops[std::size_t(c)] < 1e17) found[n++] = tops[std::size_t(c)];
    };
    take(x_left - kGap - kReach, x_left - kGap);
    take(x_right + kGap, x_right + kGap + kReach);
    if (n == 0) return std::nan("");
    std::nth_element(found, found + n / 2, found + n);
    return found[n / 2] - kWaterContactRadius;           // top of the water, not particle centres
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
        const double reach = solid->radius + kHydroRingWidth;
        if ((water->position - solid->position).length_sq() > reach * reach) continue;
        const std::uint32_t wi = a_is_water ? i : j, si = a_is_water ? j : i;
        Immersion& im = immersion_[si];
        ++im.count;
        im.water_velocity += water->velocity;
        im.viscosity += viscosity_[wi];
        immersed_pairs_.emplace_back(si, wi);
    }
    for (std::size_t s = 0; s < immersion_.size(); ++s) {
        PhysicsObject& body = *objects[s];
        if (!feels_hydrostatics(body) || body.is_ice) continue;
        const double measured = water_level_beside(body.container_index, body.position.x - body.radius,
                                                   body.position.x + body.radius);
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
// would let the particle lattice jam and hold up even an iron ball), and
// instead receives the hydrostatic force explicitly:
//     F = rho_water · A_submerged · g   (upwards)
// A_submerged is the part of the body's shape below the water level just
// beside it (see water_level_beside), so the force varies smoothly as it bobs.
// Drag relaxes the body's velocity towards the surrounding water's, and the
// opposite momentum goes to that water.
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
        const double drag = (kIceDrag + kHydroQuadraticDrag * ice->velocity.length()) * ice_submersion_[s];
        ice->velocity *= 1.0 - std::min(0.5, drag * h);
    }
    if (immersed_pairs_.empty()) return;

    const double viscosity_scale = fluid_viscosity / 0.05;
    for (std::size_t s = 0; s < immersion_.size(); ++s) {
        Immersion& im = immersion_[s];
        if (im.count == 0) continue;
        PhysicsObject* solid = objects[s].get();
        // Archimedes for the part of the body below the waterline beside it.
        const double area = solid->shape_type == ShapeType::SQUARE
            ? 4.0 * solid->radius * solid->radius
            : kPi * solid->radius * solid->radius;
        const double wet_area = is_valid(im.level)
            ? submerged_area(*solid, solid->position.y + solid->radius - im.level) : 0.0;
        const double fraction = wet_area / area;
        solid->acceleration -= gravity * (kWaterDensity * wet_area / solid->mass);

        const Vector water_velocity = im.water_velocity / double(im.count);
        const double visc = clampd(viscosity_scale * im.viscosity / double(im.count), 0.0, 10.0);
        // Linear (viscous) plus quadratic (form) drag; the quadratic part is
        // what stops a body that is dropped in fast without a huge splash.
        const double speed = (water_velocity - solid->velocity).length();
        const double rate = std::min(0.5, (kHydroDrag * (0.5 + 0.5 * visc) + kHydroQuadraticDrag * speed) * fraction * h);
        const Vector dv = (water_velocity - solid->velocity) * rate;
        solid->velocity += dv;
        solid->angular_velocity *= 1.0 - rate;
        im.reaction = dv * (-solid->mass / double(im.count));
    }
    // The drag's reaction goes to the water around each body.
    for (const auto& [si, wi] : immersed_pairs_) {
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
    std::function<int(int)> find = [&](int x) { return parent[x] == x ? x : parent[x] = find(parent[x]); };
    for (const auto& l : rigid_links) {
        if (!l.ice_bond) continue;
        auto a = index.find(l.obj_a), b = index.find(l.obj_b);
        if (a == index.end() || b == index.end()) continue;
        parent[find(a->second)] = find(b->second);
    }

    struct Floe { double x0 = 1e18, x1 = -1e18; int container = -1; double level = 0.0; };
    std::unordered_map<int, Floe> floes;
    std::vector<int> floe_of(n, -1);
    for (std::size_t i = 0; i < n; ++i) {
        const PhysicsObject& o = *objects[i];
        if (!o.is_ice) continue;
        const int root = find(index[&o]);
        floe_of[i] = root;
        Floe& f = floes[root];
        f.x0 = std::min(f.x0, o.position.x);
        f.x1 = std::max(f.x1, o.position.x);
        f.container = o.container_index;
    }
    for (auto& [root, f] : floes)
        f.level = water_level_beside(f.container, f.x0 - kWaterContactRadius, f.x1 + kWaterContactRadius);
    for (std::size_t i = 0; i < n; ++i) {
        if (floe_of[i] < 0) continue;
        const double level = floes[floe_of[i]].level;
        if (!is_valid(level)) continue;                  // no water beside it
        const PhysicsObject& o = *objects[i];
        const double r = kWaterContactRadius;
        ice_submersion_[i] = clampd((o.position.y - level + r) / (2.0 * r), 0.0, 1.0);
    }
}

void World::integrate(double h, double drag_fraction) {
    const double air_keep = std::pow(1.0 - std::min(air_resistance, 0.99), h);
    for (auto& ptr : objects) {
        PhysicsObject* o = ptr.get();
        o->previous_position = o->position;
        if (o->is_dragged) {
            // Kinematic: follow the mouse along a straight path over the frame.
            // (Static bodies such as pins and heaters can be moved this way too.)
            o->position = drag_start_ + (drag_target_ - drag_start_) * drag_fraction;
            o->pre_velocity = (o->position - o->previous_position) / h;
            continue;
        }
        if (o->is_static) {
            o->velocity = Vector();
            o->pre_velocity = Vector();
            o->angular_velocity = 0.0;
            continue;
        }

        o->velocity += o->acceleration * h;
        if (!o->is_gas) o->velocity *= air_keep;
        const double speed = o->velocity.length();
        if (speed > kMaxSpeed) o->velocity *= kMaxSpeed / speed;
        if (o->is_container_piston) o->velocity.x = 0.0;
        o->pre_velocity = o->velocity;
        o->position += o->velocity * h;

        if (o->can_rotate()) {
            if (o->is_motor) {
                const double step = kMotorAcceleration * h;
                o->angular_velocity += clampd(o->motor_speed - o->angular_velocity, -step, step);
            }
            o->angular_velocity *= air_keep;
            o->angle += o->angular_velocity * h;
            if (std::abs(o->angle) > 1e6) o->angle = std::fmod(o->angle, 2.0 * kPi);
        } else {
            o->angular_velocity = 0.0;
        }
    }
}

// ============================================================
// Substep: position solve
// ============================================================

void World::solve_positions() {
    contacts_.clear();
    for (int it = 0; it < kLinkIterations; ++it) {
        for (const auto& l : rigid_links) l.solve();
        for (const auto& p : pulleys) p.solve();
        for (const auto& p : pistons) p.solve_stops();
    }
    solve_collisions();
    // Joints get the last word over contacts, so a crowd pressing on a rope,
    // a shape or an ice floe cannot pull it apart.
    for (const auto& l : rigid_links) l.solve();
    solve_container_boundaries();
    solve_screen_walls();
    for (auto& c : containers) {
        if (!c.piston) continue;
        c.piston->position.x = 0.5 * (c.left + c.right);
        c.piston->position.y = clampd(c.piston->position.y, c.piston_min_y, c.piston_max_y);
    }
}

double World::contact_radius(const PhysicsObject& self, const PhysicsObject& other) const {
    if (self.is_fluid && !other.is_fluid) return kWaterContactRadius;
    // Ice packs like the water it froze from (its lattice spacing comes from
    // its bonds), so it touches water and unbonded ice at about water spacing
    // and no water can squeeze into a floe.
    if (self.is_ice && other.is_fluid) return kWaterContactRadius;
    if (self.is_ice && other.is_ice) return 0.6 * self.radius;
    return self.radius;
}

void World::add_contact(PhysicsObject* a, PhysicsObject* b, const Vector& normal, double depth,
                        double ra, double rb, const Vector& wall_velocity, double wb_override) {
    const double wa = a->inv_mass();
    const double wb = wb_override >= 0.0 ? wb_override : (b ? b->inv_mass() : 0.0);
    if (wa + wb == 0.0 || depth <= 0.0) return;
    const double lambda = depth / (wa + wb);
    a->position += normal * (lambda * wa);
    if (b) b->position -= normal * (lambda * wb);

    Contact c;
    c.a = a; c.b = b; c.normal = normal;
    c.wall_velocity = b ? Vector() : wall_velocity;
    c.ra = ra; c.rb = rb;
    c.wa = wa; c.wb = wb;
    c.wia = a->inv_inertia();
    c.wib = (b && wb > 0.0) ? b->inv_inertia() : 0.0;
    c.vn_pre = (a->pre_velocity - (b ? b->pre_velocity : wall_velocity)).dot(normal);
    c.restitution = b ? collision_restitution(*a, *b) : a->effective_restitution();
    c.friction = b ? std::sqrt(a->friction() * b->friction()) : a->friction();
    c.lambda = lambda;
    contacts_.push_back(c);
}

void World::solve_collisions() {
    for (const auto& [i, j] : hash_.pairs()) {
        PhysicsObject* a = objects[i].get();
        PhysicsObject* b = objects[j].get();
        if (a->is_container_piston || b->is_container_piston) continue;
        // Bodies on opposite sides of a vessel wall cannot touch mechanically.
        if (a->container_index != b->container_index) continue;
        if (a->group != 0 && a->group == b->group) continue;
        // Molecule-molecule collisions are what the Langevin thermostat
        // models, so gas particles do not also collide as hard spheres.
        if (a->is_gas && b->is_gas) continue;
        const double wa = a->inv_mass(), wb = b->inv_mass();
        if (wa + wb == 0.0) continue;

        Vector delta = a->position - b->position;
        double distance = delta.length();

        if (a->is_fluid && b->is_fluid) {
            // Short-range core of the water model: stops particles from
            // passing through each other.  It is deliberately small (one
            // particle radius); the SPH pressure above does the main work, so
            // the liquid stays fluid and levels instead of locking into a
            // stiff lattice, and the position change feeds the velocity like
            // every PBD constraint, so the pool comes fully to rest.
            const double core = a->liquid_radius * kWaterCoreRadius * 0.5 * (expansion_[i] + expansion_[j]);
            if (distance >= core || distance < 1e-9) continue;
            const Vector correction = delta * ((core - distance) * kWaterCoreStiffness / (distance * (wa + wb)));
            a->position += correction * wa;
            b->position -= correction * wb;
            continue;
        }

        double ra = contact_radius(*a, *b), rb = contact_radius(*b, *a);
        if (distance >= ra + rb) continue;
        Vector normal = distance > 1e-9 ? delta / distance : Vector(0.0, -1.0);
        // Water is pushed out of rigid bodies one-sidedly; the body feels the
        // water through apply_hydrostatics() instead.
        if (b->is_fluid && feels_hydrostatics(*a)) {
            std::swap(a, b);
            std::swap(ra, rb);
            normal = -normal;
        }
        if (a->is_fluid && feels_hydrostatics(*b)) {
            add_contact(a, b, normal, ra + rb - distance, ra, rb, Vector(), 0.0);
            continue;
        }
        add_contact(a, b, normal, ra + rb - distance, ra, rb, Vector());
    }
}

namespace {
// Circle vs. thick segment.  'side_sign' picks which side of the segment the
// body belongs to (from its position at the start of the substep), so fast or
// squeezed bodies cannot tunnel through a wall.
bool segment_penetration(const PhysicsObject& o, const Vector& a, const Vector& b, double half_thickness,
                         double side_sign, Vector& normal, double& depth) {
    const Vector ab = b - a;
    const double len2 = ab.dot(ab);
    if (len2 <= 0.0) return false;
    const double t = clampd((o.position - a).dot(ab) / len2, 0.0, 1.0);
    const Vector closest = a + ab * t;
    const double reach = o.radius + half_thickness;
    if (t > 0.0 && t < 1.0) {
        Vector n = Vector(-ab.y, ab.x) / std::sqrt(len2);
        if (side_sign < 0.0) n = -n;
        const double d = (o.position - a).dot(n);
        if (d >= reach) return false;
        normal = n;
        depth = reach - d;
        return true;
    }
    const Vector d = o.position - closest;
    const double dist = d.length();
    if (dist >= reach || dist <= 0.0) return false;
    normal = d / dist;
    depth = reach - dist;
    return true;
}
} // namespace

void World::solve_container_boundaries() {
    const double th = GasContainer::kWallHalfThickness;
    for (std::size_t i = 0; i < containers.size(); ++i) {
        GasContainer& c = containers[i];
        if (!c.piston) continue;
        PhysicsObject* face = c.piston;
        const double top = c.top();

        for (const auto& ptr : objects) {
            PhysicsObject* o = ptr.get();
            if (o->is_static || o->is_container_piston || o->is_dragged) continue;
            // Broad-phase reject over the whole substep move.
            const Vector& prev = o->previous_position;
            const double pad = o->radius + th + 2.0;
            if (std::max(o->position.x, prev.x) < c.left - pad || std::min(o->position.x, prev.x) > c.right + pad ||
                std::max(o->position.y, prev.y) < c.wall_top - pad || std::min(o->position.y, prev.y) > c.bottom + pad) continue;

            if (o->container_index == int(i)) {
                // Bodies inside the sealed chamber are hard-constrained to it.
                const double pad_in = o->radius + th;
                const double min_x = c.left + pad_in, max_x = c.right - pad_in;
                double min_y = top + pad_in, max_y = c.bottom - pad_in;
                if (min_y > max_y) min_y = max_y = 0.5 * (min_y + max_y);
                if (o->position.x < min_x) add_contact(o, nullptr, Vector(1, 0), min_x - o->position.x, o->radius, 0, Vector());
                else if (o->position.x > max_x) add_contact(o, nullptr, Vector(-1, 0), o->position.x - max_x, o->radius, 0, Vector());
                if (o->position.y > max_y) add_contact(o, nullptr, Vector(0, -1), o->position.y - max_y, o->radius, 0, Vector());
                if (o->position.y < min_y) {
                    // Gas pressure on the piston is applied analytically, so gas
                    // sees the piston as kinematic; liquids and solids push it.
                    const double wb = (o->is_gas || face->is_dragged) ? 0.0 : face->inv_mass();
                    add_contact(o, face, Vector(0, 1), min_y - o->position.y, o->radius, 0.0, Vector(), wb);
                }
                continue;
            }

            // Outside: the outer faces of the walls and the top of the piston.
            Vector n; double depth;
            if (segment_penetration(*o, Vector(c.left, c.wall_top), Vector(c.left, c.bottom), th,
                                    (prev.x < c.left) ? 1.0 : -1.0, n, depth))
                add_contact(o, nullptr, n, depth, o->radius, 0, Vector());
            if (segment_penetration(*o, Vector(c.right, c.wall_top), Vector(c.right, c.bottom), th,
                                    (prev.x < c.right) ? 1.0 : -1.0, n, depth))
                add_contact(o, nullptr, n, depth, o->radius, 0, Vector());
            if (segment_penetration(*o, Vector(c.left, c.bottom), Vector(c.right, c.bottom), th,
                                    (prev.y < c.bottom) ? -1.0 : 1.0, n, depth))
                add_contact(o, nullptr, n, depth, o->radius, 0, Vector());
            // Weights resting on the piston push it down.  Only bodies that
            // started above the piston can be on it (the segment test is
            // one-sided, so anything under the vessel would otherwise count).
            if (prev.y < top &&
                segment_penetration(*o, Vector(c.left, top), Vector(c.right, top), th, -1.0, n, depth))
                add_contact(o, face, n, depth, o->radius, 0.0, Vector(), face->is_dragged ? 0.0 : face->inv_mass());
            // Anything that still tunnelled into the sealed chamber is sent back.
            if (c.in_chamber(o->position)) o->position = prev;
        }
    }
}

void World::solve_screen_walls() {
    for (auto& ptr : objects) {
        PhysicsObject* o = ptr.get();
        if (o->is_static || o->is_dragged || o->is_container_piston) continue;
        const double r = o->radius;
        if (o->position.y + r > height) add_contact(o, nullptr, Vector(0, -1), o->position.y + r - height, r, 0, Vector());
        else if (o->position.y - r < 0.0) add_contact(o, nullptr, Vector(0, 1), r - o->position.y, r, 0, Vector());
        if (o->position.x + r > width) add_contact(o, nullptr, Vector(-1, 0), o->position.x + r - width, r, 0, Vector());
        else if (o->position.x - r < 0.0) add_contact(o, nullptr, Vector(1, 0), r - o->position.x, r, 0, Vector());
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
        if (o->is_container_piston) o->velocity.x = 0.0;
    }
}

void World::solve_velocities(double h) {
    // Bounces slower than this are treated as resting contact (no jitter).
    const double rest_threshold = 2.0 * gravity.length() * h + 1e-3;
    auto velocity_at = [](const PhysicsObject* o, const Vector& r) {
        return o->velocity + r.perp() * o->angular_velocity;
    };

    for (const Contact& c : contacts_) {
        PhysicsObject* a = c.a;
        PhysicsObject* b = c.b;
        const Vector& n = c.normal;
        const Vector ra = n * -c.ra;
        const Vector rb = n * c.rb;
        const double w = c.wa + c.wb;
        if (w == 0.0) continue;

        // ---- Restitution ----
        Vector vb = b ? velocity_at(b, rb) : c.wall_velocity;
        double vn = (velocity_at(a, ra) - vb).dot(n);
        double target;
        if (c.vn_pre < -rest_threshold) target = -c.restitution * c.vn_pre;
        else if (c.vn_pre < 0.0) target = 0.0;
        else target = std::min(vn, c.vn_pre + kPopLimit);   // only limit pop-out speed
        const double jn = (target - vn) / w;
        a->velocity += n * (jn * c.wa);
        if (b) b->velocity -= n * (jn * c.wb);

        // ---- Coulomb friction, bounded by the normal impulse ----
        const double normal_impulse = c.lambda / h + jn;
        if (c.friction <= 0.0 || normal_impulse <= 0.0) continue;
        vb = b ? velocity_at(b, rb) : c.wall_velocity;
        const Vector vrel = velocity_at(a, ra) - vb;
        const Vector vt = vrel - n * vrel.dot(n);
        const double vt_len = vt.length();
        if (vt_len < 1e-9) continue;
        const Vector t = vt / vt_len;
        const double kt = w + c.wia * c.ra * c.ra + c.wib * c.rb * c.rb;
        const double jt = std::min(c.friction * normal_impulse, vt_len / kt);
        const Vector p = t * -jt;
        a->velocity += p * c.wa;
        a->angular_velocity += c.wia * ra.cross(p);
        if (b) {
            b->velocity -= p * c.wb;
            b->angular_velocity -= c.wib * rb.cross(p);
        }
    }
    for (auto& c : containers)
        if (c.piston) c.piston->velocity.x = 0.0;
}

double World::collision_restitution(const PhysicsObject& first, const PhysicsObject& second) {
    if (first.is_fluid || second.is_fluid) return 0.04;
    // Each body's own temperature sets its restitution; the softer,
    // lossier body dominates the impact.
    return std::min(first.effective_restitution(), second.effective_restitution());
}

// ============================================================
// Frame: vessels
// ============================================================

// Classifies objects into vessels, computes each vessel's pressure from the
// ideal-gas law (P = N k T / V) and applies adiabatic heating/cooling when
// the piston moves.  The pressure force on the piston is applied per substep.
void World::update_containers() {
    for (const auto& ptr : objects) {
        ptr->container_index = -1;
        ptr->local_pressure = thermo::kAtmosphere;
    }

    for (std::size_t i = 0; i < containers.size(); ++i) {
        GasContainer& c = containers[i];
        if (!c.piston) continue;

        std::vector<PhysicsObject*> gas_inside;
        double condensed_area = 0.0;
        double sum_tk = 0.0;
        c.liquid_particles = 0;
        for (const auto& ptr : objects) {
            PhysicsObject* o = ptr.get();
            if (o->is_container_piston || o->container_index >= 0 || !c.in_chamber(o->position)) continue;
            o->container_index = int(i);
            if (o->is_gas) { gas_inside.push_back(o); sum_tk += thermo::kelvin(o->temperature); }
            else if (o->is_fluid || o->is_ice) { condensed_area += 2.0 * o->radius * o->radius; ++c.liquid_particles; }
            else condensed_area += kPi * o->radius * o->radius;
        }
        c.gas_particles = int(gas_inside.size());

        const double chamber_area = c.width() * c.chamber_height();
        c.gas_volume = std::max(600.0, chamber_area - condensed_area);

        // Adiabatic process: T·V^(γ-1) = const for the piston's displacement.
        if (c.previous_chamber_height > 0.0 && !gas_inside.empty()) {
            const double prev_volume = std::max(600.0, c.width() * c.previous_chamber_height - condensed_area);
            const double ratio = prev_volume / c.gas_volume;
            if (std::abs(ratio - 1.0) > 1e-9) {
                sum_tk = 0.0;
                for (PhysicsObject* g : gas_inside) {
                    const double tk = thermo::kelvin(g->temperature);
                    const double new_tk = tk * std::pow(ratio, g->heat_capacity_ratio() - 1.0);
                    g->add_heat((new_tk - tk) * g->thermal_mass());
                    sum_tk += thermo::kelvin(g->temperature);
                }
            }
        }
        c.previous_chamber_height = c.chamber_height();

        if (!gas_inside.empty()) {
            const double avg_tk = sum_tk / double(gas_inside.size());
            c.average_temperature = avg_tk + thermo::kAbsoluteZero;
            c.pressure = kGasPressureConstant * double(gas_inside.size()) * avg_tk / c.gas_volume;
        } else {
            c.average_temperature = ambient_temperature;
            c.pressure = 0.0; // evacuated: the atmosphere pushes the piston down
        }
        c.boiling_point = thermo::boiling_point(c.pressure);
        for (const auto& ptr : objects) {
            if (ptr->container_index == int(i)) ptr->local_pressure = std::max(c.pressure, 0.62);
        }
    }
}

// ============================================================
// Frame: heat
// ============================================================

// Heat flow coefficient between two bodies, limited so that a single
// exchange never overshoots equilibrium even with many neighbours.
double World::limited_coupling(double coupling, const PhysicsObject& a, const PhysicsObject& b) {
    const bool a_inf = a.is_heater || a.is_cooler;
    const bool b_inf = b.is_heater || b.is_cooler;
    if (a_inf && b_inf) return 0.0;
    double reduced;
    if (a_inf) reduced = b.thermal_mass();
    else if (b_inf) reduced = a.thermal_mass();
    else reduced = a.thermal_mass() * b.thermal_mass() / (a.thermal_mass() + b.thermal_mass());
    return std::min(coupling, 0.04 * reduced);
}

double World::contact_conductance(const PhysicsObject& a, const PhysicsObject& b) {
    if (a.is_gas && b.is_gas) return 1.0; // molecular mixing
    const double ka = a.conductivity(), kb = b.conductivity();
    return 2.0 * ka * kb / (ka + kb);       // series (harmonic-mean) conduction
}

void World::update_thermal_environment() {
    // The room air is a finite reservoir.  It receives the convective /
    // radiant output of heaters and coolers, sensible heat from every
    // object (gases mix with it directly), and slowly leaks to the
    // building setpoint.  Its temperature is therefore *driven* by the
    // sources and the gases rather than imposed.
    const double rate_scale = heat_transfer_rate / 0.05;
    const double t_air = ambient_temperature;
    double air_energy = (ambient_setpoint - t_air) * kRoomLeak;

    for (const auto& source_ptr : objects) {
        PhysicsObject* source = source_ptr.get();
        if (!source->is_heater && !source->is_cooler) continue;
        // Thermostatic plates: held at their temperature by external power.
        air_energy += (source->temperature - t_air) * kSourceAirConductance;
        for (const auto& target_ptr : objects) {
            PhysicsObject* target = target_ptr.get();
            if (target == source || target->is_heater || target->is_cooler || target->is_container_piston) continue;
            const double distance = (target->position - source->position).length();
            const double reach = source->radius + target->radius + 70.0;
            if (distance >= reach) continue;
            const double proximity = 1.0 - distance / reach;
            const double diff = source->temperature - target->temperature;
            double coupling = kSourceObjectConductance * target->radius * (0.1 + 0.9 * proximity * proximity);
            coupling = std::min(coupling, 0.5 * target->thermal_mass());
            target->add_heat(diff * coupling);
        }
    }

    for (const auto& object_ptr : objects) {
        PhysicsObject* object = object_ptr.get();
        if (object->is_heater || object->is_cooler || object->is_container_piston) continue;
        const double diff = object->temperature - t_air;

        // Newton convection ∝ surface (perimeter in 2D).  Free gas mixes
        // with room air; gas sealed in a vessel only conducts through the wall.
        double factor = 1.0;
        if (object->is_gas) factor = object->container_index >= 0 ? 0.25 : 3.0;
        double coupling = kConvection * object->radius * factor * rate_scale;
        coupling = std::min(coupling, 0.5 * object->thermal_mass());
        double q = diff * coupling;

        // Stefan–Boltzmann radiation (condensed matter only; air is transparent).
        if (!object->is_gas) {
            const double tk = thermo::kelvin(object->temperature), ta = thermo::kelvin(t_air);
            const double q_rad = kRadiation * object->radius * rate_scale * (tk * tk * tk * tk - ta * ta * ta * ta);
            const double cap = std::abs(diff) * 0.5 * object->thermal_mass();
            q += clampd(q_rad, -cap, cap);
        }
        object->add_heat(-q);
        air_energy += q;

        // Evaporative cooling of exposed liquid water: rate ∝ P_sat(T).
        // The latent heat leaves as humidity, not as sensible air heat.
        if (object->is_fluid && object->container_index < 0) {
            const double ratio = thermo::saturation_pressure(object->temperature) / thermo::kAtmosphere;
            object->add_heat(-kEvaporation * object->radius * std::min(ratio, 1.0) * rate_scale);
        }
    }
    ambient_temperature = clampd(t_air + air_energy / kAirHeatCapacity, -200.0, 700.0);
}

void World::conduct_heat() {
    for (const auto& [i, j] : hash_.pairs()) {
        PhysicsObject* a = objects[i].get();
        PhysicsObject* b = objects[j].get();
        if (a->is_container_piston || b->is_container_piston) continue;
        const double distance = (a->position - b->position).length();
        // Contact conduction (also through vessel walls, more slowly).
        const double reach = a->radius + b->radius + 15.0;
        if (distance <= 0.0 || distance >= reach) continue;
        const double contact = 1.0 - distance / reach;
        double coupling = heat_transfer_rate * contact * 0.12 * contact_conductance(*a, *b);
        if (a->container_index != b->container_index) coupling *= 0.2;
        coupling = limited_coupling(coupling, *a, *b);
        const double q = (a->temperature - b->temperature) * coupling;
        a->add_heat(-q);
        b->add_heat(q);
    }
}

// Freezing water bonds to neighbouring ice, so a freezing pool becomes one
// rigid floe (ice expands ~3 % in each direction as it freezes).  Its
// buoyancy comes from update_ice_waterlines().
void World::process_phase_events() {
    bool any_froze = false, any_melted = false;
    for (const auto& ptr : objects) {
        if (ptr->phase_event == PhaseEvent::Froze) any_froze = true;
        else if (ptr->phase_event == PhaseEvent::Melted) any_melted = true;
    }
    if (any_melted) {
        rigid_links.erase(std::remove_if(rigid_links.begin(), rigid_links.end(), [](const RigidLink& l) {
            return l.ice_bond && (!l.obj_a->is_ice || !l.obj_b->is_ice);
        }), rigid_links.end());
    }
    if (any_froze) {
        std::unordered_map<const PhysicsObject*, int> bonds;
        for (const auto& l : rigid_links) {
            if (!l.ice_bond) continue;
            ++bonds[l.obj_a];
            ++bonds[l.obj_b];
        }
        for (const auto& [i, j] : hash_.pairs()) {
            PhysicsObject* a = objects[i].get();
            PhysicsObject* b = objects[j].get();
            if (!a->is_ice || !b->is_ice) continue;
            if (a->phase_event != PhaseEvent::Froze && b->phase_event != PhaseEvent::Froze) continue;
            if (a->container_index != b->container_index) continue;
            const double d = (a->position - b->position).length();
            if (d > kIceBondReach * std::max(a->radius, b->radius)) continue;
            if (bonds[a] >= kMaxIceBonds || bonds[b] >= kMaxIceBonds) continue;
            const double length = std::max(d, 0.62 * (a->radius + b->radius)) * kIceExpansion;
            rigid_links.emplace_back(a, b, length, true);
            ++bonds[a];
            ++bonds[b];
        }
    }
    for (const auto& ptr : objects) ptr->phase_event = PhaseEvent::None;
}

void World::break_ice_bonds() {
    rigid_links.erase(std::remove_if(rigid_links.begin(), rigid_links.end(), [](const RigidLink& l) {
        if (!l.ice_bond) return false;
        if (!l.obj_a->is_ice || !l.obj_b->is_ice) return true;
        return (l.obj_a->position - l.obj_b->position).length() > 1.35 * l.length;   // fracture
    }), rigid_links.end());
}

// Kinetic theory: temperature *is* the particles' random kinetic energy.  An
// Ornstein–Uhlenbeck (Langevin) thermostat relaxes each particle to a Maxwell
// distribution with v_rms ∝ sqrt(T / M), so hot gas moves faster and light
// steam faster than air.  Bulk kinetic energy the thermostat removes (e.g.
// from a blast) is returned to the particle as heat.
void World::thermostat_gases() {
    const double decay = std::exp(-thermo::kGasCollisionRate);
    const double removed_fraction = 1.0 - decay * decay;
    for (auto& ptr : objects) {
        PhysicsObject* o = ptr.get();
        if (!o->is_gas || o->is_static || o->is_dragged) continue;

        const double tk = thermo::kelvin(o->temperature);
        const double v_rms = thermo::kGasSpeedScale * std::sqrt(tk / o->molar_ratio);
        // Motion clearly faster than thermal (beyond 3 v_rms, e.g. from a
        // blast) is bulk kinetic energy; what the thermostat removes of it
        // becomes heat.  Measuring only the excess speed keeps ordinary
        // Maxwell fluctuations from biasing the temperature.
        const double speed = o->velocity.length();
        const double excess = speed - 3.0 * v_rms;
        if (excess > 0.0) {
            const double ke_eq = 0.5 * o->mass * v_rms * v_rms;
            const double dke = removed_fraction * 0.5 * o->mass * excess * excess;
            double dt = thermo::gas_heating_from_kinetic(dke, ke_eq, tk, o->heat_capacity_ratio());
            dt = std::min(dt, std::max(0.0, thermo::kMaxGasTemperature - o->temperature));
            o->add_heat(dt * o->thermal_mass());
        }

        const double sigma = v_rms / std::sqrt(2.0);
        const double kick = sigma * std::sqrt(1.0 - decay * decay);
        o->velocity.x = o->velocity.x * decay + kick * random_normal();
        o->velocity.y = o->velocity.y * decay + kick * random_normal();

        // Archimedes with ideal-gas densities: rho_gas / rho_air =
        // (M_gas / M_air) · (T_air / T_gas).  Hot or light gas rises, cold
        // or heavy gas sinks; gas at room temperature is neutrally buoyant.
        // Inside a sealed vessel there is no surrounding air.
        if (o->container_index < 0) {
            const double density_ratio = o->molar_ratio * thermo::kelvin(ambient_temperature) / thermo::kelvin(o->temperature);
            o->velocity.y += gravity.y * (density_ratio - 1.0);
        }
    }
}

void World::detonate_bomb(PhysicsObject* bomb) {
    constexpr double blast_radius = 250.0;
    constexpr double max_force = 1500.0;
    for (auto& ptr : objects) {
        PhysicsObject* target = ptr.get();
        if (target == bomb || target->is_static || target->is_dragged) continue;
        const Vector delta = target->position - bomb->position;
        const double distance = delta.length();
        if (distance <= 0.0 || distance >= blast_radius) continue;
        const double falloff = 1.0 - distance / blast_radius;
        double kick = max_force * falloff * falloff / target->mass;
        if (target->is_gas) {
            // A blast drives air at a few times its thermal speed (a strong
            // shock), not at the 150 px/frame cap a ball can reach.
            const double v_rms = thermo::kGasSpeedScale
                * std::sqrt(thermo::kelvin(target->temperature) / target->molar_ratio);
            kick = std::min(kick, 4.0 * v_rms * falloff);
        }
        target->velocity += delta * (kick / distance);
        const double speed = target->velocity.length();
        if (speed > kMaxSpeed) target->velocity *= kMaxSpeed / speed;
    }
}

// ============================================================
// Editing
// ============================================================

void World::clear() {
    objects.clear(); springs.clear(); pulleys.clear(); rigid_links.clear(); pistons.clear(); containers.clear();
    contacts_.clear();
    dragged_ = nullptr;
    ambient_temperature = ambient_setpoint;
}

void World::remove_object(PhysicsObject* obj) {
    if (!obj) return;
    if (dragged_ == obj) dragged_ = nullptr;
    contacts_.clear();
    springs.erase(std::remove_if(springs.begin(), springs.end(),
        [obj](const Spring& s) { return s.obj_a == obj || s.obj_b == obj; }), springs.end());
    rigid_links.erase(std::remove_if(rigid_links.begin(), rigid_links.end(),
        [obj](const RigidLink& l) { return l.obj_a == obj || l.obj_b == obj; }), rigid_links.end());
    pulleys.erase(std::remove_if(pulleys.begin(), pulleys.end(),
        [obj](const Pulley& p) { return p.obj_a == obj || p.obj_b == obj; }), pulleys.end());
    pistons.erase(std::remove_if(pistons.begin(), pistons.end(),
        [obj](const PistonLink& p) { return p.obj_a == obj || p.obj_b == obj; }), pistons.end());
    containers.erase(std::remove_if(containers.begin(), containers.end(),
        [obj](const GasContainer& c) { return c.piston == obj; }), containers.end());
    objects.erase(std::remove_if(objects.begin(), objects.end(),
        [obj](const std::unique_ptr<PhysicsObject>& b) { return b.get() == obj; }), objects.end());
}

void World::set_elasticity(double value) {
    elasticity = value;
    for (auto& o : objects)
        if (o->is_bouncy) o->elasticity = value;
}

int World::index_of(const PhysicsObject* obj) const {
    for (std::size_t i = 0; i < objects.size(); ++i)
        if (objects[i].get() == obj) return int(i);
    return -1;
}

PhysicsObject* World::object_at(const Vector& p) const {
    // The whole piston bar of a vessel is grabbable / removable.
    for (const auto& c : containers) {
        if (c.piston && p.x >= c.left && p.x <= c.right && std::abs(p.y - c.top()) < 8.0) return c.piston;
    }
    for (auto it = objects.rbegin(); it != objects.rend(); ++it) {
        const PhysicsObject& o = **it;
        if (o.is_container_piston) continue;
        const Vector d = p - o.position;
        const bool hit = o.shape_type == ShapeType::SQUARE
            ? (std::abs(d.x) <= o.radius && std::abs(d.y) <= o.radius)
            : d.length() <= std::max(o.radius, 3.0);
        if (hit) return it->get();
    }
    return nullptr;
}

const GasContainer* World::container_of_piston(const PhysicsObject* piston) const {
    for (const auto& c : containers)
        if (c.piston == piston) return &c;
    return nullptr;
}

void World::begin_drag(PhysicsObject* obj) {
    if (dragged_) end_drag(Vector());
    if (!obj) return;
    dragged_ = obj;
    obj->is_dragged = true;
    obj->velocity = Vector();
    drag_start_ = drag_target_ = obj->position;
}

void World::drag_to(const Vector& target, bool immediate) {
    if (!dragged_) return;
    drag_target_ = target;
    if (dragged_->is_container_piston) {
        if (const GasContainer* c = container_of_piston(dragged_)) {
            drag_target_.x = 0.5 * (c->left + c->right);
            drag_target_.y = clampd(target.y, c->piston_min_y, c->piston_max_y);
        }
    }
    if (immediate) {
        dragged_->position = dragged_->previous_position = drag_start_ = drag_target_;
        dragged_->velocity = Vector();
    }
}

void World::end_drag(const Vector& throw_velocity) {
    if (!dragged_) return;
    dragged_->is_dragged = false;
    dragged_->velocity = dragged_->is_static ? Vector() : throw_velocity;
    if (dragged_->is_container_piston) dragged_->velocity.x = 0.0;
    dragged_ = nullptr;
}

// ============================================================
// Spawning
// ============================================================

PhysicsObject* World::spawn_at(double x, double y, ShapeType shape, double radius, double mass,
                               bool is_static, bool is_fluid) {
    auto obj = std::make_unique<PhysicsObject>(x, y, radius, elasticity, mass, Rgba{-1, -1, -1, -1},
                                               shape, is_static, is_fluid);
    if (is_fluid) {
        obj->elasticity = 0.1;
        obj->is_bouncy = false;
    }
    obj->temperature = ambient_temperature;
    objects.push_back(std::move(obj));
    return objects.back().get();
}

PhysicsObject* World::spawn_gas_particle(double x, double y) {
    PhysicsObject* node = spawn_at(x, y, ShapeType::CIRCLE, 1.5, 0.5);
    node->is_gas = true;
    node->is_bouncy = false;
    node->elasticity = 1.0;
    node->molar_ratio = 1.0;
    node->color = Rgba{150, 255, 100, 150};
    const double sigma = thermo::kGasSpeedScale * std::sqrt(thermo::kelvin(node->temperature)) / std::sqrt(2.0);
    node->velocity = Vector(sigma * random_normal(), sigma * random_normal());
    return node;
}

PhysicsObject* World::spawn_ball(double x, double y) { return spawn_at(x, y, ShapeType::CIRCLE); }
PhysicsObject* World::spawn_box(double x, double y) { return spawn_at(x, y, ShapeType::SQUARE); }
PhysicsObject* World::spawn_wheel(double x, double y) { return spawn_at(x, y, ShapeType::CIRCLE, 25); }

PhysicsObject* World::spawn_pin(double x, double y) {
    PhysicsObject* obj = spawn_at(x, y, ShapeType::SQUARE, 20, -1, true);
    obj->color = Rgba{100, 100, 120, 255};
    return obj;
}

PhysicsObject* World::spawn_motor(double x, double y) {
    PhysicsObject* obj = spawn_at(x, y, ShapeType::CIRCLE, 20);
    obj->is_motor = true;
    obj->motor_speed = 0.4;     // rad/frame → ~8 px/frame rim speed
    obj->color = Rgba{255, 80, 80, 255};
    return obj;
}

PhysicsObject* World::spawn_heater(double x, double y) {
    PhysicsObject* obj = spawn_at(x, y, ShapeType::SQUARE, 25, -1, true);
    obj->is_heater = true;
    obj->temperature = 300.0;
    obj->color = Rgba{255, 60, 60, 255};
    return obj;
}

PhysicsObject* World::spawn_cooler(double x, double y) {
    PhysicsObject* obj = spawn_at(x, y, ShapeType::SQUARE, 25, -1, true);
    obj->is_cooler = true;
    obj->temperature = -100.0;
    obj->color = Rgba{60, 200, 255, 255};
    return obj;
}

PhysicsObject* World::spawn_bomb(double x, double y) {
    PhysicsObject* obj = spawn_at(x, y, ShapeType::CIRCLE, 15, 20.0);   // iron: sinks in water
    obj->color = Rgba{30, 30, 30, 255};
    obj->fuse = 120;
    return obj;
}

void World::spawn_rope(double x, double y) {
    const int group = next_group_++;
    std::vector<PhysicsObject*> nodes;
    for (int i = 0; i < 12; ++i) {
        PhysicsObject* node = spawn_at(x, y + i * 15, ShapeType::CIRCLE, 6, 1.0);
        node->elasticity = 0.0;
        node->is_bouncy = false;
        node->velocity = Vector();
        node->group = group;
        if (i == 0) { node->is_static = true; node->color = Rgba{200, 200, 200, 255}; }
        nodes.push_back(node);
    }
    for (std::size_t i = 0; i + 1 < nodes.size(); ++i) rigid_links.emplace_back(nodes[i], nodes[i + 1], 15.0);
}

void World::spawn_water(double x, double y) {
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            PhysicsObject* node = spawn_at(x + i * 8, y + j * 8, ShapeType::CIRCLE, 6.0, 1.0, false, true);
            node->is_bouncy = false;
            node->add_heat(0.0); // settle phase if the room is below 0 °C or above boiling
        }
    }
}

void World::spawn_gas_cloud(double x, double y) {
    for (int i = 0; i < 400; ++i) spawn_gas_particle(x + random_double(-15, 15), y + random_double(-15, 15));
}

void World::spawn_container(double x, double y) {
    containers.emplace_back(x, y);
    GasContainer& container = containers.back();
    // The vessel is sealed with a charge of room air at room temperature.
    // Start the piston near the height where that charge supports it.
    constexpr int air_particles = 150;
    const double piston_weight_kpa = 80.0 * gravity.y / (container.width() * kPistonForceScale);
    const double target_pressure = thermo::kAtmosphere + piston_weight_kpa;
    const double eq_height = kGasPressureConstant * air_particles * thermo::kelvin(ambient_temperature)
                           / (target_pressure * container.width());
    const double start_y = clampd(container.bottom - eq_height, container.piston_min_y, container.piston_max_y);

    PhysicsObject* face = spawn_at(x, start_y, ShapeType::SQUARE, 8.0, 80.0);
    face->is_container_piston = true;
    face->is_bouncy = false;
    face->elasticity = 0.05;
    face->color = Rgba{255, 140, 50, 255};
    face->velocity = Vector();
    container.piston = face;

    for (int i = 0; i < air_particles; ++i) {
        spawn_gas_particle(random_double(container.left + 5, container.right - 5),
                           random_double(start_y + 5, container.bottom - 5));
    }
}

void World::spawn_car(double cx, double cy) {
    const int group = next_group_++;
    PhysicsObject* chassis_rear = spawn_at(cx - 35, cy, ShapeType::SQUARE, 20, 6.0);
    PhysicsObject* chassis_front = spawn_at(cx + 35, cy, ShapeType::SQUARE, 20, 6.0);
    chassis_rear->color = chassis_front->color = Rgba{100, 150, 255, 255};

    PhysicsObject* w_rear = spawn_at(cx - 45, cy + 45, ShapeType::CIRCLE, 16, 3.0);
    PhysicsObject* w_front = spawn_at(cx + 45, cy + 45, ShapeType::CIRCLE, 16, 3.0);
    w_rear->is_motor = true;
    w_rear->motor_speed = 0.5;   // rad/frame → 8 px/frame
    w_rear->color = Rgba{255, 50, 50, 255};
    w_front->color = Rgba{50, 50, 50, 255};
    for (PhysicsObject* part : {chassis_rear, chassis_front, w_rear, w_front}) {
        part->velocity = Vector();
        part->group = group;
    }

    rigid_links.emplace_back(chassis_rear, chassis_front, (chassis_rear->position - chassis_front->position).length());
    auto add_strut = [&](PhysicsObject* a, PhysicsObject* b, double stiffness) {
        springs.emplace_back(a, b, (a->position - b->position).length(), stiffness, 0.3);
    };
    add_strut(chassis_rear, w_rear, 2.4);
    add_strut(chassis_front, w_front, 2.4);
    add_strut(chassis_rear, w_front, 1.2);
    add_strut(chassis_front, w_rear, 1.2);
    add_strut(w_rear, w_front, 1.5);
}

bool World::spawn_polygon(const std::vector<Vector>& outline) {
    if (outline.size() < 3) return false;
    Vector centre;
    for (const auto& p : outline) centre += p;
    centre = centre / double(outline.size());

    const Rgba color{random_int(100, 255), random_int(100, 255), random_int(100, 255), 255};
    const int group = next_group_++;
    PhysicsObject* center_node = spawn_at(centre.x, centre.y, ShapeType::CIRCLE, 8);
    center_node->color = color;
    center_node->group = group;
    center_node->velocity = Vector();

    std::vector<PhysicsObject*> nodes;
    for (const auto& p : outline) {
        PhysicsObject* node = spawn_at(p.x, p.y, ShapeType::CIRCLE, 6);
        node->color = color;
        node->group = group;
        node->velocity = Vector();
        nodes.push_back(node);
    }
    const std::size_t n = nodes.size();
    auto link = [&](PhysicsObject* a, PhysicsObject* b) {
        rigid_links.emplace_back(a, b, (a->position - b->position).length());
    };
    for (std::size_t i = 0; i < n; ++i) {
        link(nodes[i], nodes[(i + 1) % n]);
        link(nodes[i], center_node);
        if (n > 3) link(nodes[i], nodes[(i + 2) % n]);
    }
    return true;
}

void World::connect_spring(PhysicsObject* a, PhysicsObject* b) {
    if (!a || !b || a == b) return;
    springs.emplace_back(a, b, std::max((a->position - b->position).length(), 10.0));
}

void World::connect_piston(PhysicsObject* a, PhysicsObject* b) {
    if (!a || !b || a == b) return;
    const double dist = std::max(20.0, (a->position - b->position).length());
    pistons.emplace_back(a, b, dist * 0.55, dist * 1.6, dist, ambient_temperature);
}

void World::connect_pulley(PhysicsObject* a, PhysicsObject* b) {
    if (!a || !b || a == b) return;
    const double anchor_x = 0.5 * (a->position.x + b->position.x);
    const double anchor_y = std::min(a->position.y, b->position.y) - 100.0;
    pulleys.emplace_back(a, b, Vector(anchor_x, std::max(10.0, anchor_y)));
}
