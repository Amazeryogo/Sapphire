// Editing the scene: removing, picking, dragging and spawning.

#include "world.h"

#include "collision.h"
#include "random.h"
#include "world_detail.h"

#include <algorithm>
#include <cmath>

using namespace detail;

namespace {
Rgba material_color(Material m) {
    switch (m) {
    case Material::Wood: {
        const int v = random_int(-18, 18);
        return Rgba{176 + v, 124 + v, 72 + v / 2, 255};
    }
    case Material::Steel: {
        const int v = random_int(-12, 12);
        return Rgba{150 + v, 158 + v, 170 + v, 255};
    }
    case Material::Glass:
        return Rgba{170, 215, 235, 90};
    case Material::Foam: {
        const int v = random_int(-10, 10);
        return Rgba{240 + v, 232 + v, 205 + v, 255};
    }
    case Material::Rubber:
    default:
        break;
    }
    // Rubber: a saturated random colour.
    int c[3] = {random_int(40, 90), random_int(120, 255), random_int(200, 255)};
    std::shuffle(c, c + 3, rng());
    return Rgba{c[0], c[1], c[2], 255};
}

// Spring constants from a natural frequency (rad/frame) and damping ratio,
// so a link behaves the same whatever it connects.
void tune_spring(const PhysicsObject& a, const PhysicsObject& b, double omega, double zeta,
                 double& stiffness, double& damping) {
    const double wa = a.is_static ? 0.0 : 1.0 / a.mass, wb = b.is_static ? 0.0 : 1.0 / b.mass;
    const double reduced = (wa + wb) > 0.0 ? 1.0 / (wa + wb) : 1.0;
    stiffness = reduced * omega * omega;
    damping = 2.0 * zeta * reduced * omega;
}
} // namespace

// ============================================================
// Editing
// ============================================================

void World::clear() {
    objects.clear(); springs.clear(); pulleys.clear(); rigid_links.clear(); pistons.clear();
    axles.clear(); clusters.clear(); containers.clear();
    contacts_.clear();
    patches_.clear();
    slams_.clear();
    fluid_.clear(); fluid_index_.clear(); pair_i_.clear(); pair_j_.clear();
    immersed_pairs_.clear(); immersion_.clear(); ice_submersion_.clear();
    dragged_ = nullptr;
    ambient_temperature = ambient_setpoint;
}

void World::remove_object(PhysicsObject* obj) {
    if (!obj) return;
    std::vector<const PhysicsObject*> doomed{obj};
    if ((obj->is_fixture || obj->is_container_piston) && obj->group != 0) {
        doomed.clear();
        for (const auto& o : objects)
            if (o->group == obj->group && (o->is_fixture || o->is_container_piston)) doomed.push_back(o.get());
    } else if (obj->in_cluster) {
        for (const auto& c : clusters)
            if (std::find(c.nodes.begin(), c.nodes.end(), obj) != c.nodes.end())
                doomed.assign(c.nodes.begin(), c.nodes.end());
    }
    auto gone = [&](const PhysicsObject* o) { return std::find(doomed.begin(), doomed.end(), o) != doomed.end(); };

    if (dragged_ && gone(dragged_)) dragged_ = nullptr;
    contacts_.clear();
    patches_.clear();
    slams_.clear();
    // Per-frame caches hold pointers and indices; the next step rebuilds them.
    fluid_.clear(); fluid_index_.clear(); pair_i_.clear(); pair_j_.clear();
    immersed_pairs_.clear(); immersion_.clear(); ice_submersion_.clear();

    springs.erase(std::remove_if(springs.begin(), springs.end(),
        [&](const Spring& s) { return gone(s.obj_a) || gone(s.obj_b); }), springs.end());
    rigid_links.erase(std::remove_if(rigid_links.begin(), rigid_links.end(),
        [&](const RigidLink& l) { return gone(l.obj_a) || gone(l.obj_b); }), rigid_links.end());
    pulleys.erase(std::remove_if(pulleys.begin(), pulleys.end(),
        [&](const Pulley& p) { return gone(p.obj_a) || gone(p.obj_b); }), pulleys.end());
    pistons.erase(std::remove_if(pistons.begin(), pistons.end(),
        [&](const PistonLink& p) { return gone(p.obj_a) || gone(p.obj_b); }), pistons.end());
    axles.erase(std::remove_if(axles.begin(), axles.end(),
        [&](const Axle& a) { return gone(a.wheel) || gone(a.frame); }), axles.end());
    clusters.erase(std::remove_if(clusters.begin(), clusters.end(), [&](const RigidCluster& c) {
        return std::any_of(c.nodes.begin(), c.nodes.end(), gone);
    }), clusters.end());
    containers.erase(std::remove_if(containers.begin(), containers.end(),
        [&](const GasContainer& c) { return gone(c.piston); }), containers.end());
    objects.erase(std::remove_if(objects.begin(), objects.end(),
        [&](const std::unique_ptr<PhysicsObject>& b) { return gone(b.get()); }), objects.end());
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

PhysicsObject* World::pick(const Vec3& origin, const Vec3& direction, Pick mode, double* distance) const {
    PhysicsObject* best = nullptr;
    PhysicsObject* best_fixture = nullptr;
    double best_t = 1e300, fixture_t = 1e300;
    for (const auto& ptr : objects) {
        PhysicsObject& o = *ptr;
        if (!is_valid(o.position)) continue;
        if (mode != Pick::Any && o.is_gas) continue;
        if (mode == Pick::Grab && (o.is_fluid || o.is_fixture)) continue;
        double t = 0.0;
        const bool hit = o.shape_type == ShapeType::BOX
            ? ray_box(origin, direction, box_of(o), t)
            : ray_sphere(origin, direction, o.position, std::max(o.radius, 3.0), t);
        if (!hit) continue;
        if (o.is_fixture) { if (t < fixture_t) { fixture_t = t; best_fixture = &o; } }
        else if (t < best_t) { best_t = t; best = &o; }
    }
    if (!best && best_fixture) { best = best_fixture; best_t = fixture_t; }
    if (best && distance) *distance = best_t;
    return best;
}

const GasContainer* World::container_of_piston(const PhysicsObject* piston) const {
    for (const auto& c : containers)
        if (c.piston == piston) return &c;
    return nullptr;
}

Vec3 World::clamp_to_room(const Vec3& p, double margin) const {
    return Vec3(clampd(p.x, margin, std::max(margin, width - margin)),
                clampd(p.y, margin, std::max(margin, height - margin)),
                clampd(p.z, margin, std::max(margin, depth - margin)));
}

bool World::begin_drag(PhysicsObject* obj) {
    if (dragged_) end_drag(Vec3());
    if (!obj || obj->is_fixture) return false;      // tanks and vessels stay where they were built
    dragged_ = obj;
    obj->is_dragged = true;
    obj->velocity = Vec3();
    obj->angular_velocity = Vec3();
    drag_start_ = drag_target_ = obj->position;
    return true;
}

void World::drag_to(const Vec3& target, bool immediate) {
    if (!dragged_) return;
    drag_target_ = clamp_to_room(target, 1.0);
    if (dragged_->is_container_piston) {
        if (const GasContainer* c = container_of_piston(dragged_)) {
            drag_target_.x = 0.5 * (c->left + c->right);
            drag_target_.z = 0.5 * (c->back + c->front);
            drag_target_.y = clampd(target.y, c->piston_min_y, c->piston_max_y);
        }
    }
    if (immediate) {
        dragged_->position = dragged_->previous_position = drag_start_ = drag_target_;
        dragged_->velocity = Vec3();
    }
}

void World::end_drag(const Vec3& throw_velocity) {
    if (!dragged_) return;
    dragged_->is_dragged = false;
    dragged_->velocity = dragged_->is_static ? Vec3() : throw_velocity;
    if (dragged_->is_container_piston) dragged_->velocity.x = dragged_->velocity.z = 0.0;
    dragged_ = nullptr;
}

// ============================================================
// Spawning
// ============================================================

PhysicsObject* World::add_sphere(const Vec3& p, double radius, Material material) {
    auto obj = std::make_unique<PhysicsObject>(p, radius);
    obj->make_sphere(radius);
    obj->elasticity = elasticity;
    obj->set_material(material);
    obj->color = material_color(material);
    obj->temperature = ambient_temperature;
    objects.push_back(std::move(obj));
    return objects.back().get();
}

PhysicsObject* World::add_box(const Vec3& p, const Vec3& half_extents, Material material) {
    auto obj = std::make_unique<PhysicsObject>(p);
    obj->make_box(half_extents);
    obj->elasticity = elasticity;
    obj->set_material(material);
    obj->color = material_color(material);
    obj->temperature = ambient_temperature;
    objects.push_back(std::move(obj));
    return objects.back().get();
}

PhysicsObject* World::add_particle(const Vec3& p, double radius, double mass) {
    auto obj = std::make_unique<PhysicsObject>(p, radius);
    obj->make_sphere(radius);
    obj->is_bouncy = false;
    obj->set_mass(mass);
    obj->temperature = ambient_temperature;
    objects.push_back(std::move(obj));
    return objects.back().get();
}

PhysicsObject* World::spawn_water_particle(const Vec3& p) {
    PhysicsObject* node = add_particle(p, 5.5, kWaterParticleMass);
    node->is_fluid = true;
    node->elasticity = 0.1;
    node->color = Rgba{45, 135, 245, 255};
    node->add_heat(0.0);   // settle the phase if the room is below 0 °C or above boiling
    return node;
}

PhysicsObject* World::spawn_gas_particle(const Vec3& p) {
    PhysicsObject* node = add_particle(p, 1.5, 0.5);
    node->is_gas = true;
    node->elasticity = 1.0;
    node->molar_ratio = 1.0;
    node->color = Rgba{150, 255, 100, 150};
    const double sigma = thermo::kGasSpeedScale * std::sqrt(thermo::kelvin(node->temperature)) / std::sqrt(3.0);
    node->velocity = Vec3(random_normal(), random_normal(), random_normal()) * sigma;
    return node;
}

PhysicsObject* World::spawn_ball(const Vec3& p) {
    PhysicsObject* obj = add_sphere(p, 18.0, spawn_material);
    obj->velocity = Vec3(random_double(-2.0, 2.0), random_double(-1.0, 2.0), random_double(-1.5, 1.5));
    return obj;
}

PhysicsObject* World::spawn_box(const Vec3& p) {
    PhysicsObject* obj = add_box(p, Vec3(18.0, 18.0, 18.0), spawn_material);
    obj->velocity = Vec3(random_double(-1.0, 1.0), 0.0, random_double(-1.0, 1.0));
    obj->angular_velocity = Vec3(random_double(-0.03, 0.03), random_double(-0.03, 0.03), random_double(-0.03, 0.03));
    return obj;
}

PhysicsObject* World::spawn_wheel(const Vec3& p) { return add_sphere(p, 25.0, spawn_material); }

PhysicsObject* World::spawn_pin(const Vec3& p) {
    PhysicsObject* obj = add_box(p, Vec3(12.0, 12.0, 12.0), Material::Steel);
    obj->is_static = true;
    obj->color = Rgba{100, 100, 120, 255};
    return obj;
}

PhysicsObject* World::spawn_platform(const Vec3& p, double tilt_radians) {
    PhysicsObject* obj = add_box(p, Vec3(110.0, 6.0, 70.0), Material::Wood);
    obj->is_static = true;
    obj->orientation = obj->previous_orientation = Quat::from_axis_angle(Vec3(0, 0, 1), tilt_radians);
    obj->color = Rgba{150, 110, 75, 255};
    return obj;
}

PhysicsObject* World::spawn_motor(const Vec3& p) {
    PhysicsObject* obj = add_sphere(p, 20.0, Material::Rubber);
    obj->is_motor = true;
    obj->motor_speed = 0.4;     // rad/frame → ~8 cm/frame rim speed
    obj->color = Rgba{255, 80, 80, 255};
    return obj;
}

PhysicsObject* World::spawn_heater(const Vec3& p) {
    PhysicsObject* obj = add_box(p, Vec3(32.0, 7.0, 32.0), Material::Steel);
    obj->is_static = true;
    obj->is_heater = true;
    obj->temperature = 300.0;
    obj->color = Rgba{255, 60, 60, 255};
    return obj;
}

PhysicsObject* World::spawn_cooler(const Vec3& p) {
    PhysicsObject* obj = add_box(p, Vec3(32.0, 7.0, 32.0), Material::Steel);
    obj->is_static = true;
    obj->is_cooler = true;
    obj->temperature = -100.0;
    obj->color = Rgba{60, 200, 255, 255};
    return obj;
}

PhysicsObject* World::spawn_bomb(const Vec3& p) {
    PhysicsObject* obj = add_sphere(p, 15.0, Material::Steel);   // iron: sinks in water
    obj->color = Rgba{30, 30, 30, 255};
    obj->fuse = 120;
    return obj;
}

void World::spawn_rope(const Vec3& p) {
    const int group = next_group_++;
    std::vector<PhysicsObject*> nodes;
    for (int i = 0; i < 12; ++i) {
        PhysicsObject* node = add_particle(p - Vec3(0.0, i * 15.0, 0.0), 6.0, 1.0);
        node->elasticity = 0.0;
        node->material = Material::Wood;
        node->color = Rgba{196, 164, 110, 255};
        node->group = group;
        if (i == 0) { node->is_static = true; node->color = Rgba{200, 200, 200, 255}; }
        nodes.push_back(node);
    }
    for (std::size_t i = 0; i + 1 < nodes.size(); ++i) rigid_links.emplace_back(nodes[i], nodes[i + 1], 15.0);
}

void World::spawn_water(const Vec3& p, int nx, int ny, int nz) {
    const double s = kFluidSpacing;
    const Vec3 origin = p - Vec3((nx - 1) * s, (ny - 1) * s, (nz - 1) * s) * 0.5;
    for (int i = 0; i < nx; ++i)
        for (int j = 0; j < ny; ++j)
            for (int k = 0; k < nz; ++k)
                spawn_water_particle(clamp_to_room(origin + Vec3(i * s, j * s, k * s), kWaterContactRadius));
}

void World::spawn_gas_cloud(const Vec3& p) {
    for (int i = 0; i < 400; ++i)
        spawn_gas_particle(clamp_to_room(p + Vec3(random_double(-15, 15), random_double(-15, 15), random_double(-15, 15)), 2.0));
}

// Five static glass slabs around a cavity: a floor and four walls.
void World::add_walls(const Vec3& lo, const Vec3& hi, int group) {
    const double th = GasContainer::kWallHalfThickness;
    const Vec3 mid = (lo + hi) * 0.5, half = (hi - lo) * 0.5;
    auto slab = [&](const Vec3& centre, const Vec3& half_extents) {
        PhysicsObject* wall = add_box(centre, half_extents, Material::Glass);
        wall->is_static = true;
        wall->is_fixture = true;
        wall->group = group;
    };
    slab(Vec3(mid.x, lo.y - th, mid.z), Vec3(half.x + 2.0 * th, th, half.z + 2.0 * th));
    slab(Vec3(lo.x - th, mid.y, mid.z), Vec3(th, half.y, half.z + 2.0 * th));
    slab(Vec3(hi.x + th, mid.y, mid.z), Vec3(th, half.y, half.z + 2.0 * th));
    slab(Vec3(mid.x, mid.y, lo.z - th), Vec3(half.x, half.y, th));
    slab(Vec3(mid.x, mid.y, hi.z + th), Vec3(half.x, half.y, th));
}

void World::spawn_container(const Vec3& p) {
    constexpr double hw = 100.0, hh = 90.0, hd = 70.0;
    const double th = GasContainer::kWallHalfThickness;
    const Vec3 centre(clampd(p.x, hw + 2.0 * th + 1.0, std::max(hw + 2.0 * th + 1.0, width - hw - 2.0 * th - 1.0)),
                      clampd(p.y, hh + 2.0 * th + 1.0, std::max(hh + 2.0 * th + 1.0, height - hh - 16.0)),
                      clampd(p.z, hd + 2.0 * th + 1.0, std::max(hd + 2.0 * th + 1.0, depth - hd - 2.0 * th - 1.0)));
    const int group = next_group_++;
    GasContainer container(centre, hw, hh, hd);
    container.group = group;
    add_walls(Vec3(container.left, container.bottom, container.back),
              Vec3(container.right, container.wall_top, container.front), group);

    // The vessel is sealed with a charge of room air at room temperature.
    // Start the piston near the height where that charge supports it.
    const double piston_weight_kpa = kPistonMass * gravity.length() / (container.bore_area() * kPistonForceScale);
    const double target_pressure = thermo::kAtmosphere + piston_weight_kpa;
    const double eq_height = kGasPressureConstant * kVesselAirParticles * thermo::kelvin(ambient_temperature)
                           / (target_pressure * container.bore_area());
    const double start_y = clampd(container.bottom + eq_height + GasContainer::kPistonHalfThickness,
                                  container.piston_min_y, container.piston_max_y);

    PhysicsObject* face = add_box(Vec3(centre.x, start_y, centre.z),
                                  Vec3(hw - 0.5, GasContainer::kPistonHalfThickness, hd - 0.5), Material::Steel);
    face->set_mass(kPistonMass);
    face->is_container_piston = true;
    face->elasticity = 0.05;
    face->color = Rgba{255, 140, 50, 255};
    face->group = group;
    container.piston = face;

    const double top = container.top();
    for (int i = 0; i < kVesselAirParticles; ++i) {
        spawn_gas_particle(Vec3(random_double(container.left + 5, container.right - 5),
                                random_double(container.bottom + 5, std::max(container.bottom + 6, top - 5)),
                                random_double(container.back + 5, container.front - 5)));
    }
    containers.push_back(container);
}

void World::spawn_tank(const Vec3& p, double inner_width, double inner_height, double inner_depth) {
    const double th = GasContainer::kWallHalfThickness;
    const double hw = 0.5 * inner_width + 2.0 * th + 1.0, hd = 0.5 * inner_depth + 2.0 * th + 1.0;
    const double x = clampd(p.x, hw, std::max(hw, width - hw));
    const double z = clampd(p.z, hd, std::max(hd, depth - hd));
    const double floor_y = 2.0 * th;
    add_walls(Vec3(x - 0.5 * inner_width, floor_y, z - 0.5 * inner_depth),
              Vec3(x + 0.5 * inner_width, floor_y + inner_height, z + 0.5 * inner_depth), next_group_++);
}

// A rigid chassis on four ball wheels.  Each wheel hangs on a tripod of
// stiff, damped struts (the suspension) and spins on an axle fixed in the
// chassis; the rear pair is motorised.
void World::spawn_car(const Vec3& p) {
    const int group = next_group_++;
    const Vec3 centre = clamp_to_room(p, 60.0);
    PhysicsObject* chassis = add_box(centre, Vec3(55.0, 11.0, 26.0), Material::Wood);
    chassis->set_mass(0.4 * PhysicsObject::kWaterDensity * chassis->volume());
    chassis->color = Rgba{90, 140, 250, 255};
    chassis->group = group;

    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sz = -1; sz <= 1; sz += 2) {
            const Vec3 local(sx * 38.0, -20.0, sz * 44.0);
            PhysicsObject* wheel = add_sphere(centre + local, 16.0, Material::Rubber);
            wheel->group = group;
            wheel->color = Rgba{45, 45, 50, 255};
            if (sx < 0) {
                wheel->is_motor = true;
                wheel->motor_speed = 0.5;   // rad/frame → 8 cm/frame
                wheel->color = Rgba{235, 60, 60, 255};
            }
            axles.emplace_back(wheel, chassis, Vec3(0.0, 0.0, -1.0));
            const Vec3 anchors[3] = {Vec3(local.x - 25.0, 11.0, sz * 26.0), Vec3(local.x + 25.0, 11.0, sz * 26.0),
                                     Vec3(local.x, 11.0, 0.0)};
            double k, d;
            tune_spring(*wheel, *chassis, 0.9, 0.5, k, d);
            for (const Vec3& anchor : anchors)
                springs.emplace_back(wheel, chassis, (local - anchor).length(), k, d, Vec3(), anchor);
        }
    }
}

// The outline becomes a hoop of beads held together as one rigid body.
bool World::spawn_polygon(const std::vector<Vec3>& outline) {
    if (outline.size() < 3) return false;
    constexpr double kBead = 7.0, kStep = 15.0;
    std::vector<Vec3> points;
    for (std::size_t i = 0; i < outline.size(); ++i) {
        const Vec3& a = outline[i];
        const Vec3& b = outline[(i + 1) % outline.size()];
        const int segments = std::max(1, int(std::ceil((b - a).length() / kStep)));
        for (int k = 0; k < segments; ++k) points.push_back(a + (b - a) * (double(k) / segments));
    }
    if (points.size() < 3 || points.size() > 400) return false;

    const Material material = spawn_material;
    Rgba color = material_color(material);
    const int group = next_group_++;
    std::vector<PhysicsObject*> nodes;
    for (const auto& p : points) {
        PhysicsObject* node = add_sphere(clamp_to_room(p, kBead), kBead, material);
        node->color = color;
        node->group = group;
        node->in_cluster = true;
        nodes.push_back(node);
    }
    clusters.emplace_back(nodes);
    return true;
}

void World::connect_spring(PhysicsObject* a, PhysicsObject* b) {
    if (!a || !b || a == b) return;
    double k, d;
    tune_spring(*a, *b, 0.4, 0.08, k, d);
    springs.emplace_back(a, b, std::max((a->position - b->position).length(), 10.0), k, d);
}

void World::connect_piston(PhysicsObject* a, PhysicsObject* b) {
    if (!a || !b || a == b) return;
    const double dist = std::max(20.0, (a->position - b->position).length());
    pistons.emplace_back(a, b, dist * 0.55, dist * 1.6, dist, ambient_temperature);
}

void World::connect_pulley(PhysicsObject* a, PhysicsObject* b) {
    if (!a || !b || a == b) return;
    const Vec3 mid = (a->position + b->position) * 0.5;
    const double anchor_y = std::max(a->position.y, b->position.y) + 100.0;
    pulleys.emplace_back(a, b, Vec3(mid.x, std::min(height - 10.0, anchor_y), mid.z));
}
