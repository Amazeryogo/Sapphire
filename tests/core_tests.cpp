// Tests for the Qt-free physics core.  Run: ./sapphire_tests [name-filter]
// (exit code 0 = pass).  Many of them compare the simulation with a textbook
// result: bounce heights, the 5/7 rolling rule, the pendulum period,
// Archimedes' principle, the gas laws.

#include "collision.h"
#include "random.h"
#include "spatial_hash.h"
#include "thermo.h"
#include "world.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <functional>
#include <set>
#include <string>
#include <vector>

namespace {
int g_failures = 0;
int g_checks = 0;
constexpr double kPi = 3.14159265358979323846;

void check(bool ok, const std::string& what, double value = 0.0) {
    ++g_checks;
    if (!ok) {
        ++g_failures;
        std::printf("  FAIL: %s (value %.4f)\n", what.c_str(), value);
    }
}
void near(double value, double expected, double tolerance, const std::string& what) {
    check(std::abs(value - expected) <= tolerance,
          what + " expected " + std::to_string(expected) + " ± " + std::to_string(tolerance), value);
}

void run(int frames, World& w) { for (int i = 0; i < frames; ++i) w.step(); }

// A quiet world: no air, so results can be compared with vacuum formulas.
World vacuum() {
    World w;
    w.air_density_scale = 0.0;
    return w;
}

PhysicsObject* still(PhysicsObject* o) {
    o->velocity = o->angular_velocity = Vec3();
    return o;
}

double water_surface(const World& w) {
    std::vector<double> ys;
    for (const auto& o : w.objects)
        if (o->is_fluid) ys.push_back(o->position.y);
    if (ys.empty()) return 0.0;
    std::sort(ys.begin(), ys.end());
    return ys[ys.size() * 97 / 100] + World::kWaterContactRadius;      // ignore stray droplets
}

double water_kinetic_energy(const World& w) {
    double ke = 0.0;
    int n = 0;
    for (const auto& o : w.objects)
        if (o->is_fluid) { ke += 0.5 * o->mass * o->velocity.length_sq(); ++n; }
    return n ? ke / n : 0.0;
}

// A tank in the middle of a small room, filled with 'blocks' drops of water and left to settle.
World make_pool(int blocks = 3, int settle_frames = 420) {
    World w;
    w.width = 400; w.height = 400; w.depth = 300;
    w.spawn_tank(Vec3(200, 0, 150));
    for (int k = 0; k < blocks; ++k) {
        w.spawn_water(Vec3(200, 60 + 85.0 * k, 150));
        run(40, w);
    }
    run(settle_frames, w);
    return w;
}

// ------------------------------------------------------------------

void test_thermo() {
    near(thermo::boiling_point(thermo::kAtmosphere), 100.0, 0.05, "boiling point at 1 atm");
    near(thermo::boiling_point(50.0), 81.6, 1.5, "boiling point at 50 kPa");
    near(thermo::saturation_pressure(100.0), thermo::kAtmosphere, 0.05, "P_sat(100 °C)");
    check(thermo::water_density(4.0) > thermo::water_density(0.0) &&
          thermo::water_density(4.0) > thermo::water_density(10.0), "water densest near 4 °C");
    near(thermo::water_viscosity_ratio(20.0), 1.0, 1e-9, "viscosity ratio at 20 °C");
    check(thermo::water_viscosity_ratio(80.0) < 0.5, "hot water is runnier", thermo::water_viscosity_ratio(80.0));
    near(thermo::water_surface_tension_ratio(20.0), 1.0, 1e-9, "surface tension ratio at 20 °C");
    near(thermo::elastomer_restitution_factor(20.0), 1.0, 1e-9, "rubber normalised at 20 °C");
    check(thermo::elastomer_restitution_factor(-45.0) < 0.4, "rubber dead at glass transition",
          thermo::elastomer_restitution_factor(-45.0));
}

void test_phase_changes() {
    World w;
    PhysicsObject& water = *w.spawn_water_particle(Vec3(100, 100, 100));
    water.temperature = 20.0;
    water.add_heat(-20.0 * water.thermal_mass());        // cool to 0 °C
    near(water.temperature, 0.0, 1e-9, "water cools to melting point");
    check(water.is_fluid, "still liquid at 0 °C before latent heat leaves");
    water.add_heat(-thermo::kLatentFusion * water.mass - 1e-6);
    check(water.is_ice && water.phase_event == PhaseEvent::Froze, "freezes after latent heat of fusion");
    check(!water.is_static, "ice is a free body, not frozen in place");
}

void test_materials() {
    World w;
    PhysicsObject* steel = w.add_sphere(Vec3(100, 100, 100), 10.0, Material::Steel);
    PhysicsObject* wood = w.add_box(Vec3(200, 100, 100), Vec3(10, 5, 20), Material::Wood);
    near(steel->mass, 7.85 * 0.001 * 4.0 / 3.0 * kPi * 1000.0, 1e-6, "steel ball mass = density × volume (32.9 kg)");
    near(wood->mass, 0.65 * 0.001 * 8.0 * 10 * 5 * 20, 1e-9, "wood box mass = density × volume");
    near(1.0 / steel->inv_inertia_body.x, 0.4 * steel->mass * 100.0, 1e-6, "solid sphere inertia 2/5 m r²");
    near(1.0 / wood->inv_inertia_body.y, wood->mass / 3.0 * (100.0 + 400.0), 1e-6, "box inertia m/3 (a² + c²)");
    check(!steel->is_bouncy && w.add_sphere(Vec3(), 5, Material::Rubber)->is_bouncy, "only rubber follows the bounce slider");
}

void test_broad_phase_finds_every_pair() {
    seed_rng(7);
    std::vector<std::unique_ptr<PhysicsObject>> objects;
    for (int i = 0; i < 700; ++i) {
        const double r = (i % 7 == 0) ? random_double(15.0, 40.0) : random_double(1.0, 8.0);
        auto o = std::make_unique<PhysicsObject>(
            Vec3(random_double(-50, 450), random_double(-50, 350), random_double(-50, 250)), r);
        if (i % 5 == 1) o->is_gas = true;
        if (i % 5 == 2) o->is_fluid = true;
        if (i % 11 == 0) {
            o->make_box(Vec3(random_double(5, 60), random_double(2, 10), random_double(5, 40)));
            o->orientation = Quat::from_axis_angle(Vec3(1, 2, 3), random_double(0, 3));
        }
        objects.push_back(std::move(o));
    }
    const double margin = 4.0;
    SpatialHash hash;
    hash.build(objects, margin, 20.0, Vec3(400.0, 300.0, 200.0));
    std::set<std::pair<std::uint32_t, std::uint32_t>> found(hash.pairs().begin(), hash.pairs().end());
    check(found.size() == hash.pairs().size(), "no duplicate pairs");
    int missing = 0, forbidden = 0;
    auto extent = [](const PhysicsObject& o, int axis) {
        if (o.shape_type != ShapeType::BOX) return o.radius;
        return o.support(Vec3(axis == 0, axis == 1, axis == 2));
    };
    for (std::uint32_t i = 0; i < objects.size(); ++i)
        for (std::uint32_t j = i + 1; j < objects.size(); ++j) {
            const PhysicsObject& a = *objects[i];
            const PhysicsObject& b = *objects[j];
            const bool same_particles = (a.is_gas && b.is_gas) || (a.is_fluid && b.is_fluid);
            if (same_particles) { forbidden += int(found.count({i, j})); continue; }
            bool overlap = true;
            for (int axis = 0; axis < 3; ++axis)
                if (std::abs(a.position[axis] - b.position[axis]) > extent(a, axis) + extent(b, axis) + margin) overlap = false;
            if (overlap && !found.count({i, j})) ++missing;
        }
    check(missing == 0, "broad phase misses no overlapping pair", missing);
    check(forbidden == 0, "gas–gas and liquid–liquid pairs are left out", forbidden);
}

void test_box_geometry() {
    BoxShape a{Vec3(0, 10, 0), Vec3(10, 10, 10), {Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)}};
    BoxShape b{Vec3(3, -9, 2), Vec3(20, 10, 20), {Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)}};
    Vec3 n;
    ManifoldPoint pts[8];
    int count = collide_box_box(a, b, n, pts);
    check(count == 4, "box resting on a box: four-point manifold", count);
    near(n.y, 1.0, 1e-9, "normal points from the lower box to the upper one");
    for (int k = 0; k < count; ++k) near((pts[k].on_b - pts[k].on_a).dot(n), 1.0, 1e-9, "penetration depth of 1");

    b.center = Vec3(0, -10.5, 0);
    check(collide_box_box(a, b, n, pts) == 0, "separated boxes do not collide");

    // Two bars crossed at right angles, touching edge to edge.
    PhysicsObject p(Vec3(0, 0, 0)), q(Vec3(0, 13.5, 0));
    p.make_box(Vec3(30, 5, 5));
    q.make_box(Vec3(5, 5, 30));
    p.orientation = Quat::from_axis_angle(Vec3(1, 0, 0), kPi / 4);
    q.orientation = Quat::from_axis_angle(Vec3(0, 0, 1), kPi / 4);
    count = collide_box_box(box_of(q), box_of(p), n, pts);
    check(count == 1, "crossed edges give a single contact point", count);
    near(n.y, 1.0, 1e-6, "edge–edge normal is the common perpendicular");
    near((pts[0].on_b - pts[0].on_a).dot(n), 2.0 * 5.0 * std::sqrt(2.0) - 13.5, 1e-6, "edge–edge depth");

    Vec3 on_box; double depth = 0.0;
    check(collide_sphere_box(Vec3(0, 24, 0), 5.0, a, n, on_box, depth), "sphere touches the top of a box");
    near(depth, 1.0, 1e-9, "sphere–box depth");
    double t = 0.0;
    check(ray_box(Vec3(-50, 10, 0), Vec3(1, 0, 0), a, t), "ray hits a box");
    near(t, 40.0, 1e-9, "ray–box distance");
    check(ray_sphere(Vec3(0, 0, -50), Vec3(0, 0, 1), Vec3(0, 0, 0), 10.0, t) && std::abs(t - 40.0) < 1e-9, "ray–sphere distance");
}

void test_free_fall_and_bounce() {
    seed_rng(1);
    World w = vacuum();
    // Projectile: x = v t, y = y0 + v t - g t²/2.
    PhysicsObject* shot = still(w.add_sphere(Vec3(100, 100, 250), 10.0, Material::Steel));
    shot->velocity = Vec3(4.0, 6.0, 0.0);
    run(30, w);
    const double g = -w.gravity.y, t = 30.0;
    near(shot->position.x, 100.0 + 4.0 * t, 1e-6, "projectile keeps its horizontal speed");
    // The integrator is symplectic Euler with 6 substeps: its only error is g·t·h/2.
    near(shot->position.y, 100.0 + 6.0 * t - 0.5 * g * t * t, 0.5 * g * t / World::kSubsteps + 1e-6,
         "projectile follows the parabola");

    // Bounce: a steel ball (e = 0.6) dropped from 200 cm rebounds to e² · 200.
    World b = vacuum();
    PhysicsObject* ball = still(b.add_sphere(Vec3(300, 210, 250), 10.0, Material::Steel));
    double lowest = 1e9, rebound = 0.0;
    bool bounced = false;
    for (int f = 0; f < 400; ++f) {
        b.step();
        const double bottom = ball->position.y - ball->radius;
        lowest = std::min(lowest, bottom);
        if (!bounced && ball->velocity.y > 0.0) bounced = true;
        if (bounced && ball->velocity.y > 0.0) rebound = std::max(rebound, bottom);
        if (bounced && ball->velocity.y < 0.0 && rebound > 0.0) break;
    }
    check(lowest > -0.5, "ball does not sink into the floor", lowest);
    near(rebound, 0.36 * 200.0, 4.0, "rebound height is e² times the drop height");
}

void test_ball_comes_to_rest() {
    seed_rng(1);
    World w;
    PhysicsObject* ball = still(w.spawn_ball(Vec3(450, 300, 250)));
    run(900, w);
    near(ball->position.y, ball->radius, 0.2, "ball rests on the floor");
    near(ball->velocity.y, 0.0, 0.01, "resting ball does not jitter");
    near(ball->velocity.length(), 0.0, 0.02, "resting ball does not creep");
}

void test_sliding_ball_starts_rolling() {
    seed_rng(1);
    World w = vacuum();
    PhysicsObject* ball = still(w.add_sphere(Vec3(100, 18, 250), 18.0, Material::Rubber));
    ball->velocity = Vec3(10, 0, 0);
    run(16, w);           // sliding ends after (2/7) v / (μ g) ≈ 12 frames
    // Rolling without slipping towards +x means spinning about -z.
    near(-ball->angular_velocity.z * ball->radius, ball->velocity.x, 0.05, "friction makes the ball roll (v = ωR)");
    near(ball->velocity.x, 10.0 * 5.0 / 7.0, 0.25, "a sliding solid sphere ends up rolling at 5/7 of its speed");
    near(ball->velocity.z, 0.0, 1e-6, "and does not veer sideways");
}

void test_box_friction_on_a_ramp() {
    seed_rng(1);
    // Wood on wood, μ = 0.45: holds at 15° (tan = 0.27), slides at 35° (tan = 0.70).
    for (const double degrees : {15.0, 35.0}) {
        World w = vacuum();
        const double tilt = degrees * kPi / 180.0;
        PhysicsObject* ramp = w.spawn_platform(Vec3(450, 200, 250), tilt);
        const Vec3 up = ramp->orientation.rotate(Vec3(0, 1, 0));
        PhysicsObject* box = still(w.add_box(ramp->position + up * (6.0 + 12.0 + 0.05), Vec3(20, 12, 20), Material::Wood));
        box->orientation = box->previous_orientation = ramp->orientation;
        const Vec3 start = box->position;
        run(120, w);
        const double moved = (box->position - start).length();
        if (degrees < 20.0) check(moved < 0.5, "box holds on a 15° ramp (static friction)", moved);
        else {
            const double g = w.gravity.length();
            const double expected = 0.5 * g * (std::sin(tilt) - 0.45 * std::cos(tilt)) * 120.0 * 120.0;
            check(moved > 40.0, "box slides down a 35° ramp", moved);
            if (moved < 2.0 * ramp->half_extents.x - 40.0)        // still on the ramp
                near(moved, expected, 0.15 * expected, "sliding distance = ½ g (sin θ − μ cos θ) t²");
        }
    }
}

void test_boxes_stack_and_tumble() {
    seed_rng(1);
    World w;
    std::vector<PhysicsObject*> stack;
    for (int k = 0; k < 4; ++k)
        stack.push_back(still(w.add_box(Vec3(300, 18.0 + 36.2 * k, 250), Vec3(18, 18, 18), Material::Wood)));
    run(600, w);
    for (int k = 0; k < 4; ++k) {
        near(stack[std::size_t(k)]->position.y, 18.0 + 36.0 * k, 0.6, "stacked box " + std::to_string(k) + " stays at its height");
        near(stack[std::size_t(k)]->position.x, 300.0, 1.0, "stack does not drift");
    }
    near(stack[3]->angular_velocity.length(), 0.0, 0.002, "stack is at rest");

    // A box dropped on its corner ends up lying on a face.
    PhysicsObject* box = still(w.add_box(Vec3(600, 150, 250), Vec3(30, 12, 20), Material::Wood));
    box->orientation = box->previous_orientation = Quat::from_axis_angle(Vec3(1, 0.3, 0.7), 0.9);
    run(900, w);
    const double h = box->position.y;
    check(std::abs(h - 12.0) < 0.5 || std::abs(h - 20.0) < 0.5 || std::abs(h - 30.0) < 0.5,
          "tumbled box rests on one of its faces", h);
    near(box->velocity.length() + box->angular_velocity.length() * 30.0, 0.0, 0.05, "tumbled box comes to rest");
}

void test_rope_and_pendulum() {
    seed_rng(1);
    World w;
    w.spawn_rope(Vec3(450, 400, 250));
    run(600, w);
    double worst = 0.0;
    for (const auto& l : w.rigid_links)
        worst = std::max(worst, (l.obj_a->position - l.obj_b->position).length() / l.length);
    check(worst < 1.01, "hanging rope links stay within 1 % of length", worst);
    check(std::abs(w.objects.back()->velocity.y) < 0.05, "hanging rope has no hidden velocity",
          w.objects.back()->velocity.y);

    // Simple pendulum: T = 2π √(L / g) for small swings.
    World p = vacuum();
    const double length = 150.0;
    PhysicsObject* pivot = p.spawn_pin(Vec3(450, 400, 250));
    const double angle = 0.12;
    PhysicsObject* bob = still(p.add_sphere(pivot->position + Vec3(std::sin(angle), -std::cos(angle), 0) * length, 8.0, Material::Steel));
    bob->group = pivot->group = 99;
    p.rigid_links.emplace_back(pivot, bob, length);
    const double energy0 = p.energy().kinetic + p.energy().potential;
    std::vector<int> crossings;
    double last_x = bob->position.x - pivot->position.x;
    for (int f = 1; f <= 1200; ++f) {
        p.step();
        const double x = bob->position.x - pivot->position.x;
        if (last_x > 0.0 && x <= 0.0) crossings.push_back(f);
        last_x = x;
    }
    check(crossings.size() >= 4, "pendulum keeps swinging", double(crossings.size()));
    if (crossings.size() >= 4) {
        const double period = double(crossings.back() - crossings.front()) / double(crossings.size() - 1);
        const double expected = 2.0 * kPi * std::sqrt(length / p.gravity.length());
        near(period, expected, 0.02 * expected, "pendulum period 2π√(L/g)");
    }
    const double energy1 = p.energy().kinetic + p.energy().potential;
    const double swing = bob->mass * p.gravity.length() * length * (1.0 - std::cos(angle));
    // Position-based dynamics damps a little (first order in the substep).
    check(energy1 <= energy0 + 1e-6 * swing, "pendulum never gains energy", (energy1 - energy0) / swing);
    check(energy0 - energy1 < 0.35 * swing, "pendulum keeps most of its energy over 20 s", (energy1 - energy0) / swing);
}

void test_air_drag_terminal_velocity() {
    seed_rng(1);
    World w;
    w.height = 6000.0;
    w.air_density_scale = 20.0;       // thick air, so the limit is reached quickly
    PhysicsObject* ball = still(w.add_sphere(Vec3(450, 5900, 250), 18.0, Material::Foam));
    run(300, w);
    const double rho = World::kAirDensity * w.air_density_scale;
    const double expected = std::sqrt(2.0 * ball->mass * w.gravity.length() / (rho * 0.47 * kPi * 18.0 * 18.0));
    near(-ball->velocity.y, expected, 0.02 * expected, "terminal velocity √(2mg / ρ C_d A)");
}

void test_motor_and_car_drive() {
    seed_rng(1);
    World w;
    PhysicsObject* m = still(w.spawn_motor(Vec3(100, 20, 250)));
    run(120, w);
    check(m->position.x > 300.0, "motor wheel drives itself along the floor", m->position.x);

    World c;
    c.spawn_car(Vec3(150, 40, 250));
    PhysicsObject* chassis = c.objects.front().get();
    run(60, c);
    near(chassis->position.y, 36.0, 4.0, "car sits on its suspension");
    run(180, c);
    check(chassis->position.x > 450.0, "car drives forward", chassis->position.x);
    near(chassis->position.z, 250.0, 25.0, "car drives straight");
    const Vec3 up = chassis->orientation.rotate(Vec3(0, 1, 0));
    check(up.y > 0.95, "car stays upright", up.y);
    double worst = 0.0;
    for (const auto& s : c.springs) worst = std::max(worst, std::abs((s.end_a() - s.end_b()).length() - s.length));
    check(worst < 6.0, "wheels stay on their struts", worst);
}

void test_water_settles_and_levels() {
    seed_rng(2);
    World w = make_pool(3, 600);
    int n = 0;
    double low = 1e9, densest = 0.0;
    int quadrant[4] = {0, 0, 0, 0};
    for (const auto& o : w.objects) {
        if (!o->is_fluid) continue;
        ++n;
        low = std::min(low, o->position.y);
        densest = std::max(densest, o->fluid_density);
        quadrant[(o->position.x > 200.0 ? 1 : 0) + (o->position.z > 150.0 ? 2 : 0)]++;
    }
    check(n == 3 * 512, "no water lost", n);
    const int lo = *std::min_element(quadrant, quadrant + 4), hi = *std::max_element(quadrant, quadrant + 4);
    check(double(lo) / hi > 0.9, "water spreads evenly over the tank", double(lo) / hi);
    check(water_kinetic_energy(w) < 0.01, "water comes to rest", water_kinetic_energy(w));
    // 1536 litres over a 200 × 140 cm floor is 54.9 cm deep.
    const double depth = water_surface(w) - 8.0;
    const double ideal = n * 1000.0 / (200.0 * 140.0);
    near(depth, ideal, 0.12 * ideal, "pool depth matches the volume of water (incompressible)");
    check(densest < 1.08, "no particle is compressed by more than 8 %", densest);
    check(low > 8.0 + 3.0, "water rests on the tank floor, not in it", low);
}

void test_buoyancy() {
    seed_rng(2);
    World w = make_pool(3);
    const double surface = water_surface(w);
    w.spawn_material = Material::Wood;
    PhysicsObject* wood = still(w.add_sphere(Vec3(150, surface + 30, 110), 15.0, Material::Wood));
    PhysicsObject* foam = still(w.add_sphere(Vec3(250, surface + 30, 110), 15.0, Material::Foam));
    PhysicsObject* steel = still(w.add_sphere(Vec3(150, surface + 30, 190), 12.0, Material::Steel));
    PhysicsObject* plank = still(w.add_box(Vec3(250, surface + 30, 190), Vec3(22, 8, 16), Material::Wood));
    plank->orientation = plank->previous_orientation = Quat::from_axis_angle(Vec3(0, 0, 1), 0.5);
    run(700, w);
    const double level = water_surface(w);

    // Archimedes: a floating body displaces its own weight, so the submerged
    // volume fraction equals its relative density.
    auto submerged_fraction = [&](const PhysicsObject* ball) {
        const double r = ball->radius;
        const double d = clampd(level - (ball->position.y - r), 0.0, 2.0 * r);
        return d * d * (3.0 * r - d) / (4.0 * r * r * r);
    };
    // (The surface of a liquid made of 10 cm particles is only defined to a few cm.)
    near(submerged_fraction(wood), 0.65, 0.15, "wooden ball floats 65 % submerged");
    near(submerged_fraction(foam), 0.10, 0.12, "foam ball floats high");
    check(submerged_fraction(foam) < submerged_fraction(wood) - 0.3, "foam rides much higher than wood");
    near(steel->position.y, 8.0 + steel->radius, 1.0, "steel ball sinks to the bottom of the tank");
    const Vec3 up = plank->orientation.rotate(Vec3(0, 1, 0));
    check(std::abs(up.y) > 0.97, "wooden plank floats flat", up.y);
    near((level - (plank->position.y - 8.0)) / 16.0, 0.65, 0.2, "plank floats 65 % submerged");
    double worst = 0.0;
    for (int f = 0; f < 120; ++f) {
        w.step();
        worst = std::max({worst, std::abs(wood->velocity.y), std::abs(foam->velocity.y), std::abs(plank->velocity.y)});
    }
    check(worst < 0.3, "floating bodies settle", worst);
    check(water_kinetic_energy(w) < 0.02, "and leave the water calm", water_kinetic_energy(w));
}

void test_splash_is_bounded() {
    // Regression: a slab dropped flat on the water used to blast it to the
    // ceiling, because nothing slowed the slab as it hit.
    seed_rng(2);
    World w = make_pool(2, 300);
    PhysicsObject* slab = still(w.add_box(Vec3(200, 200, 150), Vec3(30, 10, 30), Material::Wood));
    double top = 0.0, fastest = 0.0;
    for (int f = 0; f < 300; ++f) {
        w.step();
        for (const auto& o : w.objects)
            if (o->is_fluid) { top = std::max(top, o->position.y); fastest = std::max(fastest, o->velocity.length()); }
    }
    int outside = 0;
    for (const auto& o : w.objects)
        if (o->is_fluid && (std::abs(o->position.x - 200.0) > 100.0 || std::abs(o->position.z - 150.0) > 70.0)) ++outside;
    check(top < 140.0, "a slab dropped 1.5 m splashes less than a metre high", top);
    check(fastest < 30.0, "no water is shot out at high speed", fastest);
    check(outside == 0, "the water stays in the tank", outside);
    check(slab->position.y > 20.0 && slab->position.y < 60.0, "the slab ends up floating", slab->position.y);
}

void test_ice_floats() {
    seed_rng(2);
    World w = make_pool(3);
    const double surface = water_surface(w);
    w.ambient_temperature = -30.0;                     // spawn the block frozen
    w.spawn_water(Vec3(200, surface + 30, 150), 7, 3, 7);
    w.ambient_temperature = 20.0;
    w.ambient_setpoint = -2.0;                         // keep it from melting during the test
    for (auto& o : w.objects) if (o->is_fluid) o->temperature = 1.0;
    run(500, w);
    double top = -1e9, bottom = 1e9;
    int bonds = 0, ice = 0;
    for (const auto& o : w.objects)
        if (o->is_ice) { ++ice; top = std::max(top, o->position.y); bottom = std::min(bottom, o->position.y); }
    for (const auto& l : w.rigid_links) bonds += l.ice_bond;
    check(ice >= 100, "most of the block stays frozen", ice);
    check(bonds > 400, "frozen block is bonded into a floe", bonds);
    check(bottom > 8.0 + 15.0, "floe floats clear of the bottom", bottom);
    const double level = water_surface(w);
    const double above = (top + 5.0 - level) / (top - bottom + 10.0);
    check(above > -0.05 && above < 0.3, "ice floats mostly submerged, its top near the surface", above);
}

void test_vessel_gas_laws() {
    seed_rng(3);
    World w;
    w.spawn_container(Vec3(450, 250, 250));
    check(w.containers.size() == 1 && w.containers[0].piston, "vessel has a piston");
    const GasContainer& c = w.containers[0];
    run(600, w);
    near(c.average_temperature, 20.0, 1.0, "sealed air stays at room temperature (no drift)");
    check(c.gas_particles == World::kVesselAirParticles, "no gas leaks out of the vessel", c.gas_particles);
    const double p0 = c.pressure, h0 = c.chamber_height();
    near(p0, thermo::kAtmosphere + World::kPistonMass * w.gravity.length() / (c.bore_area() * World::kPistonForceScale),
         1.5, "piston floats where gas pressure balances atmosphere plus its weight");

    // Charles's law: heated at constant pressure, V ∝ T.
    w.spawn_heater(Vec3(450, 190, 250));               // inside the chamber
    run(1500, w);
    const double t_ratio = thermo::kelvin(c.average_temperature) / thermo::kelvin(20.0);
    check(t_ratio > 1.15, "heater warms the sealed gas", t_ratio);
    near(c.pressure, p0, 2.5, "heated vessel expands at constant pressure");
    const double heater_volume = 8.0 * 32.0 * 7.0 * 32.0;
    const double v_ratio = (c.chamber_height() * c.bore_area() - heater_volume) / (h0 * c.bore_area());
    near(v_ratio, t_ratio, 0.06 * t_ratio, "Charles's law: V ∝ T");

    // Boyle's law: pushing the piston down quickly raises the pressure
    // (adiabatically, so by more than the volume ratio alone).
    seed_rng(3);
    World b;
    b.spawn_container(Vec3(450, 250, 250));
    run(300, b);
    const GasContainer& bc = b.containers[0];
    const double p1 = bc.pressure, v1 = bc.gas_volume;
    b.begin_drag(bc.piston);
    for (int f = 0; f < 30; ++f) { b.drag_to(bc.piston->position - Vec3(0, 1.0, 0)); b.step(); }
    const double ratio = v1 / bc.gas_volume;
    check(ratio > 1.2, "piston was pushed in", ratio);
    check(bc.pressure > p1 * ratio * 0.97 && bc.pressure < p1 * std::pow(ratio, 1.4) * 1.05,
          "compression raises pressure between the isothermal and adiabatic limits", bc.pressure / p1);
    check(bc.average_temperature > 22.0, "fast compression heats the gas", bc.average_temperature);
    b.end_drag(Vec3());

    // A weight on the piston compresses the gas until it carries it.
    seed_rng(3);
    World u;
    u.spawn_container(Vec3(450, 250, 250));
    run(300, u);
    const double before = u.containers[0].pressure;
    PhysicsObject* weight = still(u.add_sphere(Vec3(450, u.containers[0].piston->position.y + 40, 250), 22.0, Material::Steel));
    run(600, u);
    const double extra = weight->mass * u.gravity.length() / (u.containers[0].bore_area() * World::kPistonForceScale);
    near(u.containers[0].pressure - before, extra, 0.25 * extra + 0.5, "gas carries a weight resting on the piston");
    check(weight->position.y > u.containers[0].piston->position.y, "weight rests on top of the piston", weight->position.y);
}

void test_boiling_follows_pressure() {
    seed_rng(5);
    World w;
    w.spawn_container(Vec3(450, 250, 250));
    const GasContainer& c = w.containers[0];
    w.spawn_water(Vec3(450, c.bottom + 20, 250), 6, 2, 6);
    run(200, w);
    check(c.liquid_particles == 72, "water sits inside the vessel", c.liquid_particles);
    near(c.boiling_point, thermo::boiling_point(c.pressure), 1e-9, "boiling point follows the vessel pressure");
    check(c.boiling_point > 100.0, "water under the loaded piston boils above 100 °C", c.boiling_point);
}

void test_blast() {
    seed_rng(4);
    World w;
    w.spawn_gas_cloud(Vec3(450, 300, 250));
    run(120, w);
    PhysicsObject* foam = still(w.add_sphere(Vec3(560, 300, 250), 18.0, Material::Foam));
    PhysicsObject* steel = still(w.add_sphere(Vec3(340, 300, 250), 18.0, Material::Steel));
    w.gravity = Vec3();
    PhysicsObject* bomb = still(w.spawn_bomb(Vec3(450, 300, 250)));
    bomb->fuse = 1;
    run(1, w);
    check(foam->velocity.x > 20.0 * steel->velocity.length() && steel->velocity.x < 0.0,
          "a blast throws a light body much faster than a heavy one of the same size", foam->velocity.x);
    run(30, w);
    double mean = 0.0, hottest = -1e9;
    int n = 0;
    for (const auto& o : w.objects)
        if (o->is_gas) { mean += o->temperature; hottest = std::max(hottest, o->temperature); ++n; }
    mean /= n;
    check(hottest > 60.0, "blast dissipation heats gas near the bomb", hottest);
    check(hottest <= thermo::kMaxGasTemperature + 1e-6, "gas heating is bounded", hottest);
    check(mean < 120.0, "blast heating is local, not the whole cloud", mean);
}

void test_heat_flows() {
    seed_rng(6);
    World w;
    w.gravity = Vec3();
    PhysicsObject* hot = still(w.add_sphere(Vec3(300, 300, 250), 18.0, Material::Steel));
    PhysicsObject* cold = still(w.add_sphere(Vec3(336.5, 300, 250), 18.0, Material::Steel));
    hot->temperature = 200.0;
    cold->temperature = 0.0;
    const double energy0 = hot->thermal_mass() * 200.0;
    run(150, w);
    check(hot->temperature < 170.0 && cold->temperature > 30.0, "heat conducts between touching bodies", cold->temperature);
    check(hot->temperature > cold->temperature, "heat never overshoots equilibrium", hot->temperature - cold->temperature);
    check(hot->thermal_mass() * hot->temperature + cold->thermal_mass() * cold->temperature < energy0 * 1.001,
          "conduction creates no energy");

    World r;
    r.spawn_heater(Vec3(450, 300, 250));
    run(600, r);
    check(r.ambient_temperature > 21.0, "a heater warms the room", r.ambient_temperature);

    // A heater sealed in a vessel reaches loose gas outside only through the glass.
    seed_rng(6);
    World v;
    v.spawn_container(Vec3(450, 250, 250));
    v.spawn_heater(Vec3(450, 190, 250));
    v.spawn_gas_cloud(Vec3(600, 200, 250));
    run(400, v);
    double inside = 0.0, outside = 0.0;
    int n_in = 0, n_out = 0;
    for (const auto& o : v.objects) {
        if (!o->is_gas) continue;
        if (o->container_index >= 0) { inside += o->temperature; ++n_in; }
        else { outside += o->temperature; ++n_out; }
    }
    check(n_in > 0 && n_out > 0 && inside / n_in > 50.0, "gas sealed in with a heater gets hot", inside / std::max(1, n_in));
    check(outside / std::max(1, n_out) < 35.0, "gas outside the vessel stays near room temperature", outside / std::max(1, n_out));
}

void test_pulley() {
    seed_rng(6);
    World w = vacuum();
    PhysicsObject* light = still(w.add_sphere(Vec3(400, 300, 250), 10.0, Material::Wood));
    PhysicsObject* heavy = still(w.add_sphere(Vec3(600, 300, 250), 10.0, Material::Steel));
    w.connect_pulley(light, heavy);
    const double total = w.pulleys[0].total_length;
    run(40, w);
    const Pulley& p = w.pulleys[0];
    const double len = (light->position - p.anchor).length() + (heavy->position - p.anchor).length();
    check(len <= total * 1.01, "pulley rope is inextensible", len / total);
    check(heavy->position.y < 280.0 && light->position.y > 320.0, "Atwood machine: heavy side descends",
          heavy->position.y);
}

void test_serialization_round_trip() {
    seed_rng(8);
    World w;
    w.spawn_container(Vec3(250, 250, 250));
    w.spawn_tank(Vec3(700, 0, 250));
    w.spawn_rope(Vec3(450, 500, 250));
    w.spawn_car(Vec3(450, 60, 380));
    w.spawn_water(Vec3(700, 60, 250), 5, 5, 5);
    w.spawn_material = Material::Steel;
    PhysicsObject* a = w.spawn_ball(Vec3(450, 300, 100));
    PhysicsObject* b = w.spawn_box(Vec3(520, 300, 100));
    w.connect_spring(a, b);
    w.connect_piston(a, b);
    w.connect_pulley(a, b);
    w.spawn_polygon({Vec3(100, 400, 400), Vec3(180, 400, 400), Vec3(140, 470, 400)});
    run(30, w);
    const std::string first = w.serialize();

    World copy;
    std::string error;
    check(copy.deserialize(first, &error), "scene loads: " + error);
    check(copy.serialize() == first, "save → load → save is lossless");
    check(copy.containers.size() == 1 && copy.containers[0].piston != nullptr, "vessel piston relinked");
    check(copy.axles.size() == 4 && copy.springs.size() == 13, "car joints restored", double(copy.springs.size()));
    check(copy.clusters.size() == 1 && copy.clusters[0].nodes.size() == w.clusters[0].nodes.size(), "drawn shape restored");
    check(copy.spawn_material == Material::Steel, "selected material restored");

    const std::size_t before = copy.objects.size();
    check(!copy.deserialize("SAPPHIRE 3\nobj 0 1 2\nend\n", &error), "corrupt file is rejected");
    check(!copy.deserialize("SAPPHIRE 2\nend\n", &error) && error.find("2D") != std::string::npos,
          "a 2D scene is refused with an explanation");
    check(copy.objects.size() == before, "failed load leaves the scene untouched");
    run(5, copy);   // the loaded scene runs
}

void test_editing() {
    seed_rng(9);
    World w;
    w.gravity = Vec3();
    PhysicsObject* ball = still(w.spawn_ball(Vec3(200, 200, 250)));
    PhysicsObject* target = still(w.spawn_ball(Vec3(300, 200, 250)));
    double t = 0.0;
    check(w.pick(Vec3(200, 200, 600), Vec3(0, 0, -1), World::Pick::Grab, &t) == ball, "a ray picks the ball it hits");
    near(t, 350.0 - 18.0, 1e-6, "pick distance");
    check(w.pick(Vec3(250, 200, 600), Vec3(0, 0, -1)) == nullptr, "a ray between two balls hits nothing");
    check(w.begin_drag(ball), "ball can be grabbed");
    for (int i = 0; i < 10; ++i) { w.drag_to(Vec3(200 + 10.0 * (i + 1), 200, 250)); w.step(); }
    check(ball->position.x > 260.0, "dragged body follows the mouse", ball->position.x);
    check(target->velocity.x > 0.1, "dragged body pushes what it touches", target->velocity.x);
    w.end_drag(Vec3(12, 0, 3));
    near(ball->velocity.x, 12.0, 1e-9, "release applies the throw velocity");

    // Glass walls are click-through, cannot be dragged, and go together.
    w.spawn_tank(Vec3(600, 0, 250));
    PhysicsObject* inside = still(w.add_sphere(Vec3(600, 40, 250), 10.0, Material::Wood));
    check(w.pick(Vec3(600, 40, 600), Vec3(0, 0, -1)) == inside, "contents are picked through the glass");
    PhysicsObject* wall = w.pick(Vec3(600, 120, 600), Vec3(0, 0, -1));
    check(wall && wall->is_fixture, "glass is picked when nothing is behind it");
    check(w.pick(Vec3(600, 120, 600), Vec3(0, 0, -1), World::Pick::Grab) == nullptr, "glass cannot be grabbed");
    check(!w.begin_drag(wall), "tank walls cannot be dragged");
    const std::size_t before = w.objects.size();
    w.remove_object(wall);
    check(w.objects.size() == before - 5, "removing one wall removes the whole tank", double(before - w.objects.size()));
}

void test_shapes() {
    seed_rng(10);
    World w;
    const std::vector<Vec3> outline = {Vec3(300, 300, 250), Vec3(420, 300, 250), Vec3(420, 380, 250), Vec3(300, 380, 250)};
    check(w.spawn_polygon(outline), "outline becomes a shape");
    check(!w.spawn_polygon({Vec3(0, 0, 0), Vec3(10, 0, 0)}), "two points are not a shape");
    check(w.clusters.size() == 1, "one rigid body per outline");
    const RigidCluster& shape = w.clusters[0];
    const std::size_t beads = shape.nodes.size();
    auto distortion = [&]() {
        double worst = 0.0;
        for (std::size_t i = 0; i < beads; ++i)
            for (std::size_t j = i + 1; j < beads; ++j)
                worst = std::max(worst, std::abs((shape.nodes[i]->position - shape.nodes[j]->position).length()
                                                 - (shape.rest[i] - shape.rest[j]).length()));
        return worst;
    };
    PhysicsObject* ball = still(w.add_sphere(Vec3(330, 480, 250), 18.0, Material::Steel));    // lands on the shape
    double worst = 0.0;
    for (int f = 0; f < 700; ++f) { w.step(); if (f % 10 == 0) worst = std::max(worst, distortion()); }
    check(worst < 0.2, "drawn shape stays rigid while falling and being hit (cm)", worst);
    double ke = 0.0, lowest = 1e9;
    for (const PhysicsObject* o : shape.nodes) { ke += o->kinetic_energy(); lowest = std::min(lowest, o->position.y); }
    check(ke / double(beads) < 0.02, "drawn shape comes to rest", ke / double(beads));
    near(lowest, 7.0, 0.3, "and rests on the floor");
    check(ball->position.y < 60.0, "ball ends up on the floor too", ball->position.y);

    const std::size_t before = w.objects.size();
    w.remove_object(shape.nodes[3]);
    check(w.objects.size() == before - beads && w.clusters.empty(), "removing one bead removes the whole shape");
}

void benchmark() {
    seed_rng(12);
    World w;
    w.spawn_tank(Vec3(250, 0, 250));
    for (int k = 0; k < 3; ++k) w.spawn_water(Vec3(250, 60 + 85.0 * k, 250));
    w.spawn_container(Vec3(650, 250, 250));
    w.spawn_gas_cloud(Vec3(450, 400, 250));
    w.spawn_car(Vec3(450, 60, 420));
    for (int k = 0; k < 20; ++k) w.spawn_ball(Vec3(100 + 35.0 * k, 450, 100 + 10.0 * (k % 5)));
    for (int k = 0; k < 10; ++k) w.spawn_box(Vec3(120 + 60.0 * k, 520, 150));
    run(120, w);
    const auto start = std::chrono::steady_clock::now();
    run(120, w);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count() / 120.0;
    std::printf("  benchmark: %zu bodies (1536 water, 620 gas), %.2f ms per frame of physics\n", w.objects.size(), ms);
    check(ms < 200.0, "heavy scene simulates", ms);
}
} // namespace

int main(int argc, char* argv[]) {
    const char* filter = argc > 1 ? argv[1] : "";
    const std::vector<std::pair<const char*, std::function<void()>>> tests = {
        {"thermo models", test_thermo},
        {"phase changes", test_phase_changes},
        {"materials", test_materials},
        {"broad phase", test_broad_phase_finds_every_pair},
        {"box geometry", test_box_geometry},
        {"free fall, bounce", test_free_fall_and_bounce},
        {"resting contact", test_ball_comes_to_rest},
        {"rolling friction", test_sliding_ball_starts_rolling},
        {"ramp friction", test_box_friction_on_a_ramp},
        {"box stacking", test_boxes_stack_and_tumble},
        {"rope, pendulum", test_rope_and_pendulum},
        {"air drag", test_air_drag_terminal_velocity},
        {"motors", test_motor_and_car_drive},
        {"water", test_water_settles_and_levels},
        {"buoyancy", test_buoyancy},
        {"splash", test_splash_is_bounded},
        {"ice", test_ice_floats},
        {"vessel gas laws", test_vessel_gas_laws},
        {"boiling point", test_boiling_follows_pressure},
        {"blast", test_blast},
        {"heat", test_heat_flows},
        {"pulley", test_pulley},
        {"serialization", test_serialization_round_trip},
        {"editing", test_editing},
        {"drawn shapes", test_shapes},
        {"benchmark", benchmark},
    };
    for (const auto& [name, fn] : tests) {
        if (*filter && !std::strstr(name, filter)) continue;
        const int before = g_failures;
        const auto start = std::chrono::steady_clock::now();
        fn();
        const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        std::printf("%-18s %s  (%.1f s)\n", name, g_failures == before ? "ok" : "FAILED", s);
    }
    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
