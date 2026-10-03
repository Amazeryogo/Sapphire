// Tests for the Qt-free physics core.  Run: ./sapphire_tests  (exit code 0 = pass)

#include "random.h"
#include "spatial_hash.h"
#include "thermo.h"
#include "world.h"

#include <algorithm>
#include <cstdio>
#include <functional>
#include <set>
#include <string>
#include <vector>

namespace {
int g_failures = 0;
int g_checks = 0;

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

double water_surface(const World& w, double avoid_x = -1e9, double avoid_r = 0.0) {
    double s = 1e9;
    for (const auto& o : w.objects)
        if (o->is_fluid && std::abs(o->position.x - avoid_x) > avoid_r) s = std::min(s, o->position.y);
    return s;
}

World make_pool(double width = 400.0, double height = 600.0) {
    World w;
    w.width = width;
    w.height = height;
    for (int k = 0; k < 4; ++k) w.spawn_water(20 + (k % 2) * 180, 100 + (k / 2) * 130);
    run(400, w);
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
    near(thermo::elastomer_restitution_factor(20.0), 1.0, 1e-9, "rubber normalised at 20 °C");
    check(thermo::elastomer_restitution_factor(-45.0) < 0.4, "rubber dead at glass transition",
          thermo::elastomer_restitution_factor(-45.0));
}

void test_phase_changes() {
    PhysicsObject water(0, 0, 6.0, 0.1, 1.0, Rgba{-1, -1, -1, -1}, ShapeType::CIRCLE, false, true);
    water.temperature = 20.0;
    water.add_heat(-20.0 * water.thermal_mass());        // cool to 0 °C
    near(water.temperature, 0.0, 1e-9, "water cools to melting point");
    check(water.is_fluid, "still liquid at 0 °C before latent heat leaves");
    water.add_heat(-thermo::kLatentFusion * water.mass - 1e-6);
    check(water.is_ice && water.phase_event == PhaseEvent::Froze, "freezes after latent heat of fusion");
    check(!water.is_static, "ice is a free body, not frozen in place");
}

void test_broad_phase_finds_every_pair() {
    seed_rng(7);
    std::vector<std::unique_ptr<PhysicsObject>> objects;
    for (int i = 0; i < 600; ++i) {
        const double r = (i % 7 == 0) ? random_double(15.0, 40.0) : random_double(1.0, 8.0);
        objects.push_back(std::make_unique<PhysicsObject>(random_double(-50, 450), random_double(-50, 350), r));
    }
    const double margin = 4.0;
    SpatialHash hash;
    hash.build(objects, margin, 16.0, 400.0, 300.0);
    std::set<std::pair<std::uint32_t, std::uint32_t>> found(hash.pairs().begin(), hash.pairs().end());
    check(found.size() == hash.pairs().size(), "no duplicate pairs");
    int missing = 0;
    for (std::uint32_t i = 0; i < objects.size(); ++i)
        for (std::uint32_t j = i + 1; j < objects.size(); ++j) {
            const Vector d = objects[i]->position - objects[j]->position;
            const double reach = objects[i]->radius + objects[j]->radius + margin;
            if (std::abs(d.x) <= reach && std::abs(d.y) <= reach && !found.count({i, j})) ++missing;
        }
    check(missing == 0, "broad phase misses no overlapping pair", missing);
}

void test_rope_does_not_stretch() {
    seed_rng(1);
    World w;
    w.spawn_rope(500, 100);
    run(600, w);
    double worst = 0.0;
    for (const auto& l : w.rigid_links)
        worst = std::max(worst, (l.obj_a->position - l.obj_b->position).length() / l.length);
    check(worst < 1.01, "hanging rope links stay within 1 % of length", worst);
    check(std::abs(w.objects.back()->velocity.y) < 0.05, "hanging rope has no hidden velocity",
          w.objects.back()->velocity.y);
}

void test_ball_comes_to_rest() {
    seed_rng(1);
    World w;
    PhysicsObject* ball = w.spawn_ball(500, 300);
    ball->velocity = Vector();
    run(900, w);
    near(ball->position.y, w.height - ball->radius, 0.2, "ball rests on the floor");
    near(ball->velocity.y, 0.0, 0.01, "resting ball does not jitter");
}

void test_sliding_ball_starts_rolling() {
    seed_rng(1);
    World w;
    PhysicsObject* ball = w.spawn_ball(100, w.height - 18.0);
    ball->velocity = Vector(10, 0);
    run(30, w);
    near(ball->angular_velocity * ball->radius, ball->velocity.x, 0.05, "friction makes the ball roll (v = ωR)");
    near(ball->velocity.x, 10.0 / 1.5 * std::pow(0.99, 30), 0.3, "disc keeps 2/3 of its speed when it starts rolling");
}

void test_motor_and_car_drive() {
    seed_rng(1);
    World w;
    PhysicsObject* m = w.spawn_motor(100, 700);
    m->velocity = Vector();
    run(120, w);
    check(m->position.x > 300.0, "motor wheel drives itself along the floor", m->position.x);

    World c;
    c.spawn_car(200, 700);
    run(180, c);
    double mean_x = 0.0;
    for (const auto& o : c.objects) mean_x += o->position.x / double(c.objects.size());
    check(mean_x > 450.0, "car drives forward", mean_x);
}

void test_water_settles_and_levels() {
    seed_rng(2);
    World w;
    w.width = 1000;
    w.height = 540;
    w.spawn_water(60, 330);
    w.spawn_water(200, 330);
    run(900, w);
    int columns[10] = {0};
    double ke = 0.0;
    int n = 0;
    for (const auto& o : w.objects) {
        if (!o->is_fluid) continue;
        columns[std::min(9, int(o->position.x / 100))]++;
        ke += 0.5 * o->mass * o->velocity.length_sq();
        ++n;
    }
    const int lo = *std::min_element(columns, columns + 10), hi = *std::max_element(columns, columns + 10);
    check(double(lo) / hi > 0.7, "two drops spread into a level layer", double(lo) / hi);
    check(ke / n < 0.01, "water comes to rest", ke / n);
}

void test_buoyancy() {
    seed_rng(2);
    World w = make_pool();
    const double surface = water_surface(w);
    PhysicsObject* ball = w.spawn_ball(80, surface - 80);            // rubber: ~25 % of water's density
    PhysicsObject* bomb = w.spawn_bomb(320, surface - 80);           // iron
    bomb->fuse = -1;
    PhysicsObject* neutral = w.spawn_at(200, surface + 60, ShapeType::CIRCLE, 15,
                                        World::kWaterDensity * 3.14159265 * 15 * 15);
    for (PhysicsObject* o : {ball, bomb, neutral}) o->velocity = Vector();
    run(480, w);
    const double s = water_surface(w, 80, 40);
    check(ball->position.y - ball->radius < s, "light ball floats with its top above water", ball->position.y);
    check(ball->position.y + ball->radius > s, "floating ball is partly submerged", ball->position.y);
    near(bomb->position.y, w.height - bomb->radius, 1.0, "iron bomb sinks to the bottom");
    check(neutral->position.y > s + 20 && neutral->position.y < w.height - 40,
          "body as dense as water hovers mid-water", neutral->position.y);
}

void test_bodies_settle_in_shallow_water() {
    // Regression: a body in a film of water thinner than its float depth used
    // to keep bouncing (and stirring the water) forever.
    seed_rng(4);
    World w;
    w.width = 1000;
    w.height = 600;
    w.spawn_water(100, 300);
    w.spawn_water(250, 300);
    run(900, w);
    PhysicsObject* ball = w.spawn_ball(500, 400);
    PhysicsObject* box = w.spawn_box(700, 400);
    ball->velocity = box->velocity = Vector();
    run(600, w);
    double worst = 0.0, ke = 0.0;
    int n = 0;
    for (int f = 0; f < 300; ++f) {
        w.step();
        worst = std::max({worst, std::abs(ball->velocity.y), std::abs(box->velocity.y)});
    }
    for (const auto& o : w.objects)
        if (o->is_fluid) { ke += 0.5 * o->mass * o->velocity.length_sq(); ++n; }
    check(worst < 0.5, "bodies in shallow water settle", worst);
    check(ke / n < 0.01, "and leave the water calm", ke / n);
}

void test_ice_floats() {
    seed_rng(2);
    World w;
    w.width = 300;
    w.height = 600;
    for (int k = 0; k < 6; ++k) w.spawn_water(10 + (k % 2) * 150, 60 + (k / 2) * 125);
    run(400, w);
    w.ambient_temperature = -30.0;                     // spawn the block frozen
    w.spawn_water(90, 40);
    w.ambient_temperature = 20.0;
    w.ambient_setpoint = -2.0;                         // keep it from melting during the test
    run(400, w);
    double top = 1e9, bottom = -1e9;
    int bonds = 0;
    for (const auto& o : w.objects)
        if (o->is_ice) { top = std::min(top, o->position.y); bottom = std::max(bottom, o->position.y); }
    for (const auto& l : w.rigid_links) bonds += l.ice_bond;
    double surface = 1e9;
    for (const auto& o : w.objects)
        if (o->is_fluid && (o->position.x < 70 || o->position.x > 230)) surface = std::min(surface, o->position.y);
    const double above = (surface - top) / (bottom - top);
    check(bonds > 300, "frozen block is bonded into a floe", bonds);
    check(bottom < w.height - 20.0, "floe floats clear of the bottom", bottom);
    check(above > -0.05 && above < 0.25, "ice floats mostly submerged, top at the surface", above);
}

void test_vessel_gas_laws() {
    seed_rng(3);
    World w;
    w.spawn_container(500, 600);
    const GasContainer& c = w.containers[0];
    run(600, w);
    near(c.average_temperature, 20.0, 1.0, "sealed air stays at room temperature (no drift)");
    const double p0 = c.pressure, h0 = c.chamber_height();
    w.spawn_heater(500, 720);
    run(1200, w);
    const double t_ratio = thermo::kelvin(c.average_temperature) / thermo::kelvin(20.0);
    near(c.pressure, p0, 2.0, "heated vessel expands at constant pressure");
    near(c.chamber_height() / h0, t_ratio, 0.05 * t_ratio, "Charles's law: V ∝ T");

    // Regression: bodies just under a vessel must not count as sitting on its
    // piston (they used to be thrown up through it, and to crush the gas).
    seed_rng(3);
    World u;
    u.height = 722;                                   // floor just under the vessel (bottom at 690)
    u.spawn_container(500, 600);
    for (int k = 0; k < 5; ++k) u.spawn_ball(420 + 40 * k, 704);
    run(300, u);
    bool all_below = true;
    for (const auto& o : u.objects)
        if (!o->is_gas && !o->is_container_piston && o->position.y < u.containers[0].bottom) all_below = false;
    check(all_below, "bodies under a vessel stay under it (are not thrown through it)");
    near(u.containers[0].pressure, p0, 3.0, "bodies under a vessel do not press on its piston");
}

void test_blast_heats_gas_locally() {
    seed_rng(4);
    World w;
    w.spawn_gas_cloud(500, 400);
    run(120, w);
    PhysicsObject* bomb = w.spawn_bomb(520, 400);
    bomb->fuse = 1;
    run(31, w);
    double mean = 0.0, hottest = -1e9;
    int n = 0;
    for (const auto& o : w.objects)
        if (o->is_gas) { mean += o->temperature; hottest = std::max(hottest, o->temperature); ++n; }
    mean /= n;
    check(hottest > 60.0, "blast dissipation heats gas near the bomb", hottest);
    check(hottest <= thermo::kMaxGasTemperature + 1e-6, "gas heating is bounded", hottest);
    check(mean < 80.0, "blast heating is local, not the whole cloud", mean);
}

void test_pulley() {
    seed_rng(6);
    World w;
    PhysicsObject* light = w.spawn_at(400, 400, ShapeType::CIRCLE, 10, 1.0);
    PhysicsObject* heavy = w.spawn_at(600, 400, ShapeType::CIRCLE, 10, 3.0);
    light->velocity = heavy->velocity = Vector();
    w.connect_pulley(light, heavy);
    const double total = w.pulleys[0].total_length;
    run(40, w);
    const Pulley& p = w.pulleys[0];
    const double len = (light->position - p.anchor).length() + (heavy->position - p.anchor).length();
    check(len <= total * 1.01, "pulley rope is inextensible", len / total);
    check(heavy->position.y > 420.0 && light->position.y < 380.0, "Atwood machine: heavy side descends",
          heavy->position.y);
}

void test_serialization_round_trip() {
    seed_rng(8);
    World w;
    w.spawn_container(300, 400);
    w.spawn_rope(600, 50);
    w.spawn_car(700, 600);
    w.spawn_water(100, 100);
    PhysicsObject* a = w.spawn_ball(500, 300);
    PhysicsObject* b = w.spawn_ball(560, 300);
    w.connect_spring(a, b);
    w.connect_piston(a, b);
    w.connect_pulley(a, b);
    run(30, w);
    const std::string first = w.serialize();

    World copy;
    std::string error;
    check(copy.deserialize(first, &error), "scene loads: " + error);
    check(copy.serialize() == first, "save → load → save is lossless");
    check(copy.containers.size() == 1 && copy.containers[0].piston != nullptr, "vessel piston relinked");

    const std::size_t before = copy.objects.size();
    check(!copy.deserialize("SAPPHIRE 2\nobj 0 1 2\nend\n", &error), "corrupt file is rejected");
    check(copy.objects.size() == before, "failed load leaves the scene untouched");
    run(5, copy);   // the loaded scene runs
}

void test_drag_and_throw() {
    seed_rng(9);
    World w;
    w.gravity = Vector();
    PhysicsObject* ball = w.spawn_ball(200, 200);
    PhysicsObject* target = w.spawn_ball(300, 200);
    ball->velocity = target->velocity = Vector();
    w.begin_drag(ball);
    for (int i = 0; i < 10; ++i) { w.drag_to(Vector(200 + 10.0 * (i + 1), 200)); w.step(); }
    check(ball->position.x > 260.0, "dragged body follows the mouse", ball->position.x);
    check(target->velocity.x > 0.1, "dragged body pushes what it touches", target->velocity.x);
    w.end_drag(Vector(12, 0));
    near(ball->velocity.x, 12.0, 1e-9, "release applies the throw velocity");
}
} // namespace

int main() {
    const std::vector<std::pair<const char*, std::function<void()>>> tests = {
        {"thermo models", test_thermo},
        {"phase changes", test_phase_changes},
        {"broad phase", test_broad_phase_finds_every_pair},
        {"rope", test_rope_does_not_stretch},
        {"resting contact", test_ball_comes_to_rest},
        {"rolling friction", test_sliding_ball_starts_rolling},
        {"motors", test_motor_and_car_drive},
        {"water", test_water_settles_and_levels},
        {"buoyancy", test_buoyancy},
        {"shallow water", test_bodies_settle_in_shallow_water},
        {"ice", test_ice_floats},
        {"vessel gas laws", test_vessel_gas_laws},
        {"blast heating", test_blast_heats_gas_locally},
        {"pulley", test_pulley},
        {"serialization", test_serialization_round_trip},
        {"dragging", test_drag_and_throw},
    };
    for (const auto& [name, fn] : tests) {
        const int before = g_failures;
        fn();
        std::printf("%-18s %s\n", name, g_failures == before ? "ok" : "FAILED");
    }
    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
