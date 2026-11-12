#include "physics_object.h"

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr Rgba kWaterColor{45, 135, 245, 255};
constexpr Rgba kIceColor{170, 235, 255, 255};
constexpr Rgba kSteamColor{230, 230, 240, 110};
}

PhysicsObject::PhysicsObject(const Vec3& p, double r)
    : position(p), previous_position(p), radius(r), liquid_radius(r) {
    update_inertia();
}

void PhysicsObject::make_sphere(double r) {
    shape_type = ShapeType::SPHERE;
    radius = r;
    liquid_radius = r;
    half_extents = Vec3();
    update_inertia();
}

void PhysicsObject::make_box(const Vec3& half) {
    shape_type = ShapeType::BOX;
    half_extents = half;
    radius = half.length();
    update_inertia();
}

double PhysicsObject::volume() const {
    if (shape_type == ShapeType::BOX) return 8.0 * half_extents.x * half_extents.y * half_extents.z;
    return 4.0 / 3.0 * kPi * radius * radius * radius;
}

double PhysicsObject::surface_area() const {
    if (shape_type == ShapeType::BOX) {
        const Vec3& h = half_extents;
        return 8.0 * (h.x * h.y + h.y * h.z + h.z * h.x);
    }
    return 4.0 * kPi * radius * radius;
}

double PhysicsObject::support(const Vec3& direction) const {
    if (shape_type != ShapeType::BOX) return radius;
    const Vec3 local = orientation.inv_rotate(direction);
    return std::abs(local.x) * half_extents.x + std::abs(local.y) * half_extents.y + std::abs(local.z) * half_extents.z;
}

void PhysicsObject::set_material(Material m) {
    material = m;
    const MaterialProps& props = material_props(m);
    is_bouncy = (m == Material::Rubber);
    if (!is_bouncy) elasticity = props.restitution;
    set_mass(props.density * kWaterDensity * volume());
}

void PhysicsObject::set_mass(double m) {
    mass = std::max(1e-6, m);
    update_inertia();
}

void PhysicsObject::update_inertia() {
    if (shape_type == ShapeType::BOX) {
        const Vec3& h = half_extents;
        const double k = mass / 3.0;
        const Vec3 inertia(k * (h.y * h.y + h.z * h.z), k * (h.x * h.x + h.z * h.z), k * (h.x * h.x + h.y * h.y));
        inv_inertia_body = Vec3(inertia.x > 0.0 ? 1.0 / inertia.x : 0.0,
                                inertia.y > 0.0 ? 1.0 / inertia.y : 0.0,
                                inertia.z > 0.0 ? 1.0 / inertia.z : 0.0);
    } else {
        const double inertia = 0.4 * mass * radius * radius;      // solid sphere
        const double inv = inertia > 0.0 ? 1.0 / inertia : 0.0;
        inv_inertia_body = Vec3(inv, inv, inv);
    }
}

double PhysicsObject::kinetic_energy() const {
    if (is_static) return 0.0;
    double e = 0.5 * mass * velocity.length_sq();
    if (can_rotate()) {
        const Vec3 w = orientation.inv_rotate(angular_velocity);
        const Vec3& ii = inv_inertia_body;
        if (ii.x > 0.0) e += 0.5 * w.x * w.x / ii.x;
        if (ii.y > 0.0) e += 0.5 * w.y * w.y / ii.y;
        if (ii.z > 0.0) e += 0.5 * w.z * w.z / ii.z;
    }
    return e;
}

double PhysicsObject::effective_restitution() const {
    if (is_gas) return 1.0;              // molecular collisions are elastic
    if (is_static) return 1.0;
    if (is_fluid || is_ice) return elasticity;
    if (!is_bouncy) return elasticity;
    return clampd(elasticity * thermo::elastomer_restitution_factor(temperature), 0.02, 0.98);
}

void PhysicsObject::become_liquid() {
    phase_event = is_ice ? PhaseEvent::Melted : PhaseEvent::Condensed;
    is_ice = false; is_gas = false; is_steam = false;
    is_fluid = true;
    radius = liquid_radius;
    elasticity = 0.1; is_bouncy = false;
    molar_ratio = 1.0;
    color = kWaterColor;
}

void PhysicsObject::become_ice() {
    phase_event = PhaseEvent::Froze;
    is_fluid = false; is_gas = false; is_steam = false;
    // Ice is a free rigid solid: it falls, and floats because the World
    // bonds it into a lattice that is ~9 % less dense than the water.
    is_ice = true;
    radius = liquid_radius * 1.03;   // ice is ~9 % less dense: (1.09)^(1/3)
    elasticity = 0.05; is_bouncy = false;
    angular_velocity = Vec3();
    color = kIceColor;
}

void PhysicsObject::become_steam() {
    phase_event = PhaseEvent::Boiled;
    is_fluid = false; is_ice = false;
    is_gas = true; is_steam = true;
    radius = std::max(2.0, liquid_radius * 0.4);
    elasticity = 1.0; is_bouncy = false;
    molar_ratio = thermo::kSteamMolarMass / thermo::kAirMolarMass;
    color = kSteamColor;
}

void PhysicsObject::add_heat(double energy) {
    if (is_heater || is_cooler || is_container_piston || !is_valid(energy)) return;
    if (!is_water_substance()) {
        temperature = std::max(thermo::kAbsoluteZero + 1.0, temperature + energy / thermal_mass());
        return;
    }

    const double fusion = mass * thermo::kLatentFusion;
    const double vapor = mass * thermo::kLatentVapor;
    const double melt = thermo::kMeltPoint;
    const double boil = boiling_point();

    // A pressure change can leave a phase off its boundary (superheated
    // water after decompression, supersaturated steam after compression).
    // Release that sensible heat into the phase change.
    if (latent_energy == 0.0) {
        if (is_ice && temperature > melt) { energy += (temperature - melt) * thermal_mass(); temperature = melt; }
        else if (is_fluid && temperature > boil) { energy += (temperature - boil) * thermal_mass(); temperature = boil; }
        else if (is_fluid && temperature < melt) { energy += (temperature - melt) * thermal_mass(); temperature = melt; }
        else if (is_steam && temperature < boil) { energy += (temperature - boil) * thermal_mass(); temperature = boil; }
    }

    for (int step = 0; step < 8 && std::abs(energy) > 1e-12; ++step) {
        const double c = thermal_mass();
        if (is_ice) {
            if (latent_energy > 0.0 || (energy > 0.0 && temperature >= melt)) {
                temperature = melt;
                latent_energy += energy; energy = 0.0;
                if (latent_energy >= fusion) { energy = latent_energy - fusion; latent_energy = 0.0; become_liquid(); }
                else if (latent_energy < 0.0) { energy = latent_energy; latent_energy = 0.0; }
            } else {
                const double next = temperature + energy / c;
                if (energy > 0.0 && next > melt) { energy -= (melt - temperature) * c; temperature = melt; }
                else { temperature = std::max(thermo::kAbsoluteZero + 1.0, next); energy = 0.0; }
            }
        } else if (is_fluid) {
            if (latent_energy < 0.0 || (energy < 0.0 && temperature <= melt)) {         // freezing
                temperature = melt;
                latent_energy += energy; energy = 0.0;
                if (latent_energy <= -fusion) { energy = latent_energy + fusion; latent_energy = 0.0; become_ice(); }
                else if (latent_energy > 0.0) { energy = latent_energy; latent_energy = 0.0; }
            } else if (latent_energy > 0.0 || (energy > 0.0 && temperature >= boil)) {   // boiling
                temperature = boil;
                latent_energy += energy; energy = 0.0;
                if (latent_energy >= vapor) { energy = latent_energy - vapor; latent_energy = 0.0; become_steam(); }
                else if (latent_energy < 0.0) { energy = latent_energy; latent_energy = 0.0; }
            } else {                                                                    // sensible
                const double next = temperature + energy / c;
                if (energy > 0.0 && next > boil) { energy -= (boil - temperature) * c; temperature = boil; }
                else if (energy < 0.0 && next < melt) { energy -= (melt - temperature) * c; temperature = melt; }
                else { temperature = next; energy = 0.0; }
            }
        } else { // steam
            if (latent_energy < 0.0 || (energy < 0.0 && temperature <= boil)) {         // condensing
                temperature = boil;
                latent_energy += energy; energy = 0.0;
                if (latent_energy <= -vapor) { energy = latent_energy + vapor; latent_energy = 0.0; become_liquid(); }
                else if (latent_energy > 0.0) { energy = latent_energy; latent_energy = 0.0; }
            } else {
                const double next = temperature + energy / c;
                if (energy < 0.0 && next < boil) { energy -= (boil - temperature) * c; temperature = boil; }
                else { temperature = next; energy = 0.0; }
            }
        }
    }
}

