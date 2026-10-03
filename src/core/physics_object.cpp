#include "physics_object.h"

#include "random.h"

namespace {
constexpr Rgba kWaterColor{60, 150, 255, 200};
constexpr Rgba kIceColor{150, 240, 255, 230};
constexpr Rgba kSteamColor{225, 225, 235, 150};
constexpr std::size_t kTrailLength = 12;
}

PhysicsObject::PhysicsObject(double x, double y, double r, double el, double m, Rgba c,
                             ShapeType st, bool is_stat, bool is_fl)
    : position(x, y), previous_position(x, y), radius(r), liquid_radius(r), elasticity(el),
      mass(m < 0.0 ? (r * r) / 100.0 : m), shape_type(st), is_static(is_stat), is_fluid(is_fl) {
    if (!is_static) velocity = Vector(random_double(-2.0, 2.0), random_double(-2.0, 1.0));
    pre_velocity = velocity;

    if (is_fluid) {
        color = kWaterColor;
    } else if (c.r >= 0) {
        color = c;
    } else {
        color = Rgba{random_int(80, 255), random_int(80, 255), random_int(80, 255), 255};
    }
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
    // Ice is a free rigid solid now: it falls, and floats because the World
    // bonds it into a lattice that is ~9 % less dense than the water.
    is_ice = true;
    radius = liquid_radius * 1.03;   // ice is ~9 % less dense: (1.09)^(1/3)
    elasticity = 0.05; is_bouncy = false;
    angular_velocity = 0.0;
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

void PhysicsObject::update_trail() {
    trail.push_back(position);
    if (trail.size() > kTrailLength) trail.erase(trail.begin());
}
