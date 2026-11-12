// Per-frame thermodynamics: sealed vessels, heat transfer, phase changes,
// the gas thermostat and blasts.

#include "world.h"

#include "collision.h"
#include "random.h"
#include "world_detail.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

using namespace detail;

// ============================================================
// Vessels
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
        double condensed_volume = 0.0;
        double sum_tk = 0.0;
        c.liquid_particles = 0;
        for (const auto& ptr : objects) {
            PhysicsObject* o = ptr.get();
            if (o->is_container_piston || o->is_fixture || o->container_index >= 0 || !c.in_chamber(o->position)) continue;
            o->container_index = int(i);
            if (o->is_gas) { gas_inside.push_back(o); sum_tk += thermo::kelvin(o->temperature); }
            else if (o->is_fluid || o->is_ice) {
                condensed_volume += kFluidSpacing * kFluidSpacing * kFluidSpacing * (o->is_ice ? 1.09 : 1.0);
                ++c.liquid_particles;
            }
            else condensed_volume += o->volume();
        }
        c.gas_particles = int(gas_inside.size());

        const double min_volume = 0.03 * c.bore_area() * (c.piston_max_y - c.bottom);
        c.gas_volume = std::max(min_volume, c.bore_area() * c.chamber_height() - condensed_volume);

        // Adiabatic process: T·V^(γ-1) = const for the piston's displacement.
        if (c.previous_chamber_height > 0.0 && !gas_inside.empty()) {
            const double prev_volume = std::max(min_volume, c.bore_area() * c.previous_chamber_height - condensed_volume);
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
// Heat
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
        // One sealed inside a vessel reaches the room only through its walls.
        air_energy += (source->temperature - t_air) * kSourceAirConductance * (source->container_index >= 0 ? 0.2 : 1.0);
        const double far = source->radius + kSourceReach;
        for (const auto& target_ptr : objects) {
            PhysicsObject* target = target_ptr.get();
            if (target == source || target->is_heater || target->is_cooler || target->is_container_piston) continue;
            const double reach = far + target->radius;
            if ((target->position - source->position).length_sq() >= reach * reach) continue;
            const double gap = std::max(0.0, surface_gap(*source, *target));
            if (gap >= kSourceReach) continue;
            const double proximity = 1.0 - gap / kSourceReach;
            const double diff = source->temperature - target->temperature;
            double coupling = kSourceObjectConductance * target->surface_area() * (0.1 + 0.9 * proximity * proximity);
            if (source->container_index != target->container_index && !target->is_fixture) coupling *= 0.2;   // through a vessel wall
            coupling = std::min(coupling, 0.5 * target->thermal_mass());
            target->add_heat(diff * coupling);
        }
    }

    for (const auto& object_ptr : objects) {
        PhysicsObject* object = object_ptr.get();
        if (object->is_heater || object->is_cooler || object->is_container_piston) continue;
        // Only the free surface of a body of water meets the air.
        if (object->is_fluid && object->fluid_density > 0.8) continue;
        const double diff = object->temperature - t_air;
        const double area = object->surface_area();

        // Newton convection ∝ surface.  Free gas mixes with room air; gas
        // sealed in a vessel only conducts through the wall.
        double factor = 1.0;
        if (object->is_gas) factor = object->container_index >= 0 ? 0.7 : 8.5;
        double coupling = kConvection * area * factor * rate_scale;
        coupling = std::min(coupling, 0.5 * object->thermal_mass());
        double q = diff * coupling;

        // Stefan–Boltzmann radiation (condensed matter only; air is transparent).
        if (!object->is_gas) {
            const double tk = thermo::kelvin(object->temperature), ta = thermo::kelvin(t_air);
            const double q_rad = kRadiation * area * rate_scale * (tk * tk * tk * tk - ta * ta * ta * ta);
            const double cap = std::abs(diff) * 0.5 * object->thermal_mass();
            q += clampd(q_rad, -cap, cap);
        }
        object->add_heat(-q);
        air_energy += q;

        // Evaporative cooling of exposed liquid water: rate ∝ P_sat(T).
        // The latent heat leaves as humidity, not as sensible air heat.
        if (object->is_fluid && object->container_index < 0) {
            const double ratio = thermo::saturation_pressure(object->temperature) / thermo::kAtmosphere;
            object->add_heat(-kEvaporation * std::min(ratio, 1.0) * rate_scale);
        }
    }
    ambient_temperature = clampd(t_air + air_energy / kAirHeatCapacity, -200.0, 700.0);
}

void World::conduct_heat() {
    auto exchange = [&](PhysicsObject* a, PhysicsObject* b, double gap) {
        if (gap >= kConductionReach) return;
        const double contact = 0.55 * clampd(1.0 - gap / kConductionReach, 0.0, 1.0);
        double coupling = heat_transfer_rate * contact * 0.12 * contact_conductance(*a, *b);
        // Larger touching surfaces carry more heat.
        coupling *= clampd(std::min(a->equivalent_radius(), b->equivalent_radius()) / 5.0, 0.3, 6.0);
        if (a->container_index != b->container_index) coupling *= 0.2;      // through a vessel wall
        coupling = limited_coupling(coupling, *a, *b);
        const double q = (a->temperature - b->temperature) * coupling;
        if (q == 0.0) return;
        a->add_heat(-q);
        b->add_heat(q);
    };

    for (const auto& [i, j] : hash_.pairs()) {
        PhysicsObject* a = objects[i].get();
        PhysicsObject* b = objects[j].get();
        if (a->is_container_piston || b->is_container_piston) continue;
        if (a->temperature == b->temperature) continue;
        const bool round = a->shape_type != ShapeType::BOX && b->shape_type != ShapeType::BOX;
        const double gap = round ? (a->position - b->position).length() - a->radius - b->radius
                                 : surface_gap(*a, *b);
        exchange(a, b, gap);
    }
    // Liquid–liquid pairs come from the liquid solver's own neighbour list.
    for (std::size_t p = 0; p < pair_i_.size(); ++p) {
        PhysicsObject* a = fluid_[pair_i_[p]];
        PhysicsObject* b = fluid_[pair_j_[p]];
        if (a->temperature == b->temperature) continue;
        exchange(a, b, (a->position - b->position).length() - kFluidSpacing);
    }
}

// Gas mixes: within each ~40 cm parcel of a gas cloud the particles relax
// towards their common (energy-weighted) temperature.
void World::mix_gas_temperatures() {
    struct Parcel { double energy = 0.0, capacity = 0.0; };
    static thread_local std::unordered_map<std::uint64_t, Parcel> parcels;
    parcels.clear();
    constexpr double kParcel = 40.0;
    auto key = [&](const PhysicsObject& o) {
        const auto q = [](double v) { return std::uint64_t(std::clamp(int(v / kParcel) + 64, 0, 1023)); };
        return (std::uint64_t(o.container_index + 1) << 30) | (q(o.position.x) << 20) | (q(o.position.y) << 10) | q(o.position.z);
    };
    for (const auto& ptr : objects) {
        const PhysicsObject& o = *ptr;
        if (!o.is_gas || !is_valid(o.position)) continue;
        Parcel& parcel = parcels[key(o)];
        parcel.energy += o.thermal_mass() * o.temperature;
        parcel.capacity += o.thermal_mass();
    }
    if (parcels.empty()) return;
    for (const auto& ptr : objects) {
        PhysicsObject& o = *ptr;
        if (!o.is_gas || !is_valid(o.position)) continue;
        const Parcel& parcel = parcels[key(o)];
        if (parcel.capacity <= o.thermal_mass()) continue;      // alone in its parcel
        const double mean = parcel.energy / parcel.capacity;
        o.add_heat((mean - o.temperature) * 0.25 * o.thermal_mass());
    }
}

// Freezing water bonds to neighbouring ice, so a freezing pool becomes one
// rigid floe (ice expands ~3 % in each direction as it freezes).  Its
// buoyancy comes from update_ice_waterlines().  Returns true if any particle
// changed phase this frame.
bool World::process_phase_events() {
    bool any = false, any_froze = false, any_melted = false;
    for (const auto& ptr : objects) {
        if (ptr->phase_event == PhaseEvent::None) continue;
        any = true;
        if (ptr->phase_event == PhaseEvent::Froze) any_froze = true;
        else if (ptr->phase_event == PhaseEvent::Melted) any_melted = true;
    }
    if (!any) return false;
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
        auto try_bond = [&](PhysicsObject* a, PhysicsObject* b) {
            if (!a->is_ice || !b->is_ice) return;
            if (a->phase_event != PhaseEvent::Froze && b->phase_event != PhaseEvent::Froze) return;
            if (a->container_index != b->container_index) return;
            const double d = (a->position - b->position).length();
            if (d > kIceBondReach * std::max(a->radius, b->radius)) return;
            if (bonds[a] >= kMaxIceBonds || bonds[b] >= kMaxIceBonds) return;
            const double length = std::max(d, 0.62 * (a->radius + b->radius)) * kIceExpansion;
            rigid_links.emplace_back(a, b, length, true);
            ++bonds[a];
            ++bonds[b];
        };
        // New ice next to old ice comes from the broad phase; two particles
        // that froze together were both liquid when it was built, so those
        // pairs come from the liquid's neighbour list.
        for (const auto& [i, j] : hash_.pairs()) try_bond(objects[i].get(), objects[j].get());
        for (std::size_t p = 0; p < pair_i_.size(); ++p) try_bond(fluid_[pair_i_[p]], fluid_[pair_j_[p]]);
    }
    for (const auto& ptr : objects) ptr->phase_event = PhaseEvent::None;
    return true;
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

        const double sigma = v_rms / std::sqrt(3.0);          // per component
        const double kick = sigma * std::sqrt(1.0 - decay * decay);
        o->velocity = o->velocity * decay + Vec3(random_normal(), random_normal(), random_normal()) * kick;

        // Archimedes with ideal-gas densities: rho_gas / rho_air =
        // (M_gas / M_air) · (T_air / T_gas).  Hot or light gas rises, cold
        // or heavy gas sinks; gas at room temperature is neutrally buoyant.
        // Inside a sealed vessel there is no surrounding air.
        if (o->container_index < 0) {
            const double density_ratio = o->molar_ratio * thermo::kelvin(ambient_temperature) / thermo::kelvin(o->temperature);
            o->velocity += gravity * (density_ratio - 1.0);
        }
    }
}

// A blast is a pressure pulse: the impulse a body receives is proportional
// to the cross-section it presents, so light, large things fly furthest.
void World::detonate_bomb(PhysicsObject* bomb) {
    for (auto& ptr : objects) {
        PhysicsObject* target = ptr.get();
        if (target == bomb || target->is_static || target->is_dragged) continue;
        const Vec3 delta = target->position - bomb->position;
        const double distance = delta.length();
        if (distance <= 0.0 || distance >= kBlastRadius) continue;
        const double falloff = 1.0 - distance / kBlastRadius;
        double kick = kBlastImpulse * target->cross_section() * falloff * falloff / target->mass;
        if (target->is_gas) {
            // A blast drives air at a few times its thermal speed (a strong
            // shock), not at the 150 cm/frame cap a ball can reach.
            const double v_rms = thermo::kGasSpeedScale
                * std::sqrt(thermo::kelvin(target->temperature) / target->molar_ratio);
            kick = std::min(kick, 4.0 * v_rms * falloff);
        }
        target->velocity += delta * (kick / distance);
        const double speed = target->velocity.length();
        if (speed > kMaxSpeed) target->velocity *= kMaxSpeed / speed;
    }
}
