#pragma once
// Joints between bodies.  Springs and gas struts are force elements (applied
// once per substep before integration); rigid links, pulley ropes and strut
// end stops are position constraints solved by the PBD solver.

#include "physics_object.h"

class Spring {
public:
    PhysicsObject* obj_a;
    PhysicsObject* obj_b;
    double length;
    double stiffness;   // force per px of stretch
    double damping;     // force per px/frame of closing speed

    Spring(PhysicsObject* a, PhysicsObject* b, double l, double s = 0.3, double d = 0.1)
        : obj_a(a), obj_b(b), length(l), stiffness(s), damping(d) {}

    void apply_forces() const;
};

class RigidLink {
public:
    PhysicsObject* obj_a;
    PhysicsObject* obj_b;
    double length;
    bool ice_bond;      // created by freezing; removed on melting or fracture

    RigidLink(PhysicsObject* a, PhysicsObject* b, double l, bool ice = false)
        : obj_a(a), obj_b(b), length(l), ice_bond(ice) {}

    void solve() const;
};

// An inextensible rope over a fixed pulley: |a - anchor| + |b - anchor| <= L.
class Pulley {
public:
    PhysicsObject* obj_a;
    PhysicsObject* obj_b;
    Vector anchor;
    double total_length;

    Pulley(PhysicsObject* a, PhysicsObject* b, Vector anc)
        : obj_a(a), obj_b(b), anchor(anc),
          total_length((a->position - anc).length() + (b->position - anc).length()) {}
    Pulley(PhysicsObject* a, PhysicsObject* b, Vector anc, double total)
        : obj_a(a), obj_b(b), anchor(anc), total_length(total) {}

    void solve() const;
};

// A pneumatic cylinder (gas strut).  A sealed air charge balances atmospheric
// pressure at the rest length.  Its force follows the ideal-gas law
// P ∝ T / L, the charge heats on fast compression and cools on expansion
// (adiabatic, γ = 1.4) and relaxes towards its surroundings' temperature, so
// a hot piston extends and a cold one retracts.  A dashpot damps motion and
// the rod has hard end stops.  Forces are mass-weighted so momentum is
// conserved and light objects are not flung out.
class PistonLink {
public:
    PhysicsObject* obj_a;
    PhysicsObject* obj_b;
    double min_len;
    double max_len;
    double rest_length;
    double charge_temperature;  // °C at which the charge balances at rest_length
    double gas_temperature;     // current °C of the charge
    double stiffness;
    double damping;
    double previous_length;

    PistonLink(PhysicsObject* a, PhysicsObject* b, double min_l, double max_l, double rest,
               double temperature, double s = 0.12, double d = 0.10)
        : obj_a(a), obj_b(b), min_len(min_l), max_len(max_l), rest_length(rest),
          charge_temperature(temperature), gas_temperature(temperature),
          stiffness(s), damping(d), previous_length(rest) {}

    double gauge_pressure_ratio(double length) const {
        return thermo::kelvin(gas_temperature) / thermo::kelvin(charge_temperature)
             * rest_length / std::max(1.0, length) - 1.0;
    }

    // Once per frame: adiabatic heating/cooling and heat exchange of the charge.
    void update_gas(double ambient_temperature);
    // Once per substep.
    void apply_forces() const;
    void solve_stops() const;
};
