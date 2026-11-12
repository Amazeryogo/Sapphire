#pragma once
// Joints between bodies.  Springs and gas struts are force elements (applied
// once per substep before integration); rigid links, pulley ropes and strut
// end stops are position constraints solved by the PBD solver.  Springs and
// links can attach at any point of a body (an anchor in its own frame), in
// which case they also turn it.

#include "physics_object.h"

#include <vector>

class Spring {
public:
    PhysicsObject* obj_a;
    PhysicsObject* obj_b;
    Vec3 anchor_a, anchor_b;   // body frame
    double length;
    double stiffness;   // force per cm of stretch
    double damping;     // force per cm/frame of closing speed

    Spring(PhysicsObject* a, PhysicsObject* b, double l, double s, double d,
           const Vec3& anchor_on_a = Vec3(), const Vec3& anchor_on_b = Vec3())
        : obj_a(a), obj_b(b), anchor_a(anchor_on_a), anchor_b(anchor_on_b), length(l), stiffness(s), damping(d) {}

    Vec3 end_a() const { return obj_a->position + obj_a->orientation.rotate(anchor_a); }
    Vec3 end_b() const { return obj_b->position + obj_b->orientation.rotate(anchor_b); }
    void apply_forces() const;
};

class RigidLink {
public:
    PhysicsObject* obj_a;
    PhysicsObject* obj_b;
    Vec3 anchor_a, anchor_b;   // body frame
    double length;
    bool ice_bond;      // created by freezing; removed on melting or fracture
    bool anchored;      // false when both anchors are the centres (ropes, ice)

    RigidLink(PhysicsObject* a, PhysicsObject* b, double l, bool ice = false,
              const Vec3& anchor_on_a = Vec3(), const Vec3& anchor_on_b = Vec3())
        : obj_a(a), obj_b(b), anchor_a(anchor_on_a), anchor_b(anchor_on_b), length(l), ice_bond(ice),
          anchored(anchor_on_a.length_sq() > 0.0 || anchor_on_b.length_sq() > 0.0) {}

    Vec3 end_a() const { return anchored ? obj_a->position + obj_a->orientation.rotate(anchor_a) : obj_a->position; }
    Vec3 end_b() const { return anchored ? obj_b->position + obj_b->orientation.rotate(anchor_b) : obj_b->position; }
    void solve() const;
};

// An inextensible rope over a fixed pulley: |a - anchor| + |b - anchor| <= L.
class Pulley {
public:
    PhysicsObject* obj_a;
    PhysicsObject* obj_b;
    Vec3 anchor;
    double total_length;

    Pulley(PhysicsObject* a, PhysicsObject* b, const Vec3& anc)
        : obj_a(a), obj_b(b), anchor(anc),
          total_length((a->position - anc).length() + (b->position - anc).length()) {}
    Pulley(PhysicsObject* a, PhysicsObject* b, const Vec3& anc, double total)
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

// A wheel bearing: the wheel may only spin, relative to its frame, about an
// axis fixed in the frame.  A motorised wheel is driven about that axis and
// the frame feels the reaction torque.
class Axle {
public:
    PhysicsObject* wheel;
    PhysicsObject* frame;
    Vec3 local_axis;    // in the frame's body space

    Axle(PhysicsObject* w, PhysicsObject* f, const Vec3& axis) : wheel(w), frame(f), local_axis(axis) {}

    Vec3 world_axis() const { return frame->orientation.rotate(local_axis).normalized(Vec3(0, 0, -1)); }
    // Once per substep, before integration: motor torque (limited to 'max_step' rad/frame of change).
    void drive(double max_step) const;
    // Once per substep, after the contacts: removes relative spin off the axis.
    void solve_velocity() const;
};

// A rigid body made of beads (a drawn shape).  Every substep the beads are
// snapped to the rigid pose that fits them best (shape matching, Müller et
// al. 2005), which conserves linear and angular momentum and, unlike a truss
// of distance links, is exactly rigid however many beads there are.
class RigidCluster {
public:
    std::vector<PhysicsObject*> nodes;
    std::vector<Vec3> rest;     // bead positions in the shape's own frame
    Quat rotation;              // best-fit rotation so far (warm start)

    RigidCluster() = default;
    explicit RigidCluster(const std::vector<PhysicsObject*>& beads);

    void solve();
};
