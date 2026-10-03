#pragma once
// The simulation.  World::step() advances exactly one frame (1/60 s) and is
// independent of how often the UI calls it, so results do not depend on the
// display's frame rate.
//
// Mechanics use substepped position-based dynamics (Müller et al., "Detailed
// Rigid Body Simulation with Extended Position Based Dynamics", 2020):
//   1. apply forces and integrate to predicted positions,
//   2. project constraints (rigid links, ropes, contacts, walls),
//   3. derive velocities from the position change,
//   4. a velocity pass applies restitution and Coulomb friction.
// Because velocities are derived from positions, constraints can never hide
// momentum the way a pure position projection on top of an Euler integrator
// does (the old cause of stretching ropes).
//
// Thermal processes (conduction, convection, radiation, phase changes, gas
// thermostat) run once per frame.  No Qt dependency.

#include "constraints.h"
#include "gas_container.h"
#include "physics_object.h"
#include "spatial_hash.h"

#include <memory>
#include <string>
#include <vector>

struct Contact {
    PhysicsObject* a;
    PhysicsObject* b;         // nullptr for a wall
    Vector normal;            // unit, points from b (or the wall) towards a
    Vector wall_velocity;     // used when b is nullptr
    double ra, rb;            // contact-point lever arms along the normal
    double wa, wb, wia, wib;  // inverse masses / inertias at creation
    double vn_pre;            // normal relative velocity before the solve
    double restitution;
    double friction;
    double lambda;            // mass-weighted normal correction (impulse · h)
};

class World {
public:
    // ---- Solver settings ----
    static constexpr int kSubsteps = 6;
    static constexpr int kLinkIterations = 2;
    static constexpr double kMaxSpeed = 150.0;          // px / frame
    // Broad-phase slack (px): per frame it must cover heat conduction
    // (r1 + r2 + 15); per substep only contacts and the water kernel (2.5 r).
    static constexpr double kFramePairMargin = 16.0;
    static constexpr double kSubstepPairMargin = 4.0;

    // ---- User parameters (per-frame units, as in the UI) ----
    Vector gravity = Vector(0.0, 0.3);
    double air_resistance = 0.01;
    double elasticity = 0.8;
    double ambient_temperature = 20.0;  // room air, a finite thermal reservoir
    double ambient_setpoint = 20.0;     // what the building/HVAC pulls the room toward
    double heat_transfer_rate = 0.05;
    double fluid_viscosity = 0.05;
    double width = 1000.0;
    double height = 800.0;

    // ---- Thermal tuning (sim units, see thermo namespace) ----
    static constexpr double kAirHeatCapacity      = 1500.0; // room air thermal mass
    static constexpr double kRoomLeak             = 2.0;    // room ↔ building conductance
    static constexpr double kSourceAirConductance = 0.06;   // heater/cooler → room air
    static constexpr double kSourceObjectConductance = 0.004; // per px of target radius
    static constexpr double kConvection           = 0.00075; // per px of radius, per K
    static constexpr double kRadiation            = 1.05e-12; // per px of radius, per K⁴
    static constexpr double kEvaporation          = 0.05;   // per px of radius at P_sat = 1 atm
    // ---- Gas vessel tuning ----
    static constexpr double kGasPressureConstant  = 40.0;   // kPa·px² per particle per K
    static constexpr double kPistonForceScale     = 0.0073; // kPa·px → sim force
    static constexpr double kPistonDamping        = 0.08;
    // ---- Water ----
    static constexpr double kWaterContactRadius   = 4.5;    // px, water seen by solids
    // Reference density of water for buoyancy (mass / px²): a body this
    // dense neither floats nor sinks.  Defaults: rubber ball ≈ 0.22 of it,
    // bomb ≈ 2×, ice 0.917×.
    static constexpr double kWaterDensity         = 0.0142;
    static constexpr double kHydroDrag            = 0.15;   // per frame, fully submerged
    static constexpr double kHydroQuadraticDrag   = 0.04;   // per (px/frame) per frame, fully submerged
    static constexpr double kWaterLevelSmoothing  = 0.05;   // per frame
    static constexpr double kIceBondReach         = 2.4;    // × particle radius
    static constexpr double kIceExpansion         = 1.03;   // bond length / liquid spacing
    static constexpr double kIceRelativeDensity   = 0.917;  // ice / water
    static constexpr double kIceDrag              = 0.1;    // per frame, fully submerged
    static constexpr int    kMaxIceBonds          = 6;
    // ---- Motors ----
    static constexpr double kMotorAcceleration    = 0.04;   // rad / frame²

    std::vector<std::unique_ptr<PhysicsObject>> objects;
    std::vector<Spring> springs;
    std::vector<RigidLink> rigid_links;
    std::vector<Pulley> pulleys;
    std::vector<PistonLink> pistons;
    std::vector<GasContainer> containers;

    // Advance one frame.
    void step();

    void clear();
    void remove_object(PhysicsObject* obj);
    void set_elasticity(double value);

    // Topmost object under a point (vessel piston bars included).
    PhysicsObject* object_at(const Vector& p) const;
    const GasContainer* container_of_piston(const PhysicsObject* piston) const;

    // ---- Spawning ----
    PhysicsObject* spawn_at(double x, double y, ShapeType shape, double radius = 18.0, double mass = -1.0,
                            bool is_static = false, bool is_fluid = false);
    PhysicsObject* spawn_gas_particle(double x, double y);
    PhysicsObject* spawn_ball(double x, double y);
    PhysicsObject* spawn_box(double x, double y);
    PhysicsObject* spawn_wheel(double x, double y);
    PhysicsObject* spawn_pin(double x, double y);
    PhysicsObject* spawn_motor(double x, double y);
    PhysicsObject* spawn_heater(double x, double y);
    PhysicsObject* spawn_cooler(double x, double y);
    PhysicsObject* spawn_bomb(double x, double y);
    void spawn_rope(double x, double y);
    void spawn_water(double x, double y);
    void spawn_gas_cloud(double x, double y);
    void spawn_container(double x, double y);
    void spawn_car(double x, double y);
    // Rigid shape from an outline (>= 3 points).
    bool spawn_polygon(const std::vector<Vector>& outline);

    void connect_spring(PhysicsObject* a, PhysicsObject* b);
    void connect_piston(PhysicsObject* a, PhysicsObject* b);
    void connect_pulley(PhysicsObject* a, PhysicsObject* b);

    // ---- Mouse dragging (the held body is kinematic) ----
    void begin_drag(PhysicsObject* obj);
    // Applied smoothly over the next step, or at once (e.g. while paused).
    void drag_to(const Vector& target, bool immediate = false);
    void end_drag(const Vector& throw_velocity);
    PhysicsObject* dragged() const { return dragged_; }

    // ---- Persistence (also used for undo) ----
    std::string serialize() const;
    bool deserialize(const std::string& text, std::string* error = nullptr);

    // Index of an object, or -1.
    int index_of(const PhysicsObject* obj) const;

private:
    // Per frame
    void update_containers();
    void update_thermal_environment();
    void conduct_heat();
    void process_phase_events();
    void build_water_surface();
    double water_level_beside(int container, double x_left, double x_right) const;
    void measure_immersion();
    void update_ice_waterlines();
    void thermostat_gases();
    void detonate_bomb(PhysicsObject* bomb);
    void break_ice_bonds();

    // Per substep
    void compute_forces(double h);
    void apply_hydrostatics(double h);
    void integrate(double h, double drag_fraction);
    void solve_positions();
    void solve_collisions();
    void solve_container_boundaries();
    void solve_screen_walls();
    void derive_velocities(double h);
    void solve_velocities(double h);

    void add_contact(PhysicsObject* a, PhysicsObject* b, const Vector& normal, double depth,
                     double ra, double rb, const Vector& wall_velocity, double wb_override = -1.0);
    double contact_radius(const PhysicsObject& self, const PhysicsObject& other) const;

    static double limited_coupling(double coupling, const PhysicsObject& a, const PhysicsObject& b);
    static double contact_conductance(const PhysicsObject& a, const PhysicsObject& b);
    static double collision_restitution(const PhysicsObject& a, const PhysicsObject& b);

    SpatialHash hash_;
    std::vector<double> expansion_;   // per-object water thermal expansion, per frame
    std::vector<double> viscosity_;   // per-object water viscosity ratio, per frame
    struct Immersion {
        double level = 0.0;           // water level beside the body (y), NaN if none
        int count = 0;                // water particles in the body's ring
        Vector water_velocity;        // their summed velocity
        double viscosity = 0.0;       // their summed viscosity ratio
        Vector reaction;              // drag reaction per ring particle (momentum)
    };
    std::vector<Immersion> immersion_;
    static constexpr double kSurfaceColumn = 8.0;
    std::vector<std::vector<double>> surface_tops_;   // [vessel + 1][column] highest water y
    std::vector<double> ice_submersion_;   // per object, from its floe's waterline (per frame)
    std::vector<std::pair<std::uint32_t, std::uint32_t>> immersed_pairs_;   // (body, water)
    std::vector<Contact> contacts_;
    PhysicsObject* dragged_ = nullptr;
    Vector drag_start_;
    Vector drag_target_;
    int next_group_ = 1;
};
