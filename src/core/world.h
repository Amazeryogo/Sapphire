#pragma once
// The simulation: a 3D room (x right, y up, z towards the default camera)
// measured in centimetres, kilograms and frames.  World::step() advances
// exactly one frame (1/60 s) and is independent of how often the UI calls it,
// so results do not depend on the display's frame rate.
//
// Mechanics use substepped position-based dynamics (Müller et al., "Detailed
// Rigid Body Simulation with Extended Position Based Dynamics", 2020):
//   1. apply forces and integrate to predicted poses,
//   2. project constraints (rigid links, ropes, contacts with static
//      friction, the liquid's incompressibility, walls),
//   3. derive linear and angular velocities from the change of pose,
//   4. a velocity pass applies restitution, dynamic and rolling friction.
// Because velocities are derived from positions, constraints can never hide
// momentum the way a pure position projection on top of an Euler integrator
// does.
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
    PhysicsObject* b;         // nullptr for a wall of the room
    Vec3 normal;              // unit, points from b (or the wall) towards a
    Vec3 ra, rb;              // contact point relative to each centre
    bool b_fixed;             // b takes no impulse (one-sided contact)
    double vn_pre;            // normal relative velocity before the solve
    double restitution;
    double friction;
    double lambda;            // mass-weighted normal correction (impulse · h)
    double friction_used;     // friction already applied at position level (impulse · h)
};

struct EnergyReport {
    double kinetic = 0.0;     // kg·cm²/frame² (× 0.36 = joules)
    double potential = 0.0;   // gravitational, relative to the floor
};

class World {
public:
    // ---- Solver settings ----
    static constexpr int kSubsteps = 6;
    static constexpr int kLinkIterations = 2;
    static constexpr double kMaxSpeed = 150.0;          // cm / frame
    static constexpr double kMaxSpin = 3.0;             // rad / frame
    // Broad-phase slack (cm): per frame it must cover heat conduction and the
    // immersion ring; per substep only contacts.
    static constexpr double kFramePairMargin = 16.0;
    static constexpr double kSubstepPairMargin = 4.0;

    // ---- User parameters (per-frame units) ----
    Vec3 gravity = Vec3(0.0, -0.2725, 0.0);   // 9.81 m/s²
    double air_density_scale = 1.0;      // 1 = real air (1.2 kg/m³)
    double elasticity = 0.8;             // rubber's bounce at 20 °C
    double ambient_temperature = 20.0;   // room air, a finite thermal reservoir
    double ambient_setpoint = 20.0;      // what the building/HVAC pulls the room toward
    double heat_transfer_rate = 0.05;
    double fluid_viscosity = 0.05;
    double width = 900.0;                // room size, cm
    double height = 600.0;
    double depth = 500.0;
    Material spawn_material = Material::Rubber;   // what balls, boxes and wheels are made of

    // ---- Thermal tuning (sim units, see thermo namespace) ----
    static constexpr double kAirHeatCapacity      = 30000.0; // room air thermal mass
    static constexpr double kRoomLeak             = 40.0;    // room ↔ building conductance
    static constexpr double kSourceAirConductance = 1.2;     // heater/cooler → room air
    static constexpr double kSourceObjectConductance = 7.6e-5; // per cm² of target surface
    static constexpr double kSourceReach          = 70.0;    // cm from the plate's surface
    static constexpr double kConvection           = 1.4e-5;  // per cm² of surface, per K
    static constexpr double kRadiation            = 2.0e-14; // per cm² of surface, per K⁴
    static constexpr double kEvaporation          = 0.3;     // per surface particle at P_sat = 1 atm
    static constexpr double kConductionReach      = 15.0;    // cm between surfaces
    // ---- Gas vessel tuning ----
    static constexpr double kGasPressureConstant  = 5000.0;  // kPa·cm³ per particle per K
    static constexpr double kPistonForceScale     = 2.0e-4;  // kPa·cm² → sim force
    static constexpr double kPistonDamping        = 0.08;
    static constexpr double kPistonMass           = 80.0;
    static constexpr int    kVesselAirParticles   = 220;
    // ---- Aerodynamics ----
    static constexpr double kAirDensity           = 1.2e-6;  // kg / cm³
    // ---- Liquid water: Position Based Fluids (Macklin & Müller 2013) ----
    // The pressure term of the incompressible Navier–Stokes equations is an
    // incompressibility constraint (rho_i <= rho_0) projected every substep,
    // viscosity is XSPH smoothing, gravity and thermal buoyancy are external
    // forces and surface tension is a cohesion force (Akinci et al. 2013).
    static constexpr double kFluidSpacing         = 10.0;    // rest spacing of water particles (cm)
    static constexpr double kFluidH               = 21.0;    // smoothing length
    static constexpr double kFluidRelaxation      = 0.05;    // constraint-force mixing
    static constexpr double kFluidAntiCluster     = 0.01;    // artificial pressure
    static constexpr double kFluidCohesion        = 0.004;   // surface tension at 20 °C
    static constexpr double kWaterContactRadius   = 5.0;     // cm, water seen by solids
    static constexpr double kWaterParticleMass    = PhysicsObject::kWaterDensity * 1000.0;   // spacing³
    static constexpr double kHydroLinearDrag      = 1.3e-3;  // viscous drag per cm² of cross-section (force per cm/frame)
    static constexpr double kHydroFormDrag        = 1.0;     // C_d of a submerged body (form + wave drag)
    static constexpr double kIceFormDrag          = 0.04;    // per (cm/frame) per frame, fully submerged
    static constexpr double kWaterLevelSmoothing  = 0.05;    // per frame
    static constexpr double kIceBondReach         = 3.0;     // × particle radius
    static constexpr double kIceExpansion         = 1.03;    // bond length / liquid spacing
    static constexpr double kIceRelativeDensity   = 0.917;   // ice / water
    static constexpr double kIceDrag              = 0.1;     // per frame, fully submerged
    static constexpr int    kMaxIceBonds          = 14;
    // ---- Rolling, motors, blasts ----
    static constexpr double kRollingResistance    = 0.012;   // dimensionless (rubber on a hard floor)
    static constexpr double kMotorAcceleration    = 0.04;    // rad / frame²
    static constexpr double kBlastRadius          = 250.0;   // cm
    static constexpr double kBlastImpulse         = 1.6;     // impulse per cm² of cross-section at the centre

    std::vector<std::unique_ptr<PhysicsObject>> objects;
    std::vector<Spring> springs;
    std::vector<RigidLink> rigid_links;
    std::vector<Pulley> pulleys;
    std::vector<PistonLink> pistons;
    std::vector<Axle> axles;
    std::vector<RigidCluster> clusters;
    std::vector<GasContainer> containers;

    // Advance one frame.
    void step();

    void clear();
    // Removes a body and everything attached to it.  Removing a wall of a tank
    // or vessel (or a vessel's piston), or a bead of a drawn shape, removes
    // the whole structure.
    void remove_object(PhysicsObject* obj);
    void set_elasticity(double value);

    // First body hit by a ray (unit direction).  Glass walls of tanks and
    // vessels are only returned when nothing else is hit, so their contents
    // can be reached.
    enum class Pick {
        Any,      // everything (hover readout)
        Grab,     // what the mouse can hold: no gas, no liquid, no glass walls
        Remove,   // everything but gas
    };
    PhysicsObject* pick(const Vec3& origin, const Vec3& direction, Pick mode = Pick::Any,
                        double* distance = nullptr) const;
    const GasContainer* container_of_piston(const PhysicsObject* piston) const;
    Vec3 clamp_to_room(const Vec3& p, double margin) const;

    EnergyReport energy() const;

    // ---- Spawning ----
    PhysicsObject* add_sphere(const Vec3& p, double radius, Material material);
    PhysicsObject* add_box(const Vec3& p, const Vec3& half_extents, Material material);
    PhysicsObject* add_particle(const Vec3& p, double radius, double mass);
    PhysicsObject* spawn_water_particle(const Vec3& p);
    PhysicsObject* spawn_gas_particle(const Vec3& p);
    PhysicsObject* spawn_ball(const Vec3& p);
    PhysicsObject* spawn_box(const Vec3& p);
    PhysicsObject* spawn_wheel(const Vec3& p);
    PhysicsObject* spawn_pin(const Vec3& p);
    PhysicsObject* spawn_platform(const Vec3& p, double tilt_radians = 0.0);
    PhysicsObject* spawn_motor(const Vec3& p);
    PhysicsObject* spawn_heater(const Vec3& p);
    PhysicsObject* spawn_cooler(const Vec3& p);
    PhysicsObject* spawn_bomb(const Vec3& p);
    void spawn_rope(const Vec3& p);
    void spawn_water(const Vec3& p, int nx = 8, int ny = 8, int nz = 8);
    void spawn_gas_cloud(const Vec3& p);
    void spawn_container(const Vec3& p);
    // Open glass tank standing on the floor under p (inner size in cm).
    void spawn_tank(const Vec3& p, double inner_width = 200.0, double inner_height = 130.0, double inner_depth = 140.0);
    void spawn_car(const Vec3& p);
    // Rigid hoop of beads along an outline (>= 3 points).
    bool spawn_polygon(const std::vector<Vec3>& outline);

    void connect_spring(PhysicsObject* a, PhysicsObject* b);
    void connect_piston(PhysicsObject* a, PhysicsObject* b);
    void connect_pulley(PhysicsObject* a, PhysicsObject* b);

    // ---- Mouse dragging (the held body is kinematic) ----
    bool begin_drag(PhysicsObject* obj);
    // Applied smoothly over the next step, or at once (e.g. while paused).
    void drag_to(const Vec3& target, bool immediate = false);
    void end_drag(const Vec3& throw_velocity);
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
    void mix_gas_temperatures();
    bool process_phase_events();
    void build_water_surface();
    double water_level_beside(int container, double x0, double x1, double z0, double z1) const;
    void measure_immersion();
    void update_ice_waterlines();
    void thermostat_gases();
    void detonate_bomb(PhysicsObject* bomb);
    void break_ice_bonds();

    // Liquid solver (fluid.cpp)
    void init_fluid_constants();
    void build_fluid();
    void solve_fluid_density();
    void fluid_velocity_pass(double h);

    // Per substep
    void compute_forces(double h);
    void apply_hydrostatics(double h);
    void integrate(double h, double drag_fraction);
    void solve_positions();
    void solve_collisions();
    void solve_container_boundaries();
    void solve_room_walls();
    void derive_velocities(double h);
    void solve_velocities(double h);

    // Resolves one contact at position level (normal + static friction) and
    // records it for the velocity pass.  pa/pb are world-space points on the
    // two surfaces; the normal points from b (or the wall) towards a.
    void add_contact(PhysicsObject* a, PhysicsObject* b, const Vec3& normal, const Vec3& pa, const Vec3& pb,
                     double depth, bool b_fixed = false);
    struct Manifold {
        PhysicsObject* a;
        PhysicsObject* b;          // nullptr: a wall of the room, n·x = plane_offset
        Vec3 normal;               // from b (or the wall) towards a
        double plane_offset = 0.0;
        int count = 0;
        Vec3 la[8], lb[8];         // contact points in each body's frame
    };
    void solve_manifold(const Manifold& m);
    struct PatchFriction {
        PhysicsObject* a;
        PhysicsObject* b;
        Vec3 normal;
        Vec3 la, lb;               // centre of the patch in each body's frame
        double load;               // total normal correction (impulse · h)
        double friction;
        double used;               // friction applied so far (impulse · h)
        std::size_t first_contact; // its points in contacts_
        int count;
    };
    void apply_patch_friction();
    void collide_pair(PhysicsObject* a, PhysicsObject* b);
    double contact_radius(const PhysicsObject& self, const PhysicsObject& other) const;
    void add_walls(const Vec3& inner_min, const Vec3& inner_max, int group);

    static double limited_coupling(double coupling, const PhysicsObject& a, const PhysicsObject& b);
    static double contact_conductance(const PhysicsObject& a, const PhysicsObject& b);
    static double collision_restitution(const PhysicsObject& a, const PhysicsObject& b);

    SpatialHash hash_;
    double h_ = 1.0 / kSubsteps;      // current substep length (frames)
    struct Immersion {
        double level = 0.0;           // water level beside the body (y), NaN if none
        int count = 0;                // water particles in the body's ring
        Vec3 water_velocity;          // their summed velocity
        double viscosity = 0.0;       // their summed viscosity ratio
        Vec3 reaction;                // drag reaction per ring particle (momentum)
    };
    std::vector<Immersion> immersion_;
    static constexpr double kSurfaceColumn = 10.0;
    int surface_cols_ = 1, surface_rows_ = 1;
    std::vector<std::vector<double>> surface_tops_;   // [vessel + 1][row · cols + col] highest water y
    std::vector<double> ice_submersion_;   // per object, from its floe's waterline (per frame)
    std::vector<std::pair<std::uint32_t, std::uint32_t>> immersed_pairs_;   // (body, water)
    std::vector<Contact> contacts_;
    std::vector<PatchFriction> patches_;
    struct Slam {
        PhysicsObject* water;
        PhysicsObject* body;
        Vec3 normal;               // from the body towards the water particle
        Vec3 r;                    // contact point relative to the body's centre
        double depth;              // how far the particle was pushed this substep
    };
    std::vector<Slam> slams_;
    PhysicsObject* dragged_ = nullptr;
    Vec3 drag_start_;
    Vec3 drag_target_;
    int next_group_ = 1;

    // ---- Liquid solver state (rebuilt every frame) ----
    double fluid_rest_density_ = 0.0;     // kernel density of the rest lattice at 20 °C
    double fluid_gradient_norm_ = 0.0;    // Σ|∇C|² of the rest lattice
    std::vector<PhysicsObject*> fluid_;
    std::vector<std::uint32_t> fluid_index_;          // index into objects
    std::vector<Vec3> f_pos_, f_delta_, f_gsum_, f_vel_, f_visc_, f_coh_;
    std::vector<double> f_rho_, f_g2_, f_lambda_, f_rest_, f_viscosity_, f_wsum_, f_tsum_;
    std::vector<std::uint32_t> pair_i_, pair_j_;      // neighbour pairs (i < j), within h + skin
    std::vector<double> pair_w_;
    std::vector<Vec3> pair_g_;
};
