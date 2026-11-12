#pragma once
// A single simulated body: a rigid sphere or box, a water/ice/steam particle
// or a gas particle.  Holds both mechanical state (pose, velocities) and
// thermal state (temperature, phase, latent heat).  No Qt dependency.

#include "material.h"
#include "thermo.h"
#include "vector.h"

#include <cstdint>

enum class ShapeType : int { SPHERE = 0, BOX = 1 };

struct Rgba {
    int r = 255, g = 255, b = 255, a = 255;
};

enum class PhaseEvent : int { None, Froze, Melted, Boiled, Condensed };

class PhysicsObject {
public:
    // Mass of one cm³ of water, kg.  Every solid's mass is its material's
    // relative density times this times its volume.
    static constexpr double kWaterDensity = 0.001;

    // ---- Mechanics ----
    Vec3 position;
    Vec3 previous_position;       // start of the current substep (PBD)
    Vec3 velocity;                // cm / frame
    Vec3 pre_velocity;            // velocity before the position solve (for restitution)
    Vec3 acceleration;            // accumulated per substep, cm / frame²
    Quat orientation;
    Quat previous_orientation;
    Vec3 angular_velocity;        // rad / frame, world space
    Vec3 pre_angular_velocity;
    Vec3 angular_acceleration;    // accumulated per substep
    double motor_speed = 0.0;     // target spin about motor_axis when is_motor
    Vec3 motor_axis = Vec3(0.0, 0.0, -1.0);   // world space; -z drives towards +x

    ShapeType shape_type = ShapeType::SPHERE;
    double radius = 18.0;         // sphere radius, or the bounding radius of a box
    Vec3 half_extents;            // boxes only
    double liquid_radius = 18.0;  // radius to restore when steam/ice turns back into water
    Material material = Material::Rubber;
    double elasticity = 0.8;      // restitution at 20 °C
    double mass = 1.0;
    Vec3 inv_inertia_body;        // diagonal, body frame
    Rgba color;

    bool is_static = false;
    bool is_fluid = false;
    bool is_gas = false;
    bool is_dragged = false;
    bool is_motor = false;
    bool is_bouncy = false;       // rubber: follows the Bounciness slider and its temperature
    bool is_heater = false;
    bool is_cooler = false;
    bool is_steam = false;
    bool is_ice = false;
    bool is_container_piston = false;
    bool is_fixture = false;      // wall of a tank or vessel: removed together with its group
    bool in_cluster = false;      // bead of a drawn shape: the shape turns, not the bead
    bool on_axle = false;         // transient: driven through an Axle this substep

    // ---- Thermodynamics ----
    double temperature = 20.0;
    double latent_energy = 0.0;     // progress of an unfinished phase change
    double molar_ratio = 1.0;       // molar mass relative to air (gases)
    double local_pressure = thermo::kAtmosphere;
    int container_index = -1;       // sealed vessel this object is inside, if any
    double water_level = 0.0;       // smoothed water level beside a floating body (y)
    bool has_water_level = false;
    double fluid_density = 1.0;     // liquid: density / rest density from the last solve
    PhaseEvent phase_event = PhaseEvent::None;  // set by add_heat, consumed by World

    int fuse = -1;                  // frames until a bomb detonates
    int group = 0;                  // bodies sharing a non-zero group never collide

    explicit PhysicsObject(const Vec3& p = Vec3(), double r = 18.0);

    // ---- Shape ----
    void make_sphere(double r);
    void make_box(const Vec3& half);
    double volume() const;
    double surface_area() const;
    // Mean cross-section seen by a flow (Cauchy: a quarter of the surface for convex bodies).
    double cross_section() const { return 0.25 * surface_area(); }
    // Half-width of the shape along a world direction.
    double support(const Vec3& direction) const;
    // Radius of the sphere of equal volume.
    double equivalent_radius() const { return std::cbrt(volume() * 0.238732414637843); }

    // ---- Mass ----
    void set_material(Material m);          // mass from density, bounce from the table
    void set_mass(double m);                // also updates the inertia tensor
    void update_inertia();

    // Inverse mass seen by the solver; static and mouse-held bodies are immovable.
    double inv_mass() const { return (is_static || is_dragged) ? 0.0 : 1.0 / mass; }
    bool can_rotate() const {
        return !is_gas && !is_fluid && !is_ice && !is_static && !is_container_piston && !in_cluster;
    }
    bool rotates_now() const { return can_rotate() && !is_dragged; }
    // I⁻¹ v in world space.
    Vec3 apply_inv_inertia(const Vec3& v) const {
        if (!rotates_now()) return Vec3();
        if (shape_type == ShapeType::SPHERE) return v * inv_inertia_body.x;
        return orientation.rotate(orientation.inv_rotate(v).scaled(inv_inertia_body));
    }
    // Inverse mass felt by an impulse along n applied at offset r from the centre.
    double generalized_inv_mass(const Vec3& r, const Vec3& n) const {
        double w = inv_mass();
        if (w > 0.0 && rotates_now()) {
            const Vec3 c = r.cross(n);
            w += c.dot(apply_inv_inertia(c));
        }
        return w;
    }
    // Position-level impulse p at offset r (XPBD): moves and turns the body.
    void apply_position_impulse(const Vec3& p, const Vec3& r) {
        position += p * inv_mass();
        if (rotates_now()) orientation.integrate(apply_inv_inertia(r.cross(p)));
    }
    void apply_velocity_impulse(const Vec3& p, const Vec3& r) {
        velocity += p * inv_mass();
        if (rotates_now()) angular_velocity += apply_inv_inertia(r.cross(p));
    }
    Vec3 velocity_at(const Vec3& r) const { return velocity + angular_velocity.cross(r); }
    Vec3 pre_velocity_at(const Vec3& r) const { return pre_velocity + pre_angular_velocity.cross(r); }
    // Force F applied at offset r from the centre.
    void apply_force_at(const Vec3& force, const Vec3& r) {
        acceleration += force * inv_mass();
        if (rotates_now()) angular_acceleration += apply_inv_inertia(r.cross(force));
    }
    void apply_force(const Vec3& force) { acceleration += force * inv_mass(); }

    double kinetic_energy() const;

    // Coulomb friction coefficient of the surface.
    double friction() const {
        if (is_gas || is_fluid) return 0.0;
        if (is_ice) return 0.03;
        return material_props(material).friction;
    }

    // Specific heat capacity, kJ/(kg·K).  Gases use c_v because the sandbox's
    // vessels are closed; free gas exchanges with the room at ~constant volume
    // per frame as well.
    double specific_heat() const {
        if (is_ice) return 2.09;
        if (is_fluid) return 4.18;
        if (is_steam) return 1.41;
        if (is_gas) return 0.718;
        return material_props(material).specific_heat;
    }
    double thermal_mass() const { return std::max(0.05, mass * specific_heat()); }
    double heat_capacity_ratio() const { return is_steam ? 1.33 : 1.40; }

    // Relative thermal conductivity used for contact conduction.
    double conductivity() const {
        if (is_heater || is_cooler) return 20.0;  // metal plate
        if (is_ice) return 3.7;                   // 2.2 W/mK vs water 0.6
        if (is_fluid) return 1.0;
        if (is_gas) return 0.04;                  // 0.025 W/mK
        return material_props(material).conductivity;
    }

    bool is_water_substance() const { return is_fluid || is_ice || is_steam; }
    bool is_particle() const { return is_fluid || is_ice || is_gas; }
    double boiling_point() const { return thermo::boiling_point(local_pressure); }

    // Restitution of this body at its current temperature.  Rigid static
    // bodies return 1 so a collision is governed by the softer partner.
    double effective_restitution() const;

    void become_liquid();
    void become_ice();
    void become_steam();

    // Adds energy (not temperature).  Water-substance objects hold their
    // temperature at a phase boundary while latent heat accumulates, and the
    // boiling point follows the local pressure (Clausius–Clapeyron).
    void add_heat(double energy);
};
