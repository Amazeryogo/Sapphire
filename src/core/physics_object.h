#pragma once
// A single simulated body: rigid ball/box, water/ice/steam particle or gas
// particle.  Holds both mechanical state (positions, velocities, rotation)
// and thermal state (temperature, phase, latent heat).  No Qt dependency.

#include "thermo.h"
#include "vector.h"

#include <cstdint>
#include <vector>

enum class ShapeType : int { CIRCLE = 0, SQUARE = 1 };

struct Rgba {
    int r = 255, g = 255, b = 255, a = 255;
};

enum class PhaseEvent : int { None, Froze, Melted, Boiled, Condensed };

class PhysicsObject {
public:
    // ---- Mechanics ----
    Vector position;
    Vector previous_position;   // start of the current substep (PBD)
    Vector velocity;            // px / frame
    Vector pre_velocity;        // velocity before the position solve (for restitution)
    Vector acceleration;        // accumulated per substep, px / frame²
    double angle = 0.0;              // rad
    double angular_velocity = 0.0;   // rad / frame (positive = clockwise on screen)
    double motor_speed = 0.0;        // target angular velocity when is_motor

    double radius;
    double liquid_radius;    // radius to restore when steam/ice turns back into water
    double elasticity;       // restitution at 20 °C (from the Bounciness slider)
    double mass;
    Rgba color;
    ShapeType shape_type;

    bool is_static;
    bool is_fluid;
    bool is_gas = false;
    bool is_dragged = false;
    bool is_motor = false;
    bool is_bouncy = true;
    bool is_heater = false;
    bool is_cooler = false;
    bool is_steam = false;
    bool is_ice = false;
    bool is_container_piston = false;

    // ---- Thermodynamics ----
    double temperature = 20.0;
    double latent_energy = 0.0;     // progress of an unfinished phase change
    double molar_ratio = 1.0;       // molar mass relative to air (gases)
    double local_pressure = thermo::kAtmosphere;
    int container_index = -1;       // sealed vessel this object is inside, if any
    double water_level = 0.0;       // smoothed water level beside a floating body (y)
    bool has_water_level = false;
    PhaseEvent phase_event = PhaseEvent::None;  // set by add_heat, consumed by World

    int fuse = -1;                  // frames until a bomb detonates
    int group = 0;                  // bodies sharing a non-zero group never collide

    std::vector<Vector> trail;

    PhysicsObject(double x, double y, double r = 18.0, double el = 0.8, double m = -1.0,
                  Rgba c = Rgba{-1, -1, -1, -1}, ShapeType st = ShapeType::CIRCLE,
                  bool is_stat = false, bool is_fl = false);

    // Inverse mass seen by the solver; static and mouse-held bodies are immovable.
    double inv_mass() const { return (is_static || is_dragged) ? 0.0 : 1.0 / mass; }
    bool can_rotate() const {
        return shape_type == ShapeType::CIRCLE && !is_gas && !is_fluid && !is_ice
            && !is_static && !is_container_piston;
    }
    // Solid disc: I = m r² / 2.
    double inv_inertia() const {
        return (can_rotate() && !is_dragged) ? 2.0 / (mass * radius * radius) : 0.0;
    }
    // Coulomb friction coefficient of the surface.
    double friction() const {
        if (is_gas) return 0.0;
        if (is_fluid) return 0.0;
        if (is_ice) return 0.03;
        return 0.6;
    }

    void apply_force(const Vector& force) { acceleration += force * inv_mass(); }

    // Specific heat capacity, kJ/(kg·K).  Gases use c_v because the sandbox's
    // vessels are closed; free gas exchanges with the room at ~constant volume
    // per frame as well.
    double specific_heat() const {
        if (is_ice) return 2.09;
        if (is_fluid) return 4.18;
        if (is_steam) return 1.41;
        if (is_gas) return 0.718;
        return 0.9;
    }
    double thermal_mass() const { return std::max(0.05, mass * specific_heat()); }
    double heat_capacity_ratio() const { return is_steam ? 1.33 : 1.40; }

    // Relative thermal conductivity used for contact conduction.
    double conductivity() const {
        if (is_heater || is_cooler) return 20.0;  // metal plate
        if (is_ice) return 3.7;                   // 2.2 W/mK vs water 0.6
        if (is_fluid) return 1.0;
        if (is_gas) return 0.04;                  // 0.025 W/mK
        return 0.5;
    }

    bool is_water_substance() const { return is_fluid || is_ice || is_steam; }
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

    void update_trail();
};
