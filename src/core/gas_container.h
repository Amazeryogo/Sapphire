#pragma once
// A sealed vertical cylinder with analytic walls and a massive, vertically
// guided piston.  Wall collisions are swept against each object's previous
// side, so nothing can leak in or out, and the space above the piston is open
// to the room (objects can rest on the piston and weigh it down).

#include "physics_object.h"
#include "thermo.h"

class GasContainer {
public:
    double left, right, bottom, wall_top;
    double piston_min_y, piston_max_y;
    PhysicsObject* piston = nullptr;
    double pressure = thermo::kAtmosphere;
    double average_temperature = 20.0;
    double gas_volume = 0.0;              // px² (the sandbox is 2D)
    double previous_chamber_height = -1.0;
    double boiling_point = 100.0;
    int gas_particles = 0;
    int liquid_particles = 0;

    static constexpr double kWallHalfThickness = 3.0;

    GasContainer(double x, double y, double half_width = 110.0, double half_height = 90.0)
        : left(x - half_width), right(x + half_width), bottom(y + half_height),
          wall_top(y - half_height - 12.0),
          piston_min_y(y - half_height), piston_max_y(y + half_height - 30.0) {}

    GasContainer(double l, double r, double b, double wt, double pmin, double pmax)
        : left(l), right(r), bottom(b), wall_top(wt), piston_min_y(pmin), piston_max_y(pmax) {}

    double top() const { return piston ? piston->position.y : piston_min_y; }
    double width() const { return right - left; }
    double chamber_height() const { return std::max(1.0, bottom - top()); }
    bool in_chamber(const Vector& p) const {
        return p.x > left && p.x < right && p.y > top() - 0.5 && p.y < bottom;
    }
};
