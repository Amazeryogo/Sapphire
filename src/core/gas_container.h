#pragma once
// A sealed upright vessel with a rectangular bore, analytic inner walls and a
// massive, vertically guided piston.  Bodies inside are hard-constrained to
// the chamber, so nothing can leak in or out; from outside the vessel is five
// static glass slabs plus the piston, and the space above the piston is open
// to the room (objects can rest on the piston and weigh it down).

#include "physics_object.h"
#include "thermo.h"

class GasContainer {
public:
    double left, right;      // x of the inner faces
    double back, front;      // z of the inner faces
    double bottom, wall_top; // y of the chamber floor and of the rim
    double piston_min_y, piston_max_y;   // travel of the piston's centre
    PhysicsObject* piston = nullptr;
    int group = 0;                        // shared by the piston and the wall slabs
    double pressure = thermo::kAtmosphere;
    double average_temperature = 20.0;
    double gas_volume = 0.0;              // cm³
    double previous_chamber_height = -1.0;
    double boiling_point = 100.0;
    int gas_particles = 0;
    int liquid_particles = 0;

    static constexpr double kWallHalfThickness = 4.0;
    static constexpr double kPistonHalfThickness = 5.0;

    GasContainer(const Vec3& centre, double half_width = 100.0, double half_height = 90.0, double half_depth = 70.0)
        : left(centre.x - half_width), right(centre.x + half_width),
          back(centre.z - half_depth), front(centre.z + half_depth),
          bottom(centre.y - half_height), wall_top(centre.y + half_height + 14.0),
          piston_min_y(centre.y - half_height + 30.0), piston_max_y(centre.y + half_height) {}

    GasContainer(double l, double r, double bk, double fr, double b, double wt, double pmin, double pmax)
        : left(l), right(r), back(bk), front(fr), bottom(b), wall_top(wt), piston_min_y(pmin), piston_max_y(pmax) {}

    // Underside of the piston: the top of the chamber.
    double top() const { return (piston ? piston->position.y : piston_max_y) - kPistonHalfThickness; }
    double width() const { return right - left; }
    double depth() const { return front - back; }
    double bore_area() const { return width() * depth(); }
    double chamber_height() const { return std::max(1.0, top() - bottom); }
    Vec3 centre() const { return Vec3(0.5 * (left + right), 0.5 * (bottom + wall_top), 0.5 * (back + front)); }
    bool in_chamber(const Vec3& p) const {
        return p.x > left && p.x < right && p.z > back && p.z < front && p.y > bottom && p.y < top() + 0.5;
    }
};
