#pragma once
// Small helpers shared by the World's implementation files.

#include "physics_object.h"

namespace detail {
constexpr double kCellSize = 20.0;                 // broad-phase grid, cm
constexpr double kPi = 3.14159265358979323846;
constexpr double kHydroRingWidth = 14.0;           // water this far outside a body counts as touching it

inline bool is_solid(const PhysicsObject& o) { return !o.is_fluid && !o.is_gas; }
// Bodies that float or sink by explicit buoyancy (rigid bodies and ice).
inline bool feels_hydrostatics(const PhysicsObject& o) {
    return is_solid(o) && !o.is_static && !o.is_dragged && !o.is_container_piston;
}
} // namespace detail
