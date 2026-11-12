#pragma once
// Narrow-phase geometry: sphere and oriented-box tests, ray casts and
// surface distances.  Pure functions, no Qt dependency.

#include "physics_object.h"

struct BoxShape {
    Vec3 center;
    Vec3 half;
    Vec3 axis[3];   // unit, world space
};

BoxShape box_of(const PhysicsObject& o);

Vec3 closest_point_on_box(const Vec3& p, const BoxShape& box);

// Sphere against box.  'normal' points from the box towards the sphere,
// 'on_box' is the contact point on the box's surface.
bool collide_sphere_box(const Vec3& centre, double radius, const BoxShape& box,
                        Vec3& normal, Vec3& on_box, double& depth);

struct ManifoldPoint {
    Vec3 on_a;   // point on A's surface
    Vec3 on_b;   // point on B's surface; (on_b - on_a)·normal is the penetration depth
};

// Box against box (separating-axis test, then face clipping or the closest
// points of the two touching edges).  'normal' points from B towards A.
// Returns the number of contact points written (0 = no overlap, at most 8).
int collide_box_box(const BoxShape& a, const BoxShape& b, Vec3& normal, ManifoldPoint out[8]);

// Approximate distance between two bodies' surfaces (<= 0 when they overlap).
double surface_gap(const PhysicsObject& a, const PhysicsObject& b);

// Distance along a ray (unit direction) to the first hit, if any.
bool ray_sphere(const Vec3& origin, const Vec3& dir, const Vec3& centre, double radius, double& t);
bool ray_box(const Vec3& origin, const Vec3& dir, const BoxShape& box, double& t);
