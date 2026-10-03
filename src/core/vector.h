#pragma once
// Small 2D math helpers shared by the physics core.  No Qt dependency.

#include <algorithm>
#include <cmath>

struct Vector {
    double x = 0.0;
    double y = 0.0;

    constexpr Vector() = default;
    constexpr Vector(double x_, double y_) : x(x_), y(y_) {}

    constexpr Vector operator+(const Vector& o) const { return {x + o.x, y + o.y}; }
    constexpr Vector operator-(const Vector& o) const { return {x - o.x, y - o.y}; }
    constexpr Vector operator-() const { return {-x, -y}; }
    constexpr Vector operator*(double s) const { return {x * s, y * s}; }
    constexpr Vector operator/(double s) const { return {x / s, y / s}; }

    Vector& operator+=(const Vector& o) { x += o.x; y += o.y; return *this; }
    Vector& operator-=(const Vector& o) { x -= o.x; y -= o.y; return *this; }
    Vector& operator*=(double s) { x *= s; y *= s; return *this; }

    double length() const { return std::hypot(x, y); }
    constexpr double length_sq() const { return x * x + y * y; }
    constexpr double dot(const Vector& o) const { return x * o.x + y * o.y; }
    // z component of the 3D cross product.
    constexpr double cross(const Vector& o) const { return x * o.y - y * o.x; }
    // Rotated +90° (screen coordinates, y down: clockwise on screen).
    // omega × r for an angular velocity omega is r.perp() * omega.
    constexpr Vector perp() const { return {-y, x}; }
};

constexpr Vector operator*(double s, const Vector& v) { return v * s; }

inline bool is_valid(double v) { return std::isfinite(v); }
inline bool is_valid(const Vector& v) { return std::isfinite(v.x) && std::isfinite(v.y); }

inline double clampd(double value, double minimum, double maximum) {
    return std::max(minimum, std::min(value, maximum));
}
