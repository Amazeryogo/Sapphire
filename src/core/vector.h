#pragma once
// 3D math helpers shared by the physics core.  No Qt dependency.
//
// World axes: x to the right, y up, z towards the default camera.
// Units: centimetres, kilograms and frames (1 frame = 1/60 s).

#include <algorithm>
#include <cmath>

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    constexpr Vec3() = default;
    constexpr Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    constexpr Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }

    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }

    double operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
    double& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }

    double length() const { return std::sqrt(x * x + y * y + z * z); }
    constexpr double length_sq() const { return x * x + y * y + z * z; }
    constexpr double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    constexpr Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    // Component-wise product (used for diagonal inertia tensors).
    constexpr Vec3 scaled(const Vec3& o) const { return {x * o.x, y * o.y, z * o.z}; }
    Vec3 normalized(const Vec3& fallback = Vec3(0.0, 1.0, 0.0)) const {
        const double len = length();
        return len > 1e-12 ? *this / len : fallback;
    }
};

using Vector = Vec3;

constexpr Vec3 operator*(double s, const Vec3& v) { return v * s; }

// Unit quaternion (rotation from body space to world space).
struct Quat {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    constexpr Quat() = default;
    constexpr Quat(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}

    static Quat from_axis_angle(const Vec3& axis, double angle) {
        const Vec3 a = axis.normalized();
        const double s = std::sin(0.5 * angle);
        return {std::cos(0.5 * angle), a.x * s, a.y * s, a.z * s};
    }

    constexpr Quat operator*(const Quat& o) const {
        return {w * o.w - x * o.x - y * o.y - z * o.z,
                w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x,
                w * o.z + x * o.y - y * o.x + z * o.w};
    }
    constexpr Quat conjugate() const { return {w, -x, -y, -z}; }

    Vec3 rotate(const Vec3& v) const {
        const Vec3 q(x, y, z);
        const Vec3 t = q.cross(v) * 2.0;
        return v + t * w + q.cross(t);
    }
    Vec3 inv_rotate(const Vec3& v) const { return conjugate().rotate(v); }
    // World-space direction of body axis i (0 = x, 1 = y, 2 = z).
    Vec3 axis(int i) const { return rotate(Vec3(i == 0, i == 1, i == 2)); }

    void normalize() {
        const double n = std::sqrt(w * w + x * x + y * y + z * z);
        if (n > 1e-12) { w /= n; x /= n; y /= n; z /= n; }
        else { w = 1.0; x = y = z = 0.0; }
    }
    // Rotate by a small world-space rotation vector (axis · angle).
    void integrate(const Vec3& rotation) {
        const Quat dq = Quat(0.0, rotation.x, rotation.y, rotation.z) * *this;
        w += 0.5 * dq.w; x += 0.5 * dq.x; y += 0.5 * dq.y; z += 0.5 * dq.z;
        normalize();
    }
    constexpr bool is_identity() const { return w == 1.0 && x == 0.0 && y == 0.0 && z == 0.0; }
};

inline bool is_valid(double v) { return std::isfinite(v); }
inline bool is_valid(const Vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
inline bool is_valid(const Quat& q) {
    return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}

inline double clampd(double value, double minimum, double maximum) {
    return std::max(minimum, std::min(value, maximum));
}
