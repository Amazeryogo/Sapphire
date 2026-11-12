#pragma once
// Orbit camera for the 3D view: it circles a target point, and converts
// between screen pixels and world-space rays.

#include "vector.h"

#include <QMatrix4x4>
#include <QPointF>
#include <QVector3D>

#include <cmath>

struct Ray {
    Vec3 origin;
    Vec3 direction;   // unit
    Vec3 at(double t) const { return origin + direction * t; }
};

class Camera {
public:
    Vec3 target = Vec3(450.0, 230.0, 250.0);
    double yaw = 0.0;          // radians about the vertical; 0 looks along -z
    double pitch = 0.30;       // radians above the horizon
    double distance = 1250.0;  // cm from the target
    double fov_degrees = 42.0;

    Vec3 eye() const {
        return target + Vec3(std::cos(pitch) * std::sin(yaw), std::sin(pitch), std::cos(pitch) * std::cos(yaw)) * distance;
    }
    Vec3 forward() const { return (target - eye()).normalized(Vec3(0, 0, -1)); }
    Vec3 right() const { return forward().cross(Vec3(0, 1, 0)).normalized(Vec3(1, 0, 0)); }
    Vec3 up() const { return right().cross(forward()); }

    QMatrix4x4 view() const {
        QMatrix4x4 m;
        const Vec3 e = eye();
        m.lookAt(QVector3D(float(e.x), float(e.y), float(e.z)),
                 QVector3D(float(target.x), float(target.y), float(target.z)), QVector3D(0, 1, 0));
        return m;
    }
    QMatrix4x4 projection(double aspect) const {
        QMatrix4x4 m;
        m.perspective(float(fov_degrees), float(aspect), 8.0f, 9000.0f);
        return m;
    }

    // Ray through a pixel of a viewport of the given size.
    Ray ray(const QPointF& pixel, double width, double height) const {
        const double tan_half = std::tan(0.5 * fov_degrees * 3.14159265358979323846 / 180.0);
        const double nx = (2.0 * pixel.x() / width - 1.0) * tan_half * (width / height);
        const double ny = (1.0 - 2.0 * pixel.y() / height) * tan_half;
        return Ray{eye(), (forward() + right() * nx + up() * ny).normalized()};
    }

    // Pixel a world point lands on; false if it is behind the camera.
    bool project(const Vec3& world, double width, double height, QPointF& pixel) const {
        const Vec3 rel = world - eye();
        const double z = rel.dot(forward());
        if (z < 1.0) return false;
        const double tan_half = std::tan(0.5 * fov_degrees * 3.14159265358979323846 / 180.0);
        const double nx = rel.dot(right()) / (z * tan_half * (width / height));
        const double ny = rel.dot(up()) / (z * tan_half);
        pixel = QPointF((nx + 1.0) * 0.5 * width, (1.0 - ny) * 0.5 * height);
        return true;
    }
    // Screen pixels per cm at a world point (for sizing overlays).
    double pixels_per_cm(const Vec3& world, double height) const {
        const double z = std::max(1.0, (world - eye()).dot(forward()));
        return height / (2.0 * z * std::tan(0.5 * fov_degrees * 3.14159265358979323846 / 180.0));
    }

    void orbit(double dx_pixels, double dy_pixels) {
        yaw -= dx_pixels * 0.006;
        pitch = clampd(pitch + dy_pixels * 0.006, -0.15, 1.5);
    }
    void pan(double dx_pixels, double dy_pixels, double viewport_height) {
        const double scale = 1.0 / pixels_per_cm(target, viewport_height);
        target = target - right() * (dx_pixels * scale) + up() * (dy_pixels * scale);
    }
    void zoom(double steps) { distance = clampd(distance * std::pow(0.9, steps), 150.0, 4000.0); }
};

// Point where a ray meets a plane, if it does (in front of the origin).
inline bool intersect_plane(const Ray& ray, const Vec3& point, const Vec3& normal, Vec3& hit) {
    const double denom = ray.direction.dot(normal);
    if (std::abs(denom) < 1e-9) return false;
    const double t = (point - ray.origin).dot(normal) / denom;
    if (t < 0.0) return false;
    hit = ray.at(t);
    return true;
}
