#include "collision.h"

BoxShape box_of(const PhysicsObject& o) {
    BoxShape b;
    b.center = o.position;
    b.half = o.half_extents;
    if (o.orientation.is_identity()) {
        b.axis[0] = Vec3(1, 0, 0); b.axis[1] = Vec3(0, 1, 0); b.axis[2] = Vec3(0, 0, 1);
    } else {
        for (int i = 0; i < 3; ++i) b.axis[i] = o.orientation.axis(i);
    }
    return b;
}

Vec3 closest_point_on_box(const Vec3& p, const BoxShape& box) {
    const Vec3 rel = p - box.center;
    Vec3 result = box.center;
    for (int i = 0; i < 3; ++i)
        result += box.axis[i] * clampd(rel.dot(box.axis[i]), -box.half[i], box.half[i]);
    return result;
}

bool collide_sphere_box(const Vec3& centre, double radius, const BoxShape& box,
                        Vec3& normal, Vec3& on_box, double& depth) {
    const Vec3 rel = centre - box.center;
    double local[3];
    bool inside = true;
    on_box = box.center;
    for (int i = 0; i < 3; ++i) {
        local[i] = rel.dot(box.axis[i]);
        const double c = clampd(local[i], -box.half[i], box.half[i]);
        if (c != local[i]) inside = false;
        on_box += box.axis[i] * c;
    }
    if (!inside) {
        const Vec3 d = centre - on_box;
        const double dist2 = d.length_sq();
        if (dist2 >= radius * radius) return false;
        const double dist = std::sqrt(dist2);
        if (dist > 1e-9) {
            normal = d / dist;
            depth = radius - dist;
            return true;
        }
    }
    // Centre inside the box: leave through the nearest face.
    int best = 0;
    double least = 1e300;
    for (int i = 0; i < 3; ++i) {
        const double room = box.half[i] - std::abs(local[i]);
        if (room < least) { least = room; best = i; }
    }
    const double sign = local[best] >= 0.0 ? 1.0 : -1.0;
    normal = box.axis[best] * sign;
    on_box = centre + normal * least;
    depth = radius + least;
    return true;
}

namespace {
double projected_radius(const BoxShape& s, const Vec3& l) {
    return s.half.x * std::abs(s.axis[0].dot(l)) + s.half.y * std::abs(s.axis[1].dot(l))
         + s.half.z * std::abs(s.axis[2].dot(l));
}
} // namespace

int collide_box_box(const BoxShape& A, const BoxShape& B, Vec3& normal, ManifoldPoint out[8]) {
    const Vec3 d = A.center - B.center;
    double best_score = 1e300;
    int kind = -1, bi = 0, bj = 0;          // kind: 0 = face of A, 1 = face of B, 2 = edge pair
    Vec3 best_axis;

    auto test = [&](const Vec3& l, int k, int i, int j) {
        const double overlap = projected_radius(A, l) + projected_radius(B, l) - std::abs(d.dot(l));
        if (overlap < 0.0) return false;
        // Faces are preferred over edge pairs of nearly the same depth: they
        // give a full manifold, which is what keeps stacks from rocking.
        const double score = k == 2 ? overlap * 1.05 + 0.01 : overlap;
        if (score < best_score) { best_score = score; kind = k; bi = i; bj = j; best_axis = l; }
        return true;
    };

    for (int i = 0; i < 3; ++i) if (!test(A.axis[i], 0, i, 0)) return 0;
    for (int j = 0; j < 3; ++j) if (!test(B.axis[j], 1, j, 0)) return 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const Vec3 c = A.axis[i].cross(B.axis[j]);
            const double len2 = c.length_sq();
            if (len2 < 1e-6) continue;     // parallel edges: the face axes cover it
            if (!test(c / std::sqrt(len2), 2, i, j)) return 0;
        }
    }
    if (kind < 0) return 0;
    normal = best_axis.dot(d) >= 0.0 ? best_axis : -best_axis;

    if (kind == 2) {
        // The edge of A furthest towards B and the edge of B furthest towards A.
        Vec3 pa = A.center, pb = B.center;
        for (int k = 0; k < 3; ++k) {
            if (k != bi) pa += A.axis[k] * (A.half[k] * (normal.dot(A.axis[k]) > 0.0 ? -1.0 : 1.0));
            if (k != bj) pb += B.axis[k] * (B.half[k] * (normal.dot(B.axis[k]) > 0.0 ? 1.0 : -1.0));
        }
        const Vec3& ua = A.axis[bi];
        const Vec3& ub = B.axis[bj];
        const Vec3 w = pa - pb;
        const double b = ua.dot(ub), dd = ua.dot(w), e = ub.dot(w);
        const double denom = 1.0 - b * b;
        double s = 0.0, t = 0.0;
        if (denom > 1e-9) { s = (b * e - dd) / denom; t = (e - b * dd) / denom; }
        s = clampd(s, -A.half[bi], A.half[bi]);
        t = clampd(t, -B.half[bj], B.half[bj]);
        out[0].on_a = pa + ua * s;
        out[0].on_b = pb + ub * t;
        return 1;
    }

    const BoxShape& R = kind == 0 ? A : B;      // reference box (owns the face)
    const BoxShape& I = kind == 0 ? B : A;      // incident box
    const int r = bi;
    // Outward normal of the reference face, towards the incident box.
    const Vec3 n_ref = kind == 0 ? -normal : normal;

    // Incident face: the one most opposed to the reference normal.
    int k = 0;
    double most = -1.0;
    for (int a = 0; a < 3; ++a) {
        const double v = std::abs(I.axis[a].dot(n_ref));
        if (v > most) { most = v; k = a; }
    }
    const double side = I.axis[k].dot(n_ref) > 0.0 ? -1.0 : 1.0;
    const int u = (k + 1) % 3, v = (k + 2) % 3;
    const Vec3 fc = I.center + I.axis[k] * (side * I.half[k]);
    const Vec3 eu = I.axis[u] * I.half[u], ev = I.axis[v] * I.half[v];
    Vec3 poly[16], scratch[16];
    int n = 4;
    poly[0] = fc + eu + ev; poly[1] = fc - eu + ev; poly[2] = fc - eu - ev; poly[3] = fc + eu - ev;

    // Clip it against the four side planes of the reference face.
    for (int e = 1; e <= 2; ++e) {
        const int ax = (r + e) % 3;
        for (int pass = 0; pass < 2; ++pass) {
            const double sign = pass == 0 ? 1.0 : -1.0;
            int m = 0;
            for (int q = 0; q < n; ++q) {
                const Vec3& p0 = poly[q];
                const Vec3& p1 = poly[(q + 1) % n];
                const double d0 = sign * (p0 - R.center).dot(R.axis[ax]) - R.half[ax];
                const double d1 = sign * (p1 - R.center).dot(R.axis[ax]) - R.half[ax];
                if (d0 <= 0.0) scratch[m++] = p0;
                if ((d0 < 0.0 && d1 > 0.0) || (d0 > 0.0 && d1 < 0.0)) scratch[m++] = p0 + (p1 - p0) * (d0 / (d0 - d1));
                if (m >= 15) break;
            }
            n = m;
            for (int q = 0; q < n; ++q) poly[q] = scratch[q];
            if (n == 0) return 0;
        }
    }

    int count = 0;
    for (int q = 0; q < n && count < 8; ++q) {
        const double depth = R.half[r] - (poly[q] - R.center).dot(n_ref);
        if (depth < 0.0) continue;
        const Vec3 on_ref = poly[q] + n_ref * depth;
        if (kind == 0) { out[count].on_a = on_ref; out[count].on_b = poly[q]; }
        else           { out[count].on_a = poly[q]; out[count].on_b = on_ref; }
        ++count;
    }
    return count;
}

double surface_gap(const PhysicsObject& a, const PhysicsObject& b) {
    const bool a_box = a.shape_type == ShapeType::BOX, b_box = b.shape_type == ShapeType::BOX;
    if (!a_box && !b_box) return (a.position - b.position).length() - a.radius - b.radius;
    if (a_box && b_box) {
        const BoxShape sa = box_of(a), sb = box_of(b);
        const Vec3 p = closest_point_on_box(a.position, sb);
        const Vec3 q = closest_point_on_box(p, sa);
        return (q - closest_point_on_box(q, sb)).length();
    }
    const PhysicsObject& sphere = a_box ? b : a;
    const PhysicsObject& box = a_box ? a : b;
    const BoxShape s = box_of(box);
    return (sphere.position - closest_point_on_box(sphere.position, s)).length() - sphere.radius;
}

bool ray_sphere(const Vec3& origin, const Vec3& dir, const Vec3& centre, double radius, double& t) {
    const Vec3 oc = origin - centre;
    const double b = oc.dot(dir);
    const double c = oc.length_sq() - radius * radius;
    const double disc = b * b - c;
    if (disc < 0.0) return false;
    const double root = std::sqrt(disc);
    t = -b - root;
    if (t < 0.0) t = -b + root;
    return t >= 0.0;
}

bool ray_box(const Vec3& origin, const Vec3& dir, const BoxShape& box, double& t) {
    const Vec3 rel = origin - box.center;
    double t_near = -1e300, t_far = 1e300;
    for (int i = 0; i < 3; ++i) {
        const double o = rel.dot(box.axis[i]);
        const double d = dir.dot(box.axis[i]);
        if (std::abs(d) < 1e-12) {
            if (std::abs(o) > box.half[i]) return false;
            continue;
        }
        double t0 = (-box.half[i] - o) / d, t1 = (box.half[i] - o) / d;
        if (t0 > t1) std::swap(t0, t1);
        t_near = std::max(t_near, t0);
        t_far = std::min(t_far, t1);
        if (t_near > t_far) return false;
    }
    if (t_far < 0.0) return false;
    t = t_near >= 0.0 ? t_near : t_far;
    return true;
}
