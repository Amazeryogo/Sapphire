#include "constraints.h"

void Spring::apply_forces() const {
    const Vec3 ra = obj_a->orientation.rotate(anchor_a), rb = obj_b->orientation.rotate(anchor_b);
    const Vec3 delta = (obj_a->position + ra) - (obj_b->position + rb);
    const double distance = delta.length();
    if (distance < 1e-9) return;
    const Vec3 dir = delta / distance;
    const double closing = (obj_a->velocity_at(ra) - obj_b->velocity_at(rb)).dot(dir);
    const double magnitude = (distance - length) * stiffness + closing * damping;
    obj_a->apply_force_at(dir * -magnitude, ra);
    obj_b->apply_force_at(dir * magnitude, rb);
}

void RigidLink::solve() const {
    if (!anchored) {
        const Vec3 delta = obj_b->position - obj_a->position;
        const double dist = delta.length();
        if (dist < 1e-9) return;
        const double wa = obj_a->inv_mass(), wb = obj_b->inv_mass();
        if (wa + wb == 0.0) return;
        const Vec3 correction = delta * ((dist - length) / (dist * (wa + wb)));
        obj_a->position += correction * wa;
        obj_b->position -= correction * wb;
        return;
    }
    const Vec3 ra = obj_a->orientation.rotate(anchor_a), rb = obj_b->orientation.rotate(anchor_b);
    const Vec3 delta = (obj_b->position + rb) - (obj_a->position + ra);
    const double dist = delta.length();
    if (dist < 1e-9) return;
    const Vec3 n = delta / dist;
    const double wa = obj_a->generalized_inv_mass(ra, n), wb = obj_b->generalized_inv_mass(rb, n);
    if (wa + wb == 0.0) return;
    const Vec3 p = n * ((dist - length) / (wa + wb));
    obj_a->apply_position_impulse(p, ra);
    obj_b->apply_position_impulse(-p, rb);
}

void Pulley::solve() const {
    const Vec3 da = obj_a->position - anchor, db = obj_b->position - anchor;
    const double la = da.length(), lb = db.length();
    const double c = la + lb - total_length;
    if (c <= 0.0) return;                         // rope slack
    const double wa = obj_a->inv_mass(), wb = obj_b->inv_mass();
    if (wa + wb == 0.0) return;
    const double lambda = c / (wa + wb);
    if (la > 1e-9) obj_a->position -= da * (lambda * wa / la);
    if (lb > 1e-9) obj_b->position -= db * (lambda * wb / lb);
}

void PistonLink::update_gas(double ambient_temperature) {
    const double dist = (obj_a->position - obj_b->position).length();
    if (dist < 1e-6) return;
    // Adiabatic compression of the charge: T·L^(γ-1) = const.
    const double gas_k = thermo::kelvin(gas_temperature) * std::pow(previous_length / std::max(1.0, dist), 0.4);
    gas_temperature = gas_k + thermo::kAbsoluteZero;
    previous_length = dist;
    // Heat exchange through the cylinder wall.
    const double surroundings = 0.5 * ambient_temperature + 0.25 * (obj_a->temperature + obj_b->temperature);
    gas_temperature += (surroundings - gas_temperature) * 0.02;
}

void PistonLink::apply_forces() const {
    const Vec3 delta = obj_a->position - obj_b->position;
    const double dist = delta.length();
    if (dist < 1e-6) return;
    const Vec3 dir = delta / dist;
    const double inv_a = obj_a->inv_mass(), inv_b = obj_b->inv_mass();
    if (inv_a + inv_b == 0.0) return;
    const double reduced_mass = 1.0 / (inv_a + inv_b);
    const double rel_vel = (obj_a->velocity - obj_b->velocity).dot(dir);
    const double rel_accel = stiffness * rest_length * gauge_pressure_ratio(dist) - damping * rel_vel;
    const double force = reduced_mass * rel_accel;
    obj_a->acceleration += dir * (force * inv_a);
    obj_b->acceleration -= dir * (force * inv_b);
}

void PistonLink::solve_stops() const {
    const Vec3 delta = obj_a->position - obj_b->position;
    const double dist = delta.length();
    if (dist < 1e-6 || (dist >= min_len && dist <= max_len)) return;
    const double inv_a = obj_a->inv_mass(), inv_b = obj_b->inv_mass();
    if (inv_a + inv_b == 0.0) return;
    const Vec3 dir = delta / dist;
    const double error = dist - (dist < min_len ? min_len : max_len);
    obj_a->position -= dir * (error * inv_a / (inv_a + inv_b));
    obj_b->position += dir * (error * inv_b / (inv_a + inv_b));
}

namespace {
// Changes the wheel's spin by dw and gives the frame the opposite angular momentum.
void exchange_spin(PhysicsObject* wheel, PhysicsObject* frame, const Vec3& dw) {
    if (!wheel->rotates_now()) return;
    wheel->angular_velocity += dw;
    if (!frame->rotates_now() || wheel->inv_inertia_body.x <= 0.0) return;
    const Vec3 momentum = dw / wheel->inv_inertia_body.x;       // wheels are spheres
    frame->angular_velocity -= frame->apply_inv_inertia(momentum);
}
} // namespace

void Axle::drive(double max_step) const {
    const Vec3 axis = world_axis();
    wheel->motor_axis = axis;
    wheel->on_axle = true;
    if (!wheel->is_motor) return;
    const double spin = (wheel->angular_velocity - frame->angular_velocity).dot(axis);
    exchange_spin(wheel, frame, axis * clampd(wheel->motor_speed - spin, -max_step, max_step));
}

void Axle::solve_velocity() const {
    const Vec3 axis = world_axis();
    const Vec3 relative = wheel->angular_velocity - frame->angular_velocity;
    exchange_spin(wheel, frame, axis * relative.dot(axis) - relative);
}

RigidCluster::RigidCluster(const std::vector<PhysicsObject*>& beads) : nodes(beads) {
    Vec3 centre;
    double total = 0.0;
    for (const PhysicsObject* o : nodes) { centre += o->position * o->mass; total += o->mass; }
    if (total > 0.0) centre = centre / total;
    for (const PhysicsObject* o : nodes) rest.push_back(o->position - centre);
}

void RigidCluster::solve() {
    const std::size_t n = nodes.size();
    if (n < 2 || rest.size() != n) return;
    // A held or pinned bead is (nearly) immovable: the shape turns about it.
    auto weight = [](const PhysicsObject* o) { return (o->is_static || o->is_dragged) ? o->mass * 1e6 : o->mass; };
    Vec3 centre, rest_centre;
    double total = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const double w = weight(nodes[i]);
        centre += nodes[i]->position * w;
        rest_centre += rest[i] * w;
        total += w;
    }
    if (!(total > 0.0)) return;
    centre = centre / total;
    rest_centre = rest_centre / total;

    // A = Σ w (x - c)(r - r̄)ᵀ, stored by columns.
    Vec3 a[3];
    for (std::size_t i = 0; i < n; ++i) {
        const Vec3 p = (nodes[i]->position - centre) * weight(nodes[i]);
        const Vec3 r = rest[i] - rest_centre;
        a[0] += p * r.x; a[1] += p * r.y; a[2] += p * r.z;
    }
    // Rotation closest to A (Müller et al. 2016, "A Robust Method to Extract
    // the Rotational Part of Deformations"), warm-started from the last one.
    for (int iteration = 0; iteration < 8; ++iteration) {
        const Vec3 r0 = rotation.axis(0), r1 = rotation.axis(1), r2 = rotation.axis(2);
        const Vec3 omega = (r0.cross(a[0]) + r1.cross(a[1]) + r2.cross(a[2]))
                         / (std::abs(r0.dot(a[0]) + r1.dot(a[1]) + r2.dot(a[2])) + 1e-9);
        const double angle = omega.length();
        if (angle < 1e-9) break;
        rotation = Quat::from_axis_angle(omega / angle, angle) * rotation;
        rotation.normalize();
    }
    for (std::size_t i = 0; i < n; ++i) {
        if (nodes[i]->is_static || nodes[i]->is_dragged) continue;
        nodes[i]->position = centre + rotation.rotate(rest[i] - rest_centre);
    }
}
