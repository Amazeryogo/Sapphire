#include "constraints.h"

void Spring::apply_forces() const {
    const Vector delta = obj_a->position - obj_b->position;
    const double distance = delta.length();
    if (distance < 1e-9) return;
    const Vector dir = delta / distance;
    const double closing = (obj_a->velocity - obj_b->velocity).dot(dir);
    const double magnitude = (distance - length) * stiffness + closing * damping;
    obj_a->apply_force(dir * -magnitude);
    obj_b->apply_force(dir * magnitude);
}

void RigidLink::solve() const {
    const Vector delta = obj_b->position - obj_a->position;
    const double dist = delta.length();
    if (dist < 1e-9) return;
    const double wa = obj_a->inv_mass(), wb = obj_b->inv_mass();
    if (wa + wb == 0.0) return;
    const Vector correction = delta * ((dist - length) / (dist * (wa + wb)));
    obj_a->position += correction * wa;
    obj_b->position -= correction * wb;
}

void Pulley::solve() const {
    const Vector da = obj_a->position - anchor, db = obj_b->position - anchor;
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
    const Vector delta = obj_a->position - obj_b->position;
    const double dist = delta.length();
    if (dist < 1e-6) return;
    const Vector dir = delta / dist;
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
    const Vector delta = obj_a->position - obj_b->position;
    const double dist = delta.length();
    if (dist < 1e-6 || (dist >= min_len && dist <= max_len)) return;
    const double inv_a = obj_a->inv_mass(), inv_b = obj_b->inv_mass();
    if (inv_a + inv_b == 0.0) return;
    const Vector dir = delta / dist;
    const double error = dist - (dist < min_len ? min_len : max_len);
    obj_a->position -= dir * (error * inv_a / (inv_a + inv_b));
    obj_b->position += dir * (error * inv_b / (inv_a + inv_b));
}
