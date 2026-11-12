#include <QApplication>
#include <QWidget>
#include <QMainWindow>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QMouseEvent>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QLabel>
#include <QSlider>
#include <QStackedWidget>
#include <QInputDialog>
#include <QColor>
#include <QFont>
#include <QPen>
#include <QBrush>
#include <QPolygonF>
#include <cmath>
#include <vector>
#include <memory>
#include <random>
#include <algorithm>
#include <limits>

enum class ShapeType { CIRCLE, SQUARE, TRIANGLE };
enum class ObjectType { BALL, BOX, SPRING, PULLEY, WHEEL, MOTOR, CHASSIS, PIN, ROPE, WATER, GAS, POLYGON, HEATER, COOLER, BOMB, PISTON, CONTAINER };

static std::mt19937& rng() {
    static std::mt19937 gen(std::random_device{}());
    return gen;
}

double random_double(double min, double max) {
    std::uniform_real_distribution<double> dist(min, max);
    return dist(rng());
}

int random_int(int min, int max) {
    std::uniform_int_distribution<int> dist(min, max);
    return dist(rng());
}

double random_normal() {
    static std::normal_distribution<double> dist(0.0, 1.0);
    return dist(rng());
}

static double clampd(double value, double minimum, double maximum) {
    return std::max(minimum, std::min(value, maximum));
}

struct Vector {
    double x, y, z;
    Vector(double _x = 0.0, double _y = 0.0, double _z = 0.0) : x(_x), y(_y), z(_z) {}

    Vector operator+(const Vector& o) const { return Vector(x + o.x, y + o.y, z + o.z); }
    Vector operator-(const Vector& o) const { return Vector(x - o.x, y - o.y, z - o.z); }
    Vector operator*(double s) const { return Vector(x * s, y * s, z * s); }
    Vector operator/(double s) const { return Vector(x / s, y / s, z / s); }

    Vector& operator+=(const Vector& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vector& operator-=(const Vector& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }

    double length() const { return std::hypot(x, y, z); }
    double dot(const Vector& o) const { return x * o.x + y * o.y + z * o.z; }
    QPointF to_qpointf() const { return QPointF(x, y); }
};

bool is_valid(const Vector& val) {
    return !(std::isnan(val.x) || std::isnan(val.y) || std::isinf(val.x) || std::isinf(val.y));
}
bool is_valid(double val) {
    return !(std::isnan(val) || std::isinf(val));
}

// ==========================================
// THERMODYNAMIC MATERIAL MODELS
// ==========================================
// Energies are in "sim joules": specific heats use real kJ/(kg·K) values and
// one unit of object mass is treated as 1 kg.  Latent heats keep their real
// ratio to each other and to the specific heats, but are uniformly scaled by
// kLatentScale so that phase changes happen in seconds instead of minutes.
namespace thermo {
constexpr double kAtmosphere   = 101.325;   // kPa
constexpr double kAbsoluteZero = -273.15;   // °C
constexpr double kMeltPoint    = 0.0;       // °C (pressure dependence is negligible)
constexpr double kLatentScale  = 0.25;
constexpr double kLatentFusion = 334.0 * kLatentScale;   // per unit mass
constexpr double kLatentVapor  = 2257.0 * kLatentScale;  // per unit mass
constexpr double kWaterMolarVaporHeat = 40660.0;          // J/mol (Clausius–Clapeyron)
constexpr double kGasConstant  = 8.314;                   // J/(mol·K)
constexpr double kAirMolarMass   = 28.97;
constexpr double kSteamMolarMass = 18.015;
// RMS particle speed (px/frame) of air at 20 °C.  v_rms ∝ sqrt(T / M).
const double kGasSpeedScale = 2.0 / std::sqrt(293.15);
constexpr double kGasCollisionRate = 0.03; // Langevin relaxation rate (1/frame)

inline double kelvin(double celsius) { return std::max(1.0, celsius - kAbsoluteZero); }

// Clausius–Clapeyron: boiling point of water at pressure p (kPa).
inline double boiling_point(double pressure_kpa) {
    const double p = clampd(pressure_kpa, 0.62, 5000.0); // triple point .. 50 bar
    const double inverse = 1.0 / 373.15 - (kGasConstant / kWaterMolarVaporHeat) * std::log(p / kAtmosphere);
    return 1.0 / inverse + kAbsoluteZero;
}

// Inverse of the above: saturation vapour pressure of water (kPa).
inline double saturation_pressure(double celsius) {
    return kAtmosphere * std::exp(-(kWaterMolarVaporHeat / kGasConstant)
                                  * (1.0 / kelvin(celsius) - 1.0 / 373.15));
}

// Density of liquid water (kg/m³), Tanaka et al. (2001).  Captures the
// density maximum at ~4 °C, so water expands both on heating and on cooling
// towards freezing.
inline double water_density(double celsius) {
    const double t = clampd(celsius, 0.0, 150.0);
    const double a = (t + 288.9414) / (508929.2 * (t + 68.12963));
    return 999.97495 * (1.0 - a * (t - 3.9863) * (t - 3.9863));
}

// Dynamic viscosity of water relative to 20 °C (Vogel equation).
inline double water_viscosity_ratio(double celsius) {
    auto mu = [](double c) { return 2.414e-5 * std::pow(10.0, 247.8 / (kelvin(clampd(c, 0.0, 150.0)) - 140.0)); };
    return mu(celsius) / mu(20.0);
}

// Surface tension of water relative to 20 °C (IAPWS correlation).
inline double water_surface_tension_ratio(double celsius) {
    auto sigma = [](double c) {
        const double tau = 1.0 - kelvin(clampd(c, 0.0, 370.0)) / 647.096;
        return 235.8 * std::pow(tau, 1.256) * (1.0 - 0.625 * tau);
    };
    return sigma(celsius) / sigma(20.0);
}

inline double smoothstep(double edge0, double edge1, double x) {
    const double t = clampd((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

// Viscoelastic elastomer (rubber ball) coefficient-of-restitution model,
// normalised so the user's "Bounciness" slider is the value at 20 °C.
//  * Below the glass transition (~ -45 °C) the polymer is glassy: hard, but
//    brittle and only moderately bouncy.
//  * At the glass transition internal friction (loss tangent) peaks, so the
//    ball is almost dead – the classic "unhappy ball".
//  * Warming from 0 to ~45 °C makes rubber springier (why squash balls are
//    warmed up before play).
//  * Above ~50 °C the material softens and hysteresis climbs steeply until a
//    hot ball barely bounces at all.
inline double elastomer_restitution_raw(double celsius) {
    constexpr double glass_transition = -45.0;
    const double rubbery = 1.0 / (1.0 + std::exp(-(celsius - glass_transition) / 8.0));
    const double base = 0.55 + 0.45 * rubbery;
    const double loss_peak = 0.75 * std::exp(-std::pow((celsius - glass_transition) / 14.0, 2.0));
    const double warm_up = 0.85 + 0.15 * smoothstep(-10.0, 45.0, celsius);
    const double softening = 1.0 - 0.92 * smoothstep(40.0, 200.0, celsius);
    return base * (1.0 - loss_peak) * warm_up * softening;
}
inline double elastomer_restitution_factor(double celsius) {
    static const double reference = elastomer_restitution_raw(20.0);
    return elastomer_restitution_raw(celsius) / reference;
}
} // namespace thermo

class PhysicsObject {
public:
    Vector position;
    Vector previous_position;
    Vector velocity;
    Vector acceleration;
    double radius;
    double liquid_radius;    // radius to restore when steam/ice turns back into water
    double elasticity;       // restitution at 20 °C (from the Bounciness slider)
    double mass;
    QColor color;
    ShapeType shape_type;
    bool is_static;
    bool is_fluid;
    bool is_gas = false;
    bool is_dragged = false;
    bool is_motor = false;
    bool is_bouncy = true;
    double angular_velocity = 0.0;
    double temperature = 20.0;
    bool is_heater = false;
    bool is_cooler = false;
    bool is_steam = false;
    bool is_ice = false;
    double latent_energy = 0.0;     // progress of an unfinished phase change
    double molar_ratio = 1.0;       // molar mass relative to air (gases)
    double local_pressure = thermo::kAtmosphere;
    int container_index = -1;       // sealed vessel this object is inside, if any
    bool is_container_piston = false;
    int fuse = -1;

    std::vector<QPointF> trail;

    PhysicsObject(double x, double y, double r = 18.0, double el = 0.8, double m = -1.0,
                  QColor c = QColor(), ShapeType st = ShapeType::CIRCLE,
                  bool is_stat = false, bool is_fl = false)
    {
        position = Vector(x, y, 0.0);
        previous_position = position;
        velocity = is_stat ? Vector(0,0,0) : Vector(random_double(-2.0, 2.0), random_double(-2.0, 1.0), 0.0);
        acceleration = Vector(0,0,0);
        radius = r;
        liquid_radius = r;
        elasticity = el;
        shape_type = st;
        is_static = is_stat;
        is_fluid = is_fl;
        mass = (m < 0) ? (r * r) / 100.0 : m;

        if (is_fluid) {
            color = QColor(60, 150, 255, 200);
        } else {
            color = c.isValid() ? c : QColor(random_int(80, 255), random_int(80, 255), random_int(80, 255));
        }
    }

    void apply_force(const Vector& force) {
        if (!is_static) acceleration += force;
    }

    // Specific heat capacity, kJ/(kg·K).  Gases use c_v because the sandbox's
    // vessels are closed; free gas exchanges with the room at ~constant volume
    // per frame as well.
    double specific_heat() const {
        if (is_ice) return 2.09;
        if (is_fluid) return 4.18;
        if (is_steam) return 1.41;
        if (is_gas) return 0.718;
        return 0.9;
    }
    double thermal_mass() const { return std::max(0.05, mass * specific_heat()); }
    double heat_capacity_ratio() const { return is_steam ? 1.33 : 1.40; }

    // Relative thermal conductivity used for contact conduction.
    double conductivity() const {
        if (is_heater || is_cooler) return 20.0;  // metal plate
        if (is_ice) return 3.7;                   // 2.2 W/mK vs water 0.6
        if (is_fluid) return 1.0;
        if (is_gas) return 0.04;                  // 0.025 W/mK
        return 0.5;
    }

    bool is_water_substance() const { return is_fluid || is_ice || is_steam; }
    double boiling_point() const { return thermo::boiling_point(local_pressure); }

    // Restitution of this body at its current temperature.  Rigid static
    // bodies return 1 so a collision is governed by the softer partner.
    double effective_restitution() const {
        if (is_gas) return 1.0;              // molecular collisions are elastic
        if (is_static) return 1.0;
        if (is_fluid) return elasticity;
        if (!is_bouncy) return elasticity;
        return clampd(elasticity * thermo::elastomer_restitution_factor(temperature), 0.02, 0.98);
    }

    void become_liquid() {
        is_ice = false; is_gas = false; is_steam = false;
        is_fluid = true; is_static = false;
        radius = liquid_radius;
        elasticity = 0.1; is_bouncy = false;
        molar_ratio = 1.0;
        color = QColor(60, 150, 255, 200);
    }
    void become_ice() {
        is_fluid = false; is_gas = false; is_steam = false;
        is_ice = true; is_static = true;
        velocity = Vector(0, 0, 0);
        radius = liquid_radius * 1.03;   // ice is ~9 % less dense: (1.09)^(1/3)
        elasticity = 0.0; is_bouncy = false;
        color = QColor(150, 240, 255, 220);
    }
    void become_steam() {
        is_fluid = false; is_ice = false;
        is_gas = true; is_steam = true;
        radius = std::max(2.0, liquid_radius * 0.4);
        elasticity = 1.0; is_bouncy = false;
        molar_ratio = thermo::kSteamMolarMass / thermo::kAirMolarMass;
        color = QColor(225, 225, 235, 150);
    }

    // Adds energy (not temperature).  Water-substance objects hold their
    // temperature at a phase boundary while latent heat accumulates, and the
    // boiling point follows the local pressure (Clausius–Clapeyron).
    void add_heat(double energy) {
        if (is_heater || is_cooler || is_container_piston || !is_valid(energy)) return;
        if (!is_water_substance()) {
            temperature = std::max(thermo::kAbsoluteZero + 1.0, temperature + energy / thermal_mass());
            return;
        }

        const double fusion = mass * thermo::kLatentFusion;
        const double vapor = mass * thermo::kLatentVapor;
        const double melt = thermo::kMeltPoint;
        const double boil = boiling_point();

        // A pressure change can leave a phase off its boundary (superheated
        // water after decompression, supersaturated steam after compression).
        // Release that sensible heat into the phase change.
        if (latent_energy == 0.0) {
            if (is_ice && temperature > melt) { energy += (temperature - melt) * thermal_mass(); temperature = melt; }
            else if (is_fluid && temperature > boil) { energy += (temperature - boil) * thermal_mass(); temperature = boil; }
            else if (is_fluid && temperature < melt) { energy += (temperature - melt) * thermal_mass(); temperature = melt; }
            else if (is_steam && temperature < boil) { energy += (temperature - boil) * thermal_mass(); temperature = boil; }
        }

        for (int step = 0; step < 8 && std::abs(energy) > 1e-12; ++step) {
            const double c = thermal_mass();
            if (is_ice) {
                if (latent_energy > 0.0 || (energy > 0.0 && temperature >= melt)) {
                    temperature = melt;
                    latent_energy += energy; energy = 0.0;
                    if (latent_energy >= fusion) { energy = latent_energy - fusion; latent_energy = 0.0; become_liquid(); }
                    else if (latent_energy < 0.0) { energy = latent_energy; latent_energy = 0.0; }
                } else {
                    const double next = temperature + energy / c;
                    if (energy > 0.0 && next > melt) { energy -= (melt - temperature) * c; temperature = melt; }
                    else { temperature = std::max(thermo::kAbsoluteZero + 1.0, next); energy = 0.0; }
                }
            } else if (is_fluid) {
                if (latent_energy < 0.0 || (energy < 0.0 && temperature <= melt)) {         // freezing
                    temperature = melt;
                    latent_energy += energy; energy = 0.0;
                    if (latent_energy <= -fusion) { energy = latent_energy + fusion; latent_energy = 0.0; become_ice(); }
                    else if (latent_energy > 0.0) { energy = latent_energy; latent_energy = 0.0; }
                } else if (latent_energy > 0.0 || (energy > 0.0 && temperature >= boil)) {   // boiling
                    temperature = boil;
                    latent_energy += energy; energy = 0.0;
                    if (latent_energy >= vapor) { energy = latent_energy - vapor; latent_energy = 0.0; become_steam(); }
                    else if (latent_energy < 0.0) { energy = latent_energy; latent_energy = 0.0; }
                } else {                                                                    // sensible
                    const double next = temperature + energy / c;
                    if (energy > 0.0 && next > boil) { energy -= (boil - temperature) * c; temperature = boil; }
                    else if (energy < 0.0 && next < melt) { energy -= (melt - temperature) * c; temperature = melt; }
                    else { temperature = next; energy = 0.0; }
                }
            } else { // steam
                if (latent_energy < 0.0 || (energy < 0.0 && temperature <= boil)) {         // condensing
                    temperature = boil;
                    latent_energy += energy; energy = 0.0;
                    if (latent_energy <= -vapor) { energy = latent_energy + vapor; latent_energy = 0.0; become_liquid(); }
                    else if (latent_energy > 0.0) { energy = latent_energy; latent_energy = 0.0; }
                } else {
                    const double next = temperature + energy / c;
                    if (energy < 0.0 && next < boil) { energy -= (boil - temperature) * c; temperature = boil; }
                    else { temperature = next; energy = 0.0; }
                }
            }
        }
    }

    void update(const Vector& gravity, double air_resistance, double bounds_width, double bounds_height, double ambient_temp) {
        if (is_static) {
            velocity = Vector(0,0,0);
            return;
        }

        if (is_dragged) {
            acceleration = Vector(0,0,0);
            if (!is_fluid) update_trail();
            return;
        }

        if (is_motor) velocity.x += angular_velocity;

        if (is_gas) {
            // Kinetic theory: temperature *is* the particles' random kinetic
            // energy.  An Ornstein–Uhlenbeck (Langevin) thermostat relaxes each
            // particle to a Maxwell distribution with v_rms ∝ sqrt(T / M), so
            // hot gas moves faster and light steam faster than air.
            const double v_rms = thermo::kGasSpeedScale * std::sqrt(thermo::kelvin(temperature) / molar_ratio);
            const double sigma = v_rms / std::sqrt(2.0);
            const double decay = std::exp(-thermo::kGasCollisionRate);
            const double kick = sigma * std::sqrt(1.0 - decay * decay);
            velocity.x = velocity.x * decay + kick * random_normal();
            velocity.y = velocity.y * decay + kick * random_normal();

            // Archimedes with ideal-gas densities: rho_gas / rho_air =
            // (M_gas / M_air) · (T_air / T_gas).  Hot or light gas rises, cold
            // or heavy gas sinks; gas at room temperature is neutrally buoyant.
            // Inside a sealed vessel there is no surrounding air, and the
            // weight of the gas itself is negligible.
            if (container_index < 0) {
                const double density_ratio = molar_ratio * thermo::kelvin(ambient_temp) / thermo::kelvin(temperature);
                velocity.y += gravity.y * (density_ratio - 1.0);
            }
            velocity += acceleration;
        } else {
            apply_force(gravity);
            velocity += acceleration;
            velocity = velocity * (1.0 - std::min(air_resistance, 0.99));
        }

        double max_speed = 150.0;
        if (velocity.length() > max_speed) {
            velocity = velocity * (max_speed / velocity.length());
        }

        position += velocity;
        acceleration = Vector(0, 0, 0);

        handle_wall_collisions(bounds_width, bounds_height);
        if (!is_fluid && !is_gas && !is_ice) update_trail();
    }

    void update_trail() {
        trail.push_back(position.to_qpointf());
        if (trail.size() > 12) trail.erase(trail.begin());
    }

private:
    void handle_wall_collisions(double width, double height) {
        const double e = effective_restitution();
        if (position.y + radius >= height) {
            position.y = height - radius;
            velocity.y = -std::abs(velocity.y) * e;
            if (!is_gas) velocity.x *= 0.98;
        } else if (position.y - radius <= 0) {
            position.y = radius;
            velocity.y = std::abs(velocity.y) * e;
        }

        if (position.x + radius >= width) {
            position.x = width - radius;
            velocity.x = -std::abs(velocity.x) * e;
        } else if (position.x - radius <= 0) {
            position.x = radius;
            velocity.x = std::abs(velocity.x) * e;
        }
    }
};

class Spring {
public:
    PhysicsObject* obj_a;
    PhysicsObject* obj_b;
    double length;
    double stiffness;
    double damping;

    Spring(PhysicsObject* a, PhysicsObject* b, double l, double s = 0.1, double d = 0.05)
        : obj_a(a), obj_b(b), length(l), stiffness(s), damping(d) {}

    void update() {
        Vector delta_pos = obj_a->position - obj_b->position;
        double distance = delta_pos.length();
        if (distance == 0) return;

        double force_mag = (distance - length) * stiffness;
        Vector force_dir = delta_pos * (1.0 / distance);
        Vector rel_vel = obj_a->velocity - obj_b->velocity;
        Vector damping_force = force_dir * (rel_vel.dot(force_dir) * damping);
        Vector total_force = (force_dir * force_mag) + damping_force;

        if (!obj_a->is_static) obj_a->apply_force(total_force * -1.0);
        if (!obj_b->is_static) obj_b->apply_force(total_force);
    }
};

class RigidLink {
public:
    PhysicsObject* obj_a;
    PhysicsObject* obj_b;
    double length;

    RigidLink(PhysicsObject* a, PhysicsObject* b, double l)
        : obj_a(a), obj_b(b), length(l) {}

    void update() {
        Vector delta = obj_b->position - obj_a->position;
        double dist = delta.length();
        if (dist == 0) return;

        double error = dist - length;
        Vector direction = delta * (1.0 / dist);

        double inv_mass_a = obj_a->is_static ? 0 : 1.0 / obj_a->mass;
        double inv_mass_b = obj_b->is_static ? 0 : 1.0 / obj_b->mass;
        double sum_mass = inv_mass_a + inv_mass_b;
        if (sum_mass == 0) return;

        double correction = error / sum_mass;

        if (!obj_a->is_static) obj_a->position += direction * (correction * inv_mass_a);
        if (!obj_b->is_static) obj_b->position -= direction * (correction * inv_mass_b);
    }
};

class Pulley {
public:
    PhysicsObject* obj_a;
    PhysicsObject* obj_b;
    Vector anchor;
    double total_length;
    double stiffness;

    Pulley(PhysicsObject* a, PhysicsObject* b, Vector anc, double s = 0.5)
        : obj_a(a), obj_b(b), anchor(anc), stiffness(s) {
        total_length = (a->position - anchor).length() + (b->position - anchor).length();
    }

    void update() {
        double dist_a = (obj_a->position - anchor).length();
        double dist_b = (obj_b->position - anchor).length();
        double current_total = dist_a + dist_b;
        if (current_total == 0) return;

        double diff = current_total - total_length;
        if (diff > 0) {
            double force_mag = diff * stiffness;
            Vector dir_a = dist_a > 0 ? (anchor - obj_a->position) * (1.0 / dist_a) : Vector(0,0);
            Vector dir_b = dist_b > 0 ? (anchor - obj_b->position) * (1.0 / dist_b) : Vector(0,0);
            if (!obj_a->is_static) obj_a->apply_force(dir_a * force_mag);
            if (!obj_b->is_static) obj_b->apply_force(dir_b * force_mag);
        }
    }
};

// A pneumatic cylinder (gas strut).  A sealed air charge balances atmospheric
// pressure at the rest length.  Its force follows the ideal-gas law
// P ∝ T / L, the charge heats on fast compression and cools on expansion
// (adiabatic, γ = 1.4) and relaxes towards its surroundings' temperature, so
// a hot piston extends and a cold one retracts.  A dashpot damps motion and
// the rod has hard end stops.  Forces are mass-weighted so momentum is
// conserved and light objects are not flung out.
class PistonLink {
public:
    PhysicsObject* obj_a;
    PhysicsObject* obj_b;
    double min_len;
    double max_len;
    double rest_length;
    double charge_temperature;  // °C at which the charge balances at rest_length
    double gas_temperature;     // current °C of the charge
    double stiffness;
    double damping;
    double previous_length;

    PistonLink(PhysicsObject* a, PhysicsObject* b, double min_l, double max_l, double rest,
               double temperature, double s = 0.12, double d = 0.10)
        : obj_a(a), obj_b(b), min_len(min_l), max_len(max_l), rest_length(rest),
          charge_temperature(temperature), gas_temperature(temperature),
          stiffness(s), damping(d), previous_length(rest) {}

    double gauge_pressure_ratio(double length) const {
        return thermo::kelvin(gas_temperature) / thermo::kelvin(charge_temperature)
             * rest_length / std::max(1.0, length) - 1.0;
    }

    void update(double ambient_temperature) {
        Vector delta = obj_a->position - obj_b->position;
        double dist = delta.length();
        if (dist < 1e-6) return;
        Vector dir = delta * (1.0 / dist);

        const double inv_a = obj_a->is_static ? 0.0 : 1.0 / obj_a->mass;
        const double inv_b = obj_b->is_static ? 0.0 : 1.0 / obj_b->mass;
        if (inv_a + inv_b == 0.0) return;
        const double reduced_mass = 1.0 / (inv_a + inv_b);

        // Adiabatic compression of the charge: T·L^(γ-1) = const.
        const double gas_k = thermo::kelvin(gas_temperature) * std::pow(previous_length / std::max(1.0, dist), 0.4);
        gas_temperature = gas_k + thermo::kAbsoluteZero;
        previous_length = dist;
        // Heat exchange through the cylinder wall.
        const double surroundings = 0.5 * ambient_temperature + 0.25 * (obj_a->temperature + obj_b->temperature);
        gas_temperature += (surroundings - gas_temperature) * 0.02;

        const double rel_vel = (obj_a->velocity - obj_b->velocity).dot(dir);
        const double rel_accel = stiffness * rest_length * gauge_pressure_ratio(dist) - damping * rel_vel;
        const double force = reduced_mass * rel_accel;
        obj_a->apply_force(dir * (force * inv_a));
        obj_b->apply_force(dir * (-force * inv_b));

        // Hard end stops: positional correction + inelastic impact.
        if (dist < min_len || dist > max_len) {
            const double target = dist < min_len ? min_len : max_len;
            const double error = dist - target;
            obj_a->position -= dir * (error * inv_a / (inv_a + inv_b));
            obj_b->position += dir * (error * inv_b / (inv_a + inv_b));
            if ((dist < min_len && rel_vel < 0.0) || (dist > max_len && rel_vel > 0.0)) {
                const double impulse = -(1.0 + 0.2) * rel_vel * reduced_mass;
                obj_a->velocity += dir * (impulse * inv_a);
                obj_b->velocity -= dir * (impulse * inv_b);
            }
        }
    }
};

// A sealed vertical cylinder with analytic walls and a massive, vertically
// guided piston.  Wall collisions are swept against each object's previous
// side, so nothing can leak in or out, and the space above the piston is open
// to the room (objects can rest on the piston and weigh it down).
class GasContainer {
public:
    double left, right, bottom, wall_top;
    double piston_min_y, piston_max_y;
    PhysicsObject* piston = nullptr;
    double pressure = thermo::kAtmosphere;
    double average_temperature = 20.0;
    double gas_volume = 0.0;
    double previous_chamber_height = -1.0;
    double piston_start_y = 0.0;
    double boiling_point = 100.0;
    int gas_particles = 0;
    int liquid_particles = 0;

    static constexpr double kWallHalfThickness = 3.0;

    GasContainer(double x, double y, double half_width = 110.0, double half_height = 90.0)
        : left(x - half_width), right(x + half_width), bottom(y + half_height),
          wall_top(y - half_height - 12.0),
          piston_min_y(y - half_height), piston_max_y(y + half_height - 30.0) {}

    double top() const { return piston ? piston->position.y : piston_min_y; }
    double width() const { return right - left; }
    double chamber_height() const { return std::max(1.0, bottom - top()); }
    bool in_chamber(const Vector& p) const {
        return p.x > left && p.x < right && p.y > top() - 0.5 && p.y < bottom;
    }
};

class PhysicsCanvas : public QWidget {
    Q_OBJECT
public:
    Vector gravity = Vector(0.0, 0.3, 0.0);
    double air_resistance = 0.01;
    double elasticity = 0.8;
    double ambient_temperature = 20.0;  // room air, a finite thermal reservoir
    double ambient_setpoint = 20.0;     // what the building/HVAC pulls the room toward
    double heat_transfer_rate = 0.05;
    double fluid_viscosity = 0.05;

    // ---- Thermal tuning (sim units, see thermo namespace) ----
    static constexpr double kAirHeatCapacity      = 1500.0; // room air thermal mass
    static constexpr double kRoomLeak             = 2.0;    // room ↔ building conductance
    static constexpr double kSourceAirConductance = 0.06;   // heater/cooler → room air
    static constexpr double kSourceObjectConductance = 0.004; // per px of target radius
    static constexpr double kConvection           = 0.00075; // per px of radius, per K
    static constexpr double kRadiation            = 1.05e-12; // per px of radius, per K⁴
    static constexpr double kEvaporation          = 0.05;   // per px of radius at P_sat = 1 atm
    // ---- Gas vessel tuning ----
    static constexpr double kGasPressureConstant  = 40.0;   // kPa·px² per particle per K
    static constexpr double kPistonForceScale     = 0.0073; // kPa·px → sim force
    static constexpr double kPistonDamping        = 0.08;
    // ---- Liquid water: Position Based Fluids (Macklin & Müller 2013) ----
    // PBF solves the incompressible Navier–Stokes equations on particles:
    // the pressure term is an incompressibility constraint (rho_i = rho_0)
    // solved by Jacobi iterations, viscosity is XSPH smoothing, gravity and
    // thermal buoyancy are external forces, and surface tension is a
    // cohesion force (Akinci et al. 2013).
    static constexpr double kFluidSpacing    = 8.0;   // rest spacing of water particles (px)
    static constexpr double kFluidH          = 20.0;  // smoothing length (2.5 × spacing)
    static constexpr int    kFluidMinIterations = 2;
    static constexpr int    kFluidMaxIterations = 8;
    static constexpr double kFluidRelaxation = 0.05;  // constraint-force mixing (fraction of stiffness)
    static constexpr double kFluidAntiCluster = 0.01; // artificial pressure (tensile instability fix)
    static constexpr double kFluidCohesion   = 0.012; // surface tension at 20 °C
    double fluid_rest_density = 0.0;                  // kernel density of the rest lattice at 20 °C
    double fluid_gradient_norm = 0.0;                 // Σ|∇C|² of the rest lattice

    std::vector<std::shared_ptr<PhysicsObject>> balls;
    std::vector<Spring> springs;
    std::vector<Pulley> pulleys;
    std::vector<RigidLink> rigid_links;
    std::vector<PistonLink> pistons;
    std::vector<GasContainer> containers;

    PhysicsObject* dragged_ball = nullptr;
    QPointF last_mouse_pos;
    bool has_last_mouse = false;
    Vector mouse_velocity = Vector(0,0,0);
    PhysicsObject* connection_start_obj = nullptr;

    ObjectType active_tool = ObjectType::BALL;
    bool is_paused = false;
    std::vector<QPointF> drawing_points;
    QTimer timer;

    PhysicsCanvas(QWidget* parent = nullptr) : QWidget(parent) {
        init_fluid_constants();
        connect(&timer, &QTimer::timeout, this, &PhysicsCanvas::game_loop);
        timer.start(16);
    }

    void restart(int num_balls) {
        static_cast<void>(num_balls);
        balls.clear(); springs.clear(); pulleys.clear(); rigid_links.clear(); pistons.clear(); containers.clear();
        dragged_ball = nullptr; connection_start_obj = nullptr;
        ambient_temperature = ambient_setpoint;
        update();
    }

    void set_tool(ObjectType tool_type) { active_tool = tool_type; }

    void remove_object(PhysicsObject* obj) {
        if (dragged_ball == obj) dragged_ball = nullptr;
        if (connection_start_obj == obj) connection_start_obj = nullptr;

        springs.erase(std::remove_if(springs.begin(), springs.end(),
            [obj](const Spring& s) { return s.obj_a == obj || s.obj_b == obj; }), springs.end());

        rigid_links.erase(std::remove_if(rigid_links.begin(), rigid_links.end(),
            [obj](const RigidLink& l) { return l.obj_a == obj || l.obj_b == obj; }), rigid_links.end());

        pulleys.erase(std::remove_if(pulleys.begin(), pulleys.end(),
            [obj](const Pulley& p) { return p.obj_a == obj || p.obj_b == obj; }), pulleys.end());

        pistons.erase(std::remove_if(pistons.begin(), pistons.end(),
            [obj](const PistonLink& p) { return p.obj_a == obj || p.obj_b == obj; }), pistons.end());

        containers.erase(std::remove_if(containers.begin(), containers.end(),
            [obj](const GasContainer& c) { return c.piston == obj; }), containers.end());

        balls.erase(std::remove_if(balls.begin(), balls.end(),
            [obj](const std::shared_ptr<PhysicsObject>& b) { return b.get() == obj; }), balls.end());
    }

    PhysicsObject* spawn_at(double x, double y, ShapeType shape, double radius=18.0, double mass=-1.0,
                            bool is_static=false, bool is_fluid=false) {
        auto obj = std::make_shared<PhysicsObject>(x, y, radius, elasticity, mass, QColor(), shape, is_static, is_fluid);
        if (is_fluid) {
            obj->elasticity = 0.1;
            obj->is_bouncy = false;
        }
        obj->temperature = ambient_temperature;
        balls.push_back(obj);
        return obj.get();
    }

    PhysicsObject* spawn_gas_particle(double x, double y) {
        auto node = spawn_at(x, y, ShapeType::CIRCLE, 1.5, 0.5);
        node->is_gas = true;
        node->is_bouncy = false;
        node->elasticity = 1.0;
        node->molar_ratio = 1.0;
        node->color = QColor(150, 255, 100, 150);
        const double sigma = thermo::kGasSpeedScale * std::sqrt(thermo::kelvin(node->temperature)) / std::sqrt(2.0);
        node->velocity = Vector(sigma * random_normal(), sigma * random_normal());
        return node;
    }

    void spawn_container(double x, double y, double half_width = 110.0, double half_height = 90.0) {
        containers.emplace_back(x, y, half_width, half_height);
        GasContainer& container = containers.back();
        // The vessel is sealed with a charge of room air at room temperature.
        // Start the piston near the height where that charge supports it.
        constexpr int air_particles = 150;
        const double piston_weight_kpa = 80.0 * gravity.y / (container.width() * kPistonForceScale);
        const double target_pressure = thermo::kAtmosphere + piston_weight_kpa;
        const double eq_height = kGasPressureConstant * air_particles * thermo::kelvin(ambient_temperature)
                               / (target_pressure * container.width());
        const double start_y = clampd(container.bottom - eq_height, container.piston_min_y, container.piston_max_y);

        PhysicsObject* face = spawn_at(x, start_y, ShapeType::SQUARE, 8.0, 80.0);
        face->is_container_piston = true;
        face->is_bouncy = false;
        face->elasticity = 0.05;
        face->color = QColor(255, 140, 50);
        face->velocity = Vector(0, 0, 0);
        container.piston = face;
        container.piston_start_y = start_y;

        for (int i = 0; i < air_particles; ++i) {
            spawn_gas_particle(random_double(container.left + 5, container.right - 5),
                               random_double(start_y + 5, container.bottom - 5));
        }
    }

protected:
    void mousePressEvent(QMouseEvent* event) override {
        QPointF pos = event->position();
        Vector v_pos(pos.x(), pos.y());
        last_mouse_pos = pos;
        has_last_mouse = true;

        // The whole piston bar of a vessel is grabbable / removable.
        PhysicsObject* clicked_piston = nullptr;
        for (const auto& c : containers) {
            if (c.piston && pos.x() >= c.left && pos.x() <= c.right && std::abs(pos.y() - c.top()) < 8.0) {
                clicked_piston = c.piston;
                break;
            }
        }

        if (event->button() == Qt::RightButton) {
            if (active_tool == ObjectType::POLYGON && !drawing_points.empty()) {
                finalize_polygon();
                return;
            }
            if (clicked_piston) { remove_object(clicked_piston); return; }
            for (auto it = balls.rbegin(); it != balls.rend(); ++it) {
                if ((v_pos - (*it)->position).length() <= (*it)->radius) {
                    remove_object((*it).get());
                    break;
                }
            }
            return;
        }

        if (event->button() == Qt::LeftButton) {
            if (active_tool == ObjectType::POLYGON) {
                if (!drawing_points.empty() &&
                    Vector(pos.x() - drawing_points[0].x(), pos.y() - drawing_points[0].y()).length() < 15) {
                    finalize_polygon();
                } else {
                    drawing_points.push_back(pos);
                }
                return;
            }

            PhysicsObject* clicked_obj = clicked_piston;
            if (!clicked_obj) {
                for (auto it = balls.rbegin(); it != balls.rend(); ++it) {
                    if ((v_pos - (*it)->position).length() <= (*it)->radius) {
                        clicked_obj = (*it).get();
                        break;
                    }
                }
            }

            if (clicked_obj) {
                if (active_tool == ObjectType::SPRING || active_tool == ObjectType::PULLEY || active_tool == ObjectType::PISTON) {
                    connection_start_obj = clicked_obj;
                } else {
                    dragged_ball = clicked_obj;
                    clicked_obj->is_dragged = true;
                    clicked_obj->velocity = Vector(0,0,0);
                }
            } else {
                handle_spawning(pos.x(), pos.y());
            }
        }
    }

    void mouseMoveEvent(QMouseEvent* event) override {
        QPointF pos = event->position();
        if (has_last_mouse) {
            mouse_velocity = Vector(pos.x() - last_mouse_pos.x(), pos.y() - last_mouse_pos.y());
        }
        last_mouse_pos = pos;
        has_last_mouse = true;

        if (dragged_ball) {
            dragged_ball->position.x = pos.x();
            dragged_ball->position.y = pos.y();
            dragged_ball->velocity = Vector(0,0,0);
        }
    }

    void mouseReleaseEvent(QMouseEvent* event) override {
        if (event->button() == Qt::LeftButton) {
            if (dragged_ball) {
                dragged_ball->is_dragged = false;
                if (!is_paused) dragged_ball->velocity = mouse_velocity * 0.8;
                dragged_ball = nullptr;
            } else if (connection_start_obj) {
                QPointF pos = event->position();

                for (auto it = balls.rbegin(); it != balls.rend(); ++it) {
                    PhysicsObject* target_ball = (*it).get();

                    if (target_ball != connection_start_obj && (Vector(pos.x(), pos.y()) - target_ball->position).length() <= target_ball->radius) {

                        if (active_tool == ObjectType::SPRING) {
                            double dist = (connection_start_obj->position - target_ball->position).length();
                            springs.emplace_back(connection_start_obj, target_ball, std::max(dist, 10.0));

                        } else if (active_tool == ObjectType::PISTON) {
                            double dist = std::max(20.0, (connection_start_obj->position - target_ball->position).length());
                            pistons.emplace_back(connection_start_obj, target_ball, dist * 0.55, dist * 1.6, dist,
                                                 ambient_temperature);

                        } else if (active_tool == ObjectType::PULLEY) {
                            double anchor_x = (connection_start_obj->position.x + target_ball->position.x) / 2.0;
                            double anchor_y = std::min(connection_start_obj->position.y, target_ball->position.y) - 100.0;
                            pulleys.emplace_back(connection_start_obj, target_ball, Vector(anchor_x, std::max(10.0, anchor_y)));
                        }
                        break;
                    }
                }
                connection_start_obj = nullptr;
            }
            has_last_mouse = false;
        }
    }

    // Colour shift that makes temperature visible: hot → red/orange, cold → blue.
    static QColor thermal_tint(const QColor& base, double temperature) {
        double r = base.red(), g = base.green(), b = base.blue();
        if (temperature > 25.0) {
            const double k = clampd((temperature - 25.0) / 250.0, 0.0, 0.85);
            r += (255 - r) * k; g += (90 - g) * k; b += (40 - b) * k;
        } else if (temperature < 10.0) {
            const double k = clampd((10.0 - temperature) / 120.0, 0.0, 0.8);
            r += (120 - r) * k; g += (200 - g) * k; b += (255 - b) * k;
        }
        return QColor(int(r), int(g), int(b), base.alpha());
    }

    void paintEvent(QPaintEvent* event) override {
        static_cast<void>(event);
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.fillRect(rect(), QColor(30, 32, 38));

        if (!drawing_points.empty()) {
            painter.setPen(QPen(QColor(100, 255, 100), 2, Qt::DashLine));
            for (size_t i = 0; i + 1 < drawing_points.size(); ++i) {
                painter.drawLine(drawing_points[i], drawing_points[i+1]);
            }
            if (has_last_mouse) painter.drawLine(drawing_points.back(), last_mouse_pos);
            painter.setBrush(QBrush(QColor(100, 255, 100)));
            painter.drawEllipse(drawing_points[0], 5, 5);
        }

        painter.setPen(QPen(QColor(200, 200, 200, 150), 2));
        for (const auto& p : pulleys) {
            if (is_valid(p.obj_a->position) && is_valid(p.obj_b->position)) {
                painter.drawLine(p.obj_a->position.to_qpointf(), p.anchor.to_qpointf());
                painter.drawLine(p.obj_b->position.to_qpointf(), p.anchor.to_qpointf());
                painter.setBrush(QBrush(QColor(255, 255, 255)));
                painter.drawEllipse(p.anchor.to_qpointf(), 5, 5);
            }
        }

        painter.setPen(QPen(QColor(150, 150, 150), 3));
        for (const auto& s : springs) {
            if (is_valid(s.obj_a->position) && is_valid(s.obj_b->position))
                painter.drawLine(s.obj_a->position.to_qpointf(), s.obj_b->position.to_qpointf());
        }

        painter.setPen(QPen(QColor(180, 150, 100), 4));
        for (const auto& l : rigid_links) {
            if (is_valid(l.obj_a->position) && is_valid(l.obj_b->position))
                painter.drawLine(l.obj_a->position.to_qpointf(), l.obj_b->position.to_qpointf());
        }

        // Pistons: a thick barrel anchored on obj_b and a thinner rod to obj_a.
        for (const auto& l : pistons) {
            if (!is_valid(l.obj_a->position) || !is_valid(l.obj_b->position)) continue;
            Vector delta = l.obj_a->position - l.obj_b->position;
            double dist = delta.length();
            if (dist < 1e-6) continue;
            Vector dir = delta / dist;
            Vector barrel_end = l.obj_b->position + dir * std::min(dist, l.min_len);
            painter.setPen(QPen(QColor(255, 140, 50), 3));
            painter.drawLine(l.obj_a->position.to_qpointf(), l.obj_b->position.to_qpointf());
            painter.setPen(QPen(thermal_tint(QColor(120, 125, 135), l.gas_temperature), 10, Qt::SolidLine, Qt::FlatCap));
            painter.drawLine(l.obj_b->position.to_qpointf(), barrel_end.to_qpointf());
        }

        // Vessels: fixed cylinder walls, tinted gas chamber and guided piston.
        for (const auto& c : containers) {
            QColor chamber = thermal_tint(QColor(80, 110, 90), c.average_temperature);
            chamber.setAlpha(45);
            painter.setPen(Qt::NoPen);
            painter.setBrush(chamber);
            painter.drawRect(QRectF(c.left, c.top(), c.width(), c.bottom - c.top()));

            painter.setPen(QPen(QColor(110, 125, 140), GasContainer::kWallHalfThickness * 2));
            painter.setBrush(Qt::NoBrush);
            painter.drawLine(QPointF(c.left, c.wall_top), QPointF(c.left, c.bottom));
            painter.drawLine(QPointF(c.right, c.wall_top), QPointF(c.right, c.bottom));
            painter.drawLine(QPointF(c.left, c.bottom), QPointF(c.right, c.bottom));
            painter.setPen(QPen(QColor(255, 140, 50), 8, Qt::SolidLine, Qt::FlatCap));
            painter.drawLine(QPointF(c.left + 3, c.top()), QPointF(c.right - 3, c.top()));
            painter.setPen(QPen(QColor(200, 110, 40), 4));
            painter.drawLine(QPointF((c.left + c.right) * 0.5, c.top()), QPointF((c.left + c.right) * 0.5, c.top() - 18));
            painter.setPen(QColor(245, 220, 150));
            painter.setFont(QFont("Consolas", 9, QFont::Bold));
            painter.drawText(QPointF(c.left + 6, c.top() + 17),
                QString("%1 kPa  %2 °C").arg(c.pressure, 0, 'f', 1).arg(c.average_temperature, 0, 'f', 1));
        }

        if (connection_start_obj && has_last_mouse) {
            painter.setPen(QPen(QColor(100, 200, 255, 200), 2, Qt::DashLine));
            painter.drawLine(connection_start_obj->position.to_qpointf(), last_mouse_pos);
        }

        for (const auto& obj_ptr : balls) {
            PhysicsObject* obj = obj_ptr.get();
            if (!is_valid(obj->position) || !is_valid(obj->radius)) continue;

            if (obj->is_container_piston) continue; // rendered as the vessel's piston bar

            if (obj->trail.size() > 2 && !obj->is_static && !obj->is_fluid && !obj->is_gas && !obj->is_ice) {
                QPen t_pen(obj->color);
                for (size_t i = 1; i < obj->trail.size(); ++i) {
                    int opacity = int(255 * (float(i) / obj->trail.size()));
                    t_pen.setColor(QColor(obj->color.red(), obj->color.green(), obj->color.blue(), opacity));
                    t_pen.setWidth(std::max(2, int((obj->radius/2.0) * (float(i)/obj->trail.size()))));
                    painter.setPen(t_pen);
                    painter.drawLine(obj->trail[i-1], obj->trail[i]);
                }
            }

            if (obj->fuse > 0) {
                if (obj->fuse % 10 < 5) painter.setPen(QPen(QColor(255, 255, 255), 4));
                else painter.setPen(QPen(QColor(255, 50, 50), 4));
            } else if (obj->is_static && !obj->is_ice) painter.setPen(QPen(QColor(255, 255, 255), 2));
            else if (obj->is_motor) painter.setPen(QPen(QColor(255, 255, 100), 3));
            else if (!obj->is_gas && !obj->is_fluid && !obj->is_ice && std::abs(obj->temperature - 20.0) > 15.0) {
                // Temperature ring on solids, so the temperature-dependent bounce is readable.
                const bool hot = obj->temperature > 20.0;
                const int alpha = int(clampd(std::abs(obj->temperature - 20.0) * 1.5, 40.0, 255.0));
                painter.setPen(QPen(hot ? QColor(255, 110, 40, alpha) : QColor(120, 220, 255, alpha), 3));
            }
            else painter.setPen(Qt::NoPen);

            QColor fill = obj->color;
            if (obj->is_gas || obj->is_fluid) fill = thermal_tint(obj->color, obj->temperature);
            painter.setBrush(QBrush(fill));

            if (obj->shape_type == ShapeType::CIRCLE) {
                painter.drawEllipse(obj->position.to_qpointf(), obj->radius, obj->radius);
            } else if (obj->shape_type == ShapeType::SQUARE) {
                double r = obj->radius;
                painter.drawRect(QRectF(obj->position.x - r, obj->position.y - r, r * 2, r * 2));
            }
        }

        // ---- Statistics HUD ----
        struct Stat { int n = 0; double sum_t = 0.0; double avg() const { return n ? sum_t / n : 0.0; } };
        Stat gas, steam, liquid, ice;
        double min_x = width(), max_x = 0, min_y = height(), max_y = 0;
        for (const auto& obj_ptr : balls) {
            const PhysicsObject* o = obj_ptr.get();
            if (o->is_gas) {
                ++gas.n; gas.sum_t += o->temperature;
                if (o->is_steam) { ++steam.n; steam.sum_t += o->temperature; }
                if (o->container_index < 0) {
                    min_x = std::min(min_x, o->position.x); max_x = std::max(max_x, o->position.x);
                    min_y = std::min(min_y, o->position.y); max_y = std::max(max_y, o->position.y);
                }
            } else if (o->is_fluid) { ++liquid.n; liquid.sum_t += o->temperature; }
            else if (o->is_ice) { ++ice.n; ice.sum_t += o->temperature; }
        }

        // Free gas is at atmospheric pressure; outline the free cloud.
        if (max_x > min_x && max_y > min_y) {
            painter.setPen(QPen(QColor(150, 255, 100, 80), 2, Qt::DashLine));
            painter.setBrush(Qt::NoBrush);
            painter.drawRect(QRectF(min_x - 10, min_y - 10, (max_x - min_x) + 20, (max_y - min_y) + 20));
        }

        auto temp_color = [](double t) {
            if (t > 50.0) return QColor(255, 100, 100);
            if (t < 0.0) return QColor(100, 240, 255);
            return QColor(255, 255, 255);
        };
        std::vector<std::pair<QString, QColor>> lines;
        if (gas.n > 0) {
            lines.push_back({QString("Gas particles : %1 (steam %2)").arg(gas.n).arg(steam.n), Qt::white});
            lines.push_back({QString("Gas avg T     : %1 °C").arg(gas.avg(), 0, 'f', 1), temp_color(gas.avg())});
        }
        if (liquid.n > 0)
            lines.push_back({QString("Water         : %1 @ %2 °C").arg(liquid.n).arg(liquid.avg(), 0, 'f', 1), temp_color(liquid.avg())});
        if (ice.n > 0)
            lines.push_back({QString("Ice           : %1 @ %2 °C").arg(ice.n).arg(ice.avg(), 0, 'f', 1), QColor(100, 240, 255)});
        for (size_t i = 0; i < containers.size(); ++i) {
            const auto& c = containers[i];
            lines.push_back({QString("Vessel %1 P    : %2 kPa").arg(i + 1).arg(c.pressure, 0, 'f', 1),
                             c.pressure > 200.0 ? QColor(255, 150, 50) : QColor(255, 255, 255)});
            lines.push_back({QString("  V %1 m³  T %2 °C").arg(c.gas_volume / 1000.0, 0, 'f', 1)
                                 .arg(c.average_temperature, 0, 'f', 1), temp_color(c.average_temperature)});
            if (c.liquid_particles > 0)
                lines.push_back({QString("  boils at %1 °C").arg(c.boiling_point, 0, 'f', 1), QColor(150, 200, 255)});
        }
        if (!lines.empty()) {
            painter.setBrush(QColor(20, 22, 28, 220));
            painter.setPen(QPen(QColor(100, 150, 255), 1));
            painter.drawRoundedRect(10, 10, 300, 14 + 22 * int(lines.size()), 8, 8);
            painter.setFont(QFont("Consolas", 11, QFont::Bold));
            for (size_t i = 0; i < lines.size(); ++i) {
                painter.setPen(lines[i].second);
                painter.drawText(20, 32 + 22 * int(i), lines[i].first);
            }
        }

        // Room air temperature (driven by heaters, coolers and every object).
        painter.setBrush(QColor(20, 22, 28, 220));
        painter.setPen(QPen(QColor(100, 150, 255), 1));
        painter.drawRoundedRect(10, height() - 40, 340, 30, 8, 8);
        painter.setPen(temp_color(ambient_temperature));
        painter.setFont(QFont("Consolas", 11, QFont::Bold));
        painter.drawText(20, height() - 20, QString("Room Air : %1 °C  (setpoint %2)")
            .arg(ambient_temperature, 0, 'f', 1).arg(ambient_setpoint, 0, 'f', 0));
    }

private:
    // Heat flow coefficient between two bodies, limited so that a single
    // exchange never overshoots equilibrium even with many neighbours.
    static double limited_coupling(double coupling, const PhysicsObject& a, const PhysicsObject& b) {
        const bool a_inf = a.is_heater || a.is_cooler;
        const bool b_inf = b.is_heater || b.is_cooler;
        if (a_inf && b_inf) return 0.0;
        double reduced;
        if (a_inf) reduced = b.thermal_mass();
        else if (b_inf) reduced = a.thermal_mass();
        else reduced = a.thermal_mass() * b.thermal_mass() / (a.thermal_mass() + b.thermal_mass());
        return std::min(coupling, 0.04 * reduced);
    }

    static double contact_conductance(const PhysicsObject& a, const PhysicsObject& b) {
        if (a.is_gas && b.is_gas) return 1.0; // molecular mixing
        const double ka = a.conductivity(), kb = b.conductivity();
        return 2.0 * ka * kb / (ka + kb);       // series (harmonic-mean) conduction
    }

    void update_thermal_environment() {
        // The room air is a finite reservoir.  It receives the convective /
        // radiant output of heaters and coolers, sensible heat from every
        // object (gases mix with it directly), and slowly leaks to the
        // building setpoint.  Its temperature is therefore *driven* by the
        // sources and the gases rather than imposed.
        const double rate_scale = heat_transfer_rate / 0.05;
        const double t_air = ambient_temperature;
        double air_energy = (ambient_setpoint - t_air) * kRoomLeak;

        for (const auto& source_ptr : balls) {
            PhysicsObject* source = source_ptr.get();
            if (!source->is_heater && !source->is_cooler) continue;
            // Thermostatic plates: held at their temperature by external power.
            air_energy += (source->temperature - t_air) * kSourceAirConductance;
            for (const auto& target_ptr : balls) {
                PhysicsObject* target = target_ptr.get();
                if (target == source || target->is_heater || target->is_cooler || target->is_container_piston) continue;
                const double distance = (target->position - source->position).length();
                const double reach = source->radius + target->radius + 70.0;
                if (distance >= reach) continue;
                const double proximity = 1.0 - distance / reach;
                const double diff = source->temperature - target->temperature;
                double coupling = kSourceObjectConductance * target->radius * (0.1 + 0.9 * proximity * proximity);
                coupling = std::min(coupling, 0.5 * target->thermal_mass());
                target->add_heat(diff * coupling);
            }
        }

        for (const auto& object_ptr : balls) {
            PhysicsObject* object = object_ptr.get();
            if (object->is_heater || object->is_cooler || object->is_container_piston) continue;
            const double diff = object->temperature - t_air;

            // Newton convection ∝ surface (perimeter in 2D).  Free gas mixes
            // with room air; gas sealed in a vessel only conducts through the wall.
            double factor = 1.0;
            if (object->is_gas) factor = object->container_index >= 0 ? 0.25 : 3.0;
            double coupling = kConvection * object->radius * factor * rate_scale;
            coupling = std::min(coupling, 0.5 * object->thermal_mass());
            double q = diff * coupling;

            // Stefan–Boltzmann radiation (condensed matter only; air is transparent).
            if (!object->is_gas) {
                const double tk = thermo::kelvin(object->temperature), ta = thermo::kelvin(t_air);
                double q_rad = kRadiation * object->radius * rate_scale * (tk * tk * tk * tk - ta * ta * ta * ta);
                const double cap = std::abs(diff) * 0.5 * object->thermal_mass();
                q += clampd(q_rad, -cap, cap);
            }
            object->add_heat(-q);
            air_energy += q;

            // Evaporative cooling of exposed liquid water: rate ∝ P_sat(T).
            // The latent heat leaves as humidity, not as sensible air heat.
            if (object->is_fluid && object->container_index < 0) {
                const double ratio = thermo::saturation_pressure(object->temperature) / thermo::kAtmosphere;
                object->add_heat(-kEvaporation * object->radius * std::min(ratio, 1.0) * rate_scale);
            }
        }
        ambient_temperature = clampd(t_air + air_energy / kAirHeatCapacity, -200.0, 700.0);
    }

    // Classifies objects into vessels, computes each vessel's pressure from the
    // ideal-gas law (P = N k T / V), applies adiabatic heating/cooling when the
    // piston moves, and drives the piston.
    void update_containers() {
        for (const auto& object_ptr : balls) {
            object_ptr->container_index = -1;
            object_ptr->local_pressure = thermo::kAtmosphere;
        }

        for (size_t i = 0; i < containers.size(); ++i) {
            GasContainer& c = containers[i];
            if (!c.piston) continue;
            c.piston_start_y = c.top();

            std::vector<PhysicsObject*> gas_inside;
            double condensed_area = 0.0;
            double sum_tk = 0.0;
            c.liquid_particles = 0;
            for (const auto& object_ptr : balls) {
                PhysicsObject* o = object_ptr.get();
                if (o->is_container_piston || o->container_index >= 0 || !c.in_chamber(o->position)) continue;
                o->container_index = int(i);
                if (o->is_gas) { gas_inside.push_back(o); sum_tk += thermo::kelvin(o->temperature); }
                else if (o->is_fluid || o->is_ice) { condensed_area += 2.0 * o->radius * o->radius; ++c.liquid_particles; }
                else condensed_area += 3.14159265358979 * o->radius * o->radius;
            }
            c.gas_particles = int(gas_inside.size());

            const double chamber_area = c.width() * c.chamber_height();
            c.gas_volume = std::max(600.0, chamber_area - condensed_area);

            // Adiabatic process: T·V^(γ-1) = const for the piston's displacement.
            if (c.previous_chamber_height > 0.0 && !gas_inside.empty()) {
                const double prev_volume = std::max(600.0, c.width() * c.previous_chamber_height - condensed_area);
                const double ratio = prev_volume / c.gas_volume;
                if (std::abs(ratio - 1.0) > 1e-9) {
                    sum_tk = 0.0;
                    for (PhysicsObject* g : gas_inside) {
                        const double tk = thermo::kelvin(g->temperature);
                        const double new_tk = tk * std::pow(ratio, g->heat_capacity_ratio() - 1.0);
                        g->add_heat((new_tk - tk) * g->thermal_mass());
                        sum_tk += thermo::kelvin(g->temperature);
                    }
                }
            }
            c.previous_chamber_height = c.chamber_height();

            if (!gas_inside.empty()) {
                const double avg_tk = sum_tk / gas_inside.size();
                c.average_temperature = avg_tk + thermo::kAbsoluteZero;
                c.pressure = kGasPressureConstant * gas_inside.size() * avg_tk / c.gas_volume;
            } else {
                c.average_temperature = ambient_temperature;
                c.pressure = 0.0; // evacuated: the atmosphere pushes the piston down
            }
            c.boiling_point = thermo::boiling_point(c.pressure);
            for (const auto& object_ptr : balls) {
                if (object_ptr->container_index == int(i)) object_ptr->local_pressure = std::max(c.pressure, 0.62);
            }

            // Piston dynamics: gas pressure below vs. atmosphere above, plus its
            // own weight (gravity, applied in update) and a guide-friction dashpot.
            PhysicsObject* face = c.piston;
            const double pressure_force = (thermo::kAtmosphere - c.pressure) * c.width() * kPistonForceScale;
            face->apply_force(Vector(0.0, pressure_force / face->mass - face->velocity.y * kPistonDamping));
            face->position.x = (c.left + c.right) * 0.5;
            face->velocity.x = 0.0;
        }
    }

    // Swept circle-vs-thick-segment collision.  'side_ref' decides which side
    // of the segment the object belongs to (its position at the start of the
    // frame), so fast or squeezed objects cannot tunnel through a wall.
    // Returns the velocity change applied to the object (zero if none).
    static Vector collide_segment(PhysicsObject& o, const Vector& a, const Vector& b, double half_thickness,
                                  double side_ref_sign, const Vector& wall_velocity, double wall_mass) {
        const Vector ab = b - a;
        const double len2 = ab.dot(ab);
        if (len2 <= 0.0) return Vector();
        const double t = clampd((o.position - a).dot(ab) / len2, 0.0, 1.0);
        const Vector closest = a + ab * t;
        const double reach = o.radius + half_thickness;
        Vector normal;

        if (t > 0.0 && t < 1.0) {
            Vector n = Vector(-ab.y, ab.x) / std::sqrt(len2);
            if (side_ref_sign < 0.0) n = n * -1.0;
            const double depth = (o.position - a).dot(n);
            if (depth >= reach) return Vector();
            o.position += n * (reach - depth);
            normal = n;
        } else {
            const Vector d = o.position - closest;
            const double dist = d.length();
            if (dist >= reach || dist <= 0.0) return Vector();
            normal = d / dist;
            o.position = closest + normal * reach;
        }

        const double vn = (o.velocity - wall_velocity).dot(normal);
        if (vn >= 0.0) return Vector();
        const double e = o.effective_restitution();
        // Two-body impulse against a finite-mass wall (piston) or a fixed wall.
        const double mass_share = wall_mass > 0.0 ? wall_mass / (wall_mass + o.mass) : 1.0;
        const Vector dv = normal * (-(1.0 + e) * vn * mass_share);
        o.velocity += dv;
        if (!o.is_gas) {
            // Coulomb-like sliding friction along the wall.
            Vector tangent_v = (o.velocity - wall_velocity) - normal * (o.velocity - wall_velocity).dot(normal);
            o.velocity -= tangent_v * 0.02;
        }
        return dv;
    }

    void resolve_container_boundaries() {
        const double th = GasContainer::kWallHalfThickness;
        for (size_t i = 0; i < containers.size(); ++i) {
            GasContainer& c = containers[i];
            if (!c.piston) continue;
            PhysicsObject* face = c.piston;
            face->position.x = (c.left + c.right) * 0.5;
            face->velocity.x = 0.0;
            if (face->position.y < c.piston_min_y) {
                face->position.y = c.piston_min_y;
                if (face->velocity.y < 0.0) face->velocity.y = 0.0;
            }
            if (face->position.y > c.piston_max_y) {
                face->position.y = c.piston_max_y;
                if (face->velocity.y > 0.0) face->velocity.y = 0.0;
            }
            const double piston_vy = face->is_dragged ? face->position.y - c.piston_start_y : face->velocity.y;
            const Vector piston_v(0.0, piston_vy);
            const double top = c.top();

            for (const auto& object_ptr : balls) {
                PhysicsObject* o = object_ptr.get();
                if (o->is_static || o->is_container_piston || o->is_dragged) continue;
                // Broad-phase reject: skip only if the whole move (start and
                // end of the frame) stays clear of the vessel, so fast bodies
                // (e.g. blasted by a bomb) cannot jump over a wall.
                const Vector& prev = o->previous_position;
                const double pad = o->radius + th + 2.0;
                if (std::max(o->position.x, prev.x) < c.left - pad || std::min(o->position.x, prev.x) > c.right + pad ||
                    std::max(o->position.y, prev.y) < c.wall_top - pad || std::min(o->position.y, prev.y) > c.bottom + pad) continue;

                const bool in_chamber = o->container_index == int(i);

                if (in_chamber) {
                    // Bodies that started the frame inside the sealed chamber are
                    // hard-constrained to its interior, however fast they move.
                    const double e = o->effective_restitution();
                    const double pad_in = o->radius + th;
                    const double min_x = c.left + pad_in, max_x = c.right - pad_in;
                    double min_y = top + pad_in, max_y = c.bottom - pad_in;
                    if (min_y > max_y) min_y = max_y = (min_y + max_y) * 0.5;
                    if (o->position.x < min_x) { o->position.x = min_x; o->velocity.x = std::abs(o->velocity.x) * e; }
                    else if (o->position.x > max_x) { o->position.x = max_x; o->velocity.x = -std::abs(o->velocity.x) * e; }
                    if (o->position.y > max_y) {
                        o->position.y = max_y; o->velocity.y = -std::abs(o->velocity.y) * e;
                        if (!o->is_gas) o->velocity.x *= 0.98;
                    }
                    if (o->position.y < min_y) {
                        o->position.y = min_y;
                        const double vn = o->velocity.y - piston_vy;      // approach speed (negative = upward into piston)
                        if (vn < 0.0) {
                            if (o->is_gas) {
                                // Gas pressure on the piston is applied analytically.
                                o->velocity.y = piston_vy - vn * e;
                            } else {
                                // Liquids / solids exchange momentum with the piston.
                                const double share = face->mass / (face->mass + o->mass);
                                const double dv = -(1.0 + e) * vn * share;
                                o->velocity.y += dv;
                                if (!face->is_dragged) face->velocity.y -= dv * o->mass / face->mass;
                            }
                        }
                    }
                    continue;
                }

                // Bodies outside the chamber collide with the outer faces of
                // the walls and with the top of the piston (open bore).
                collide_segment(*o, Vector(c.left, c.wall_top), Vector(c.left, c.bottom), th,
                                (prev.x < c.left) ? 1.0 : -1.0, Vector(), 0.0);
                collide_segment(*o, Vector(c.right, c.wall_top), Vector(c.right, c.bottom), th,
                                (prev.x < c.right) ? 1.0 : -1.0, Vector(), 0.0);
                collide_segment(*o, Vector(c.left, c.bottom), Vector(c.right, c.bottom), th,
                                (prev.y < c.bottom) ? -1.0 : 1.0, Vector(), 0.0);
                Vector dv = collide_segment(*o, Vector(c.left, top), Vector(c.right, top), th,
                                            -1.0, piston_v, face->mass);
                // Weights resting on the piston push it down.
                if ((dv.x != 0.0 || dv.y != 0.0) && !face->is_dragged) {
                    face->velocity.y -= dv.y * o->mass / face->mass;
                }
                // Anything that still tunnelled into the sealed chamber is sent back.
                if (c.in_chamber(o->position)) {
                    o->position = prev;
                    o->velocity = o->velocity * -o->effective_restitution();
                }
            }
        }
    }

    // ==========================================
    // POSITION BASED FLUIDS (incompressible Navier–Stokes on particles)
    // ==========================================
    // 2-D kernels with precomputed normalisation (fixed smoothing length).
    static double kernel_poly6(double r2) {
        static const double h2 = kFluidH * kFluidH;
        static const double coefficient = 4.0 / (M_PI_VALUE * std::pow(kFluidH, 8));
        if (r2 >= h2) return 0.0;
        const double d = h2 - r2;
        return coefficient * d * d * d;
    }
    // Gradient of the spiky kernel with respect to x_i, for r_ij = x_i − x_j.
    static Vector kernel_spiky_gradient(const Vector& r_ij, double r) {
        static const double coefficient = -30.0 / (M_PI_VALUE * std::pow(kFluidH, 5));
        if (r <= 1e-9 || r >= kFluidH) return Vector();
        return r_ij * (coefficient * (kFluidH - r) * (kFluidH - r) / r);
    }
    static constexpr double M_PI_VALUE = 3.14159265358979323846;

    void init_fluid_constants() {
        // Rest density = kernel sum over the lattice that the Water tool spawns.
        double density = 0.0;
        Vector gradient_sum;
        double gradient_sq = 0.0;
        for (int i = -4; i <= 4; ++i) {
            for (int j = -4; j <= 4; ++j) {
                const Vector r(i * kFluidSpacing, j * kFluidSpacing);
                const double len = r.length();
                density += kernel_poly6(r.dot(r));
                if (i == 0 && j == 0) continue;
                const Vector g = kernel_spiky_gradient(r, len);
                gradient_sum += g;
                gradient_sq += g.dot(g);
            }
        }
        fluid_rest_density = density;
        fluid_gradient_norm = (gradient_sq + gradient_sum.dot(gradient_sum)) / (density * density);
    }

    // Keeps a liquid particle out of walls, vessel walls and static bodies
    // (pins, heaters, coolers, ice).  Position-only: PBF derives velocity
    // from the corrected positions afterwards.
    void project_fluid_position(PhysicsObject* o, const std::vector<PhysicsObject*>& obstacles) {
        const double r = o->radius;
        o->position.x = clampd(o->position.x, r, std::max(r, double(width()) - r));
        o->position.y = clampd(o->position.y, r, std::max(r, double(height()) - r));

        const double th = GasContainer::kWallHalfThickness;
        for (size_t i = 0; i < containers.size(); ++i) {
            const GasContainer& c = containers[i];
            if (!c.piston) continue;
            if (o->container_index == int(i)) {
                const double pad = r + th;
                double min_y = c.top() + pad, max_y = c.bottom - pad;
                if (min_y > max_y) min_y = max_y = (min_y + max_y) * 0.5;
                o->position.x = clampd(o->position.x, c.left + pad, c.right - pad);
                o->position.y = clampd(o->position.y, min_y, max_y);
                continue;
            }
            const Vector& prev = o->previous_position;
            const double pad = r + th + 2.0;
            if (std::max(o->position.x, prev.x) < c.left - pad || std::min(o->position.x, prev.x) > c.right + pad ||
                std::max(o->position.y, prev.y) < c.wall_top - pad || std::min(o->position.y, prev.y) > c.bottom + pad) continue;
            const Vector saved_velocity = o->velocity;
            collide_segment(*o, Vector(c.left, c.wall_top), Vector(c.left, c.bottom), th, prev.x < c.left ? 1.0 : -1.0, Vector(), 0.0);
            collide_segment(*o, Vector(c.right, c.wall_top), Vector(c.right, c.bottom), th, prev.x < c.right ? 1.0 : -1.0, Vector(), 0.0);
            collide_segment(*o, Vector(c.left, c.bottom), Vector(c.right, c.bottom), th, prev.y < c.bottom ? -1.0 : 1.0, Vector(), 0.0);
            collide_segment(*o, Vector(c.left, c.top()), Vector(c.right, c.top()), th, -1.0, Vector(), 0.0);
            if (c.in_chamber(o->position)) o->position = prev;
            o->velocity = saved_velocity;
        }

        for (PhysicsObject* s : obstacles) {
            if (s->container_index != o->container_index) continue;
            Vector d = o->position - s->position;
            if (s->shape_type == ShapeType::SQUARE) {
                // Closest point on an axis-aligned square.
                const Vector closest(clampd(o->position.x, s->position.x - s->radius, s->position.x + s->radius),
                                     clampd(o->position.y, s->position.y - s->radius, s->position.y + s->radius));
                d = o->position - closest;
                const double dist = d.length();
                if (dist >= r) continue;
                if (dist > 1e-9) { o->position = closest + d * (r / dist); continue; }
                // Centre inside the square: exit through the nearest face.
                const Vector rel = o->position - s->position;
                if (std::abs(rel.x) > std::abs(rel.y)) o->position.x = s->position.x + (rel.x >= 0 ? 1 : -1) * (s->radius + r);
                else o->position.y = s->position.y + (rel.y >= 0 ? 1 : -1) * (s->radius + r);
            } else {
                const double dist = d.length();
                const double min_dist = r + s->radius;
                if (dist >= min_dist) continue;
                if (dist > 1e-9) o->position = s->position + d * (min_dist / dist);
                else o->position.y = s->position.y - min_dist;
            }
        }
    }

    void solve_fluids() {
        std::vector<PhysicsObject*> fluid;
        std::vector<PhysicsObject*> obstacles;
        for (const auto& b : balls) {
            if (b->is_fluid) fluid.push_back(b.get());
            else if (b->is_static && !b->is_container_piston) obstacles.push_back(b.get());
        }
        const int n = int(fluid.size());
        if (n == 0) return;
        const double h = kFluidH;
        const double h2 = h * h;

        // ---- Neighbour search on a uniform grid (cell = h) ----
        const int cols = std::max(1, int(std::ceil(width() / h)) + 2);
        const int rows = std::max(1, int(std::ceil(height() / h)) + 2);
        std::vector<std::vector<int>> cells(size_t(cols) * rows);
        auto cell_of = [&](const Vector& p, int& cx, int& cy) {
            cx = std::clamp(int(std::floor(p.x / h)) + 1, 0, cols - 1);
            cy = std::clamp(int(std::floor(p.y / h)) + 1, 0, rows - 1);
        };
        for (int i = 0; i < n; ++i) {
            int cx, cy; cell_of(fluid[i]->position, cx, cy);
            cells[size_t(cy) * cols + cx].push_back(i);
        }
        std::vector<std::vector<int>> neighbors(n);
        for (int i = 0; i < n; ++i) {
            int cx, cy; cell_of(fluid[i]->position, cx, cy);
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int x = cx + dx, y = cy + dy;
                    if (x < 0 || y < 0 || x >= cols || y >= rows) continue;
                    for (int j : cells[size_t(y) * cols + x]) {
                        if (j == i) continue;
                        // Liquid on opposite sides of a vessel wall does not interact.
                        if (fluid[j]->container_index != fluid[i]->container_index) continue;
                        const Vector d = fluid[i]->position - fluid[j]->position;
                        if (d.dot(d) < h2) neighbors[i].push_back(j);
                    }
                }
            }
        }

        // Temperature-dependent rest density: real water density curve.
        const double reference_density = thermo::water_density(20.0);
        std::vector<double> rest(n), lambda(n);
        std::vector<Vector> delta(n);
        for (int i = 0; i < n; ++i)
            rest[i] = fluid_rest_density * thermo::water_density(fluid[i]->temperature) / reference_density;

        const double w0 = kernel_poly6(0.0);
        const double w_dq = kernel_poly6(0.04 * h2); // Δq = 0.2 h
        const double relaxation = kFluidRelaxation * fluid_gradient_norm;
        std::vector<double> density(n, 0.0);

        // ---- Incompressibility (pressure) projection ----
        // Jacobi iterations until the mean density error is below 0.1 %.
        for (int iteration = 0; iteration < kFluidMaxIterations; ++iteration) {
            double total_error = 0.0;
            for (int i = 0; i < n; ++i) {
                PhysicsObject* pi = fluid[i];
                double rho = pi->mass * w0;
                Vector grad_i;
                double grad_sq = 0.0;
                for (int j : neighbors[i]) {
                    PhysicsObject* pj = fluid[j];
                    const Vector r_ij = pi->position - pj->position;
                    const double r2 = r_ij.dot(r_ij);
                    if (r2 >= h2) continue;
                    rho += pj->mass * kernel_poly6(r2);
                    const Vector g = kernel_spiky_gradient(r_ij, std::sqrt(r2)) * (pj->mass / rest[i]);
                    grad_i += g;
                    grad_sq += g.dot(g);
                }
                density[i] = rho;
                // Unilateral constraint: liquid resists compression but a free
                // surface is not pulled together by negative pressure.
                const double constraint = std::max(0.0, rho / rest[i] - 1.0);
                total_error += constraint;
                lambda[i] = -constraint / (grad_sq + grad_i.dot(grad_i) + relaxation);
            }
            if (iteration >= kFluidMinIterations && total_error / n < 0.001) break;
            for (int i = 0; i < n; ++i) {
                PhysicsObject* pi = fluid[i];
                Vector dp;
                for (int j : neighbors[i]) {
                    PhysicsObject* pj = fluid[j];
                    const Vector r_ij = pi->position - pj->position;
                    const double r2 = r_ij.dot(r_ij);
                    if (r2 >= h2) continue;
                    const double ratio = kernel_poly6(r2) / w_dq;
                    const double s_corr = -kFluidAntiCluster * ratio * ratio * ratio * ratio / fluid_gradient_norm;
                    dp += kernel_spiky_gradient(r_ij, std::sqrt(r2)) * ((lambda[i] + lambda[j] + s_corr) * pj->mass / rest[i]);
                }
                delta[i] = dp;
            }
            for (int i = 0; i < n; ++i) {
                if (fluid[i]->is_dragged) continue;
                fluid[i]->position += delta[i];
                project_fluid_position(fluid[i], obstacles);
            }
        }

        // ---- Velocity update (dt = 1 frame) ----
        std::vector<Vector> velocity(n);
        for (int i = 0; i < n; ++i) {
            PhysicsObject* pi = fluid[i];
            if (pi->is_dragged) { velocity[i] = Vector(); continue; }
            Vector v = pi->position - pi->previous_position;
            const double speed = v.length();
            if (speed > 150.0) v = v * (150.0 / speed);
            velocity[i] = v;
        }

        // ---- Viscosity (XSPH), thermal buoyancy and surface tension ----
        for (int i = 0; i < n; ++i) {
            PhysicsObject* pi = fluid[i];
            if (pi->is_dragged) continue;
            Vector viscous, cohesion;
            double weight_sum = 0.0, weighted_temperature = 0.0;
            for (int j : neighbors[i]) {
                PhysicsObject* pj = fluid[j];
                const Vector r_ij = pi->position - pj->position;
                const double r2 = r_ij.dot(r_ij);
                if (r2 >= h2) continue;
                const double w = kernel_poly6(r2);
                viscous += (velocity[j] - velocity[i]) * w;
                weight_sum += w;
                weighted_temperature += pj->temperature * w;

                // Akinci cohesion: repulsive very close, attractive out to h.
                const double r = std::sqrt(r2);
                if (r > 1e-9) {
                    const double a = (h - r) * (h - r) * (h - r) * r * r * r * 64.0 / std::pow(h, 6);
                    const double shape = (2.0 * r > h) ? a : 2.0 * a - 1.0;
                    // Akinci correction: weaker in the bulk, stronger at the surface.
                    const double correction = (rest[i] + rest[j]) / std::max(1e-12, density[i] + density[j]);
                    cohesion -= r_ij * (shape * correction / r);
                }
            }
            Vector v = velocity[i];
            if (weight_sum > 0.0) {
                // Viscosity follows the real temperature dependence of water.
                const double mu = std::min(0.5, fluid_viscosity * thermo::water_viscosity_ratio(pi->temperature));
                v += viscous * (mu / weight_sum);

                // Boussinesq buoyancy: a parcel lighter than the surrounding
                // water (hotter, or below 4 °C) rises; denser parcels sink.
                const double surrounding = thermo::water_density(weighted_temperature / weight_sum);
                const double own = thermo::water_density(pi->temperature);
                v.y -= gravity.y * (surrounding - own) / surrounding;
            }
            // Surface tension weakens as water heats up.
            v += cohesion * (kFluidCohesion * thermo::water_surface_tension_ratio(pi->temperature));
            pi->velocity = v;
        }
    }

    double collision_restitution(const PhysicsObject& first, const PhysicsObject& second) const {
        if (first.is_fluid || second.is_fluid) return 0.04;
        // Each body's own temperature sets its restitution; the softer,
        // lossier body dominates the impact.
        return std::min(first.effective_restitution(), second.effective_restitution());
    }

    void handle_spawning(double x, double y) {
        if (active_tool == ObjectType::BALL) spawn_at(x, y, ShapeType::CIRCLE);
        else if (active_tool == ObjectType::BOX) spawn_at(x, y, ShapeType::SQUARE);
        else if (active_tool == ObjectType::WHEEL) spawn_at(x, y, ShapeType::CIRCLE, 25);
        else if (active_tool == ObjectType::PIN) {
            auto obj = spawn_at(x, y, ShapeType::SQUARE, 20, -1, true);
            obj->color = QColor(100, 100, 120);
        } else if (active_tool == ObjectType::MOTOR) {
            auto obj = spawn_at(x, y, ShapeType::CIRCLE, 20);
            obj->is_motor = true; obj->angular_velocity = 0.5; obj->color = QColor(255, 80, 80);
        } else if (active_tool == ObjectType::ROPE) {
            std::vector<PhysicsObject*> nodes;
            for (int i = 0; i < 12; ++i) {
                auto node = spawn_at(x, y + (i * 15), ShapeType::CIRCLE, 6, 1.0);
                node->elasticity = 0.0;
                node->is_bouncy = false;
                if (i == 0) { node->is_static = true; node->color = QColor(200, 200, 200); }
                nodes.push_back(node);
            }
            for (size_t i = 0; i < nodes.size() - 1; ++i) {
                rigid_links.emplace_back(nodes[i], nodes[i+1], 15.0);
            }
        } else if (active_tool == ObjectType::WATER) {
            for (int i = 0; i < 15; ++i) {
                for (int j = 0; j < 15; ++j) {
                    auto node = spawn_at(x + i*kFluidSpacing, y + j*kFluidSpacing, ShapeType::CIRCLE, 6.0, 1.0, false, true);
                    node->is_bouncy = false;
                    node->velocity = Vector(0, 0, 0); // a coherent drop, not a spray
                    node->add_heat(0.0); // settle phase if the room is below 0 °C or above boiling
                }
            }
        } else if (active_tool == ObjectType::GAS) {
            for (int i = 0; i < 400; ++i) {
                spawn_gas_particle(x + random_double(-15, 15), y + random_double(-15, 15));
            }
        } else if (active_tool == ObjectType::HEATER) {
            auto obj = spawn_at(x, y, ShapeType::SQUARE, 25, -1, true);
            obj->is_heater = true;
            obj->temperature = 300.0;
            obj->color = QColor(255, 60, 60);
        } else if (active_tool == ObjectType::COOLER) {
            auto obj = spawn_at(x, y, ShapeType::SQUARE, 25, -1, true);
            obj->is_cooler = true;
            obj->temperature = -100.0;
            obj->color = QColor(60, 200, 255);
        } else if (active_tool == ObjectType::CONTAINER) {
            spawn_container(x, y);
        } else if (active_tool == ObjectType::BOMB) {
            auto obj = spawn_at(x, y, ShapeType::CIRCLE, 15, 10.0);
            obj->color = QColor(30, 30, 30);
            obj->fuse = 120;
        }
    }

    void finalize_polygon() {
        if (drawing_points.size() < 3) { drawing_points.clear(); return; }

        double cx = 0, cy = 0;
        for (const auto& p : drawing_points) { cx += p.x(); cy += p.y(); }
        cx /= drawing_points.size(); cy /= drawing_points.size();

        QColor poly_color(random_int(100, 255), random_int(100, 255), random_int(100, 255));

        auto center_node = spawn_at(cx, cy, ShapeType::CIRCLE, 8);
        center_node->color = poly_color;

        std::vector<PhysicsObject*> b_nodes;
        for (const auto& p : drawing_points) {
            auto node = spawn_at(p.x(), p.y(), ShapeType::CIRCLE, 6);
            node->color = poly_color;
            b_nodes.push_back(node);
        }

        int n = b_nodes.size();
        for (int i = 0; i < n; ++i) {
            auto n1 = b_nodes[i];
            auto n2 = b_nodes[(i + 1) % n];

            rigid_links.emplace_back(n1, n2, (n1->position - n2->position).length());
            rigid_links.emplace_back(n1, center_node, (n1->position - center_node->position).length());

            if (n > 3) {
                auto n3 = b_nodes[(i + 2) % n];
                rigid_links.emplace_back(n1, n3, (n1->position - n3->position).length());
            }
        }
        drawing_points.clear();
    }

    void resolve_collisions() {
        std::vector<PhysicsObject*> sorted_balls;
        sorted_balls.reserve(balls.size());
        for (auto& b : balls) sorted_balls.push_back(b.get());

        std::sort(sorted_balls.begin(), sorted_balls.end(), [](PhysicsObject* a, PhysicsObject* b) {
            return a->position.x < b->position.x;
        });

        int n = sorted_balls.size();
        for (int i = 0; i < n; ++i) {
            PhysicsObject* b1 = sorted_balls[i];
            if (b1->is_container_piston) continue;
            for (int j = i + 1; j < n; ++j) {
                PhysicsObject* b2 = sorted_balls[j];

                if ((b2->position.x - b1->position.x) > (b1->radius + b2->radius + 20.0)) break;
                // Vessel pistons are handled analytically.
                if (b2->is_container_piston) continue;
                // Bodies on opposite sides of a vessel wall cannot touch mechanically.
                const bool separated = b1->container_index != b2->container_index;

                Vector delta_pos = b1->position - b2->position;
                double distance = delta_pos.length();

                // Contact conduction (also through vessel walls, more slowly).
                const double reach = b1->radius + b2->radius + 15.0;
                if (distance > 0 && distance < reach) {
                    const double contact = 1.0 - distance / reach;
                    double coupling = heat_transfer_rate * contact * 0.12 * contact_conductance(*b1, *b2);
                    if (separated) coupling *= 0.2;
                    coupling = limited_coupling(coupling, *b1, *b2);
                    const double q = (b1->temperature - b2->temperature) * coupling;
                    b1->add_heat(-q);
                    b2->add_heat(q);
                }

                if (b1->is_static && b2->is_static) continue;
                if (separated) continue;

                // Liquid–liquid mechanics are solved by the PBF solver.
                if (b1->is_fluid && b2->is_fluid) continue;
                // Solids and Gases
                {
                    double min_distance = b1->radius + b2->radius;
                    if (distance > 0 && distance < min_distance) {
                        Vector normal = delta_pos * (1.0 / distance);
                        double overlap = min_distance - distance;

                        if (b1->is_static) b2->position -= normal * overlap;
                        else if (b2->is_static) b1->position += normal * overlap;
                        else {
                            double total_mass = b1->mass + b2->mass;
                            b1->position += normal * (overlap * (b2->mass / total_mass));
                            b2->position -= normal * (overlap * (b1->mass / total_mass));
                        }

                        double restitution = collision_restitution(*b1, *b2);

                        Vector delta_vel = b1->velocity - b2->velocity;
                        double vel_along_normal = delta_vel.dot(normal);

                        if (vel_along_normal < 0) {
                            double inv_mass1 = b1->is_static ? 0 : 1.0 / b1->mass;
                            double inv_mass2 = b2->is_static ? 0 : 1.0 / b2->mass;
                            double impulse_scalar = -(1.0 + restitution) * vel_along_normal / (inv_mass1 + inv_mass2);
                            Vector impulse = normal * impulse_scalar;

                            if (!b1->is_static) b1->velocity += impulse * inv_mass1;
                            if (!b2->is_static) b2->velocity -= impulse * inv_mass2;
                        }
                    }
                }
            }
        }
    }

    void detonate_bomb(PhysicsObject* bomb) {
        double blast_radius = 250.0;
        double max_force = 1500.0;

        for (auto& obj_ptr : balls) {
            PhysicsObject* target = obj_ptr.get();
            if (target == bomb || target->is_static) continue;

            Vector delta = target->position - bomb->position;
            double distance = delta.length();

            if (distance > 0 && distance < blast_radius) {
                double falloff = 1.0 - (distance / blast_radius);
                Vector push_dir = delta * (1.0 / distance);
                double applied_force = max_force * (falloff * falloff);
                const Vector kick = push_dir * (applied_force / target->mass);
                target->velocity += kick;
                // PBF derives liquid velocity from displacement this frame.
                if (target->is_fluid) target->position += kick;
            }
        }
    }

    void game_loop() {
        if (!is_paused) step_simulation();
        update();
    }

public:
    void step_simulation() {
        for (auto& b : balls) b->previous_position = b->position;

        update_containers();
        update_thermal_environment();
        for (auto& s : springs) s.update();
        for (auto& p : pulleys) p.update();
        for (auto& l : pistons) l.update(ambient_temperature);
        for (int i = 0; i < 4; ++i) {
            for (auto& l : rigid_links) l.update();
        }

        std::vector<PhysicsObject*> to_delete;
        for (auto& b : balls) {
            b->update(gravity, air_resistance, width(), height(), ambient_temperature);

            if (b->fuse > 0) {
                b->fuse--;
                if (b->fuse == 0) {
                    detonate_bomb(b.get());
                    to_delete.push_back(b.get());
                }
            }
        }

        resolve_container_boundaries();

        for (auto* bomb : to_delete) {
            remove_object(bomb);
        }

        solve_fluids();
        resolve_collisions();
        // Collision pushes can shove bodies into a wall; re-apply the walls.
        resolve_container_boundaries();
    }
};

// ==========================================
// MAIN WINDOW
// ==========================================

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    QStackedWidget* stack;
    PhysicsCanvas* canvas;
    std::vector<std::pair<QPushButton*, ObjectType>> tool_buttons;
    QPushButton* pause_btn;
    QSlider *grav_slider, *elas_slider, *wind_slider;
    QSlider *temp_slider, *heat_slider, *visc_slider;

    MainWindow() {
        setWindowTitle("Sapphire");
        resize(1200, 900);
        setStyleSheet("background-color: #282c34;");

        stack = new QStackedWidget(this);
        setCentralWidget(stack);

        QWidget* intro_page = new QWidget();
        QVBoxLayout* intro_layout = new QVBoxLayout(intro_page);
        intro_layout->setAlignment(Qt::AlignCenter);

        QLabel* title = new QLabel("Sapphire");
        title->setFont(QFont("Arial", 32, QFont::Bold));
        title->setStyleSheet("color: #61afef; margin-bottom: 20px;");
        intro_layout->addWidget(title, 0, Qt::AlignCenter);

        QPushButton* start_btn = new QPushButton("Enter Sandbox");
        start_btn->setFixedSize(300, 60);
        start_btn->setStyleSheet(
            "QPushButton { background-color: #3e4451; color: white; font-size: 18px; border-radius: 10px; }"
            "QPushButton:hover { background-color: #4b5263; }"
        );
        connect(start_btn, &QPushButton::clicked, this, &MainWindow::start_module);
        intro_layout->addWidget(start_btn, 0, Qt::AlignCenter);

        stack->addWidget(intro_page);
    }

private slots:
    void start_module() {
        bool ok;
        int num_balls = QInputDialog::getInt(this, "Initialization", "Initial object count:", 10, 0, 500, 1, &ok);
        if (ok) {
            setup_sandbox(num_balls);
            stack->setCurrentIndex(1);
        }
    }

    void setup_sandbox(int num_balls) {
        QWidget* sandbox_widget = new QWidget();
        QHBoxLayout* sandbox_layout = new QHBoxLayout(sandbox_widget);
        sandbox_layout->setContentsMargins(0,0,0,0);
        sandbox_layout->setSpacing(0);

        QWidget* toolbox = new QWidget();
        toolbox->setFixedWidth(180);
        toolbox->setStyleSheet("background-color: #21252b; border-right: 1px solid #333;");
        QVBoxLayout* tb_layout = new QVBoxLayout(toolbox);

        QLabel* l1 = new QLabel("<b style='color:#abb2bf; font-size:14px'>SPAWN TOOLS</b>");
        tb_layout->addWidget(l1);

        std::vector<std::pair<QString, ObjectType>> tools = {
            {"Ball", ObjectType::BALL}, {"Box", ObjectType::BOX}, {"Pin (Static)", ObjectType::PIN},
            {"Draw Shape", ObjectType::POLYGON}, {"Gas Cloud", ObjectType::GAS},
            {"Rope Chain", ObjectType::ROPE}, {"Water Drop", ObjectType::WATER},
            {"Container", ObjectType::CONTAINER}, {"Heater", ObjectType::HEATER},
            {"Cooler", ObjectType::COOLER}, {"Wheel", ObjectType::WHEEL},
            {"Motor", ObjectType::MOTOR}, {"Car Chassis", ObjectType::CHASSIS},
            {"Shockwave Bomb", ObjectType::BOMB}
        };

        for (auto& t : tools) {
            QPushButton* btn = new QPushButton(t.first);
            btn->setCheckable(true);
            btn->setStyleSheet(
                "QPushButton { background-color: #3e4451; color: white; padding: 6px; border-radius: 4px; }"
                "QPushButton:checked { background-color: #61afef; color: black; font-weight: bold; }"
            );
            ObjectType t_type = t.second;
            connect(btn, &QPushButton::clicked, this, [this, t_type]() { select_tool(t_type); });
            tb_layout->addWidget(btn);
            tool_buttons.push_back({btn, t_type});
        }

        QLabel* l2 = new QLabel("<b style='color:#abb2bf; font-size:14px; margin-top:15px'>LINK TOOLS</b>");
        tb_layout->addWidget(l2);

        for (auto& t : std::vector<std::pair<QString, ObjectType>>{
            {"Bouncy Spring", ObjectType::SPRING},
            {"Pulley", ObjectType::PULLEY},
            {"Piston", ObjectType::PISTON}
        }) {
            QPushButton* btn = new QPushButton(t.first);
            btn->setCheckable(true);
            btn->setStyleSheet(
                "QPushButton { background-color: #d19a66; color: black; padding: 6px; border-radius: 4px; }"
                "QPushButton:checked { background-color: #e5c07b; font-weight: bold; }"
            );
            ObjectType t_type = t.second;
            connect(btn, &QPushButton::clicked, this, [this, t_type]() { select_tool(t_type); });
            tb_layout->addWidget(btn);
            tool_buttons.push_back({btn, t_type});
        }

        tb_layout->addStretch();

        pause_btn = new QPushButton("Pause Physics");
        pause_btn->setStyleSheet("background-color: #98c379; color: black; padding: 10px; font-weight: bold;");
        connect(pause_btn, &QPushButton::clicked, this, &MainWindow::toggle_pause);
        tb_layout->addWidget(pause_btn);

        QPushButton* clear_btn = new QPushButton("Clear All");
        clear_btn->setStyleSheet("background-color: #e06c75; color: white; padding: 10px; font-weight: bold;");
        connect(clear_btn, &QPushButton::clicked, this, [this]() { canvas->restart(0); });
        tb_layout->addWidget(clear_btn);

        sandbox_layout->addWidget(toolbox);

        QWidget* right_container = new QWidget();
        QVBoxLayout* right_layout = new QVBoxLayout(right_container);
        right_layout->setContentsMargins(0,0,0,0);
        right_layout->setSpacing(0);

        canvas = new PhysicsCanvas(this);
        right_layout->addWidget(canvas);

        QWidget* controls = new QWidget();
        controls->setStyleSheet("background-color: #1e1e24; color: white; border-top: 1px solid #333; padding: 10px;");
        QGridLayout* ctrl_layout = new QGridLayout(controls);

        auto add_slider = [&](const QString& name, QSlider*& slider_ptr, int min_v, int max_v, int def_v, int row, int col) {
            QVBoxLayout* l = new QVBoxLayout();
            QLabel* lbl = new QLabel(name);
            lbl->setAlignment(Qt::AlignCenter);
            l->addWidget(lbl);
            slider_ptr = new QSlider(Qt::Horizontal);
            slider_ptr->setRange(min_v, max_v);
            slider_ptr->setValue(def_v);
            connect(slider_ptr, &QSlider::valueChanged, this, &MainWindow::update_physics_params);
            l->addWidget(slider_ptr);
            ctrl_layout->addLayout(l, row, col);
        };

        add_slider("Gravity", grav_slider, 0, 100, 30, 0, 0);
        add_slider("Bounciness (at 20 °C)", elas_slider, 10, 100, 80, 0, 1);
        add_slider("Wind Drag", wind_slider, 0, 50, 10, 0, 2);

        add_slider("Room Setpoint (°C)", temp_slider, -100, 300, 20, 1, 0);
        add_slider("Heat Transfer Rate", heat_slider, 0, 100, 5, 1, 1);
        add_slider("Fluid Viscosity (at 20 °C)", visc_slider, 0, 100, 5, 1, 2);

        right_layout->addWidget(controls);
        sandbox_layout->addWidget(right_container);
        stack->addWidget(sandbox_widget);

        select_tool(ObjectType::BALL);
        for (int i = 0; i < num_balls; ++i) {
            canvas->spawn_at(random_double(50, 950), random_double(50, 400), ShapeType::CIRCLE);
        }
    }

    void select_tool(ObjectType type) {
        canvas->set_tool(type);
        for (auto& pair : tool_buttons) {
            pair.first->setChecked(pair.second == type);
        }
        if (type == ObjectType::CHASSIS) spawn_car();
    }

    void toggle_pause() {
        canvas->is_paused = !canvas->is_paused;
        if (canvas->is_paused) {
            pause_btn->setText("Resume Physics");
            pause_btn->setStyleSheet("background-color: #e5c07b; color: black; padding: 10px; font-weight: bold;");
        } else {
            pause_btn->setText("Pause Physics");
            pause_btn->setStyleSheet("background-color: #98c379; color: black; padding: 10px; font-weight: bold;");
        }
    }

    void spawn_car() {
        double cx = canvas->width() / 2.0;
        double cy = canvas->height() / 2.0;

        auto chassis_rear = canvas->spawn_at(cx - 35, cy, ShapeType::SQUARE, 20, 6.0);
        auto chassis_front = canvas->spawn_at(cx + 35, cy, ShapeType::SQUARE, 20, 6.0);
        chassis_rear->color = QColor(100, 150, 255);
        chassis_front->color = QColor(100, 150, 255);

        auto w_rear = canvas->spawn_at(cx - 45, cy + 45, ShapeType::CIRCLE, 16, 3.0);
        auto w_front = canvas->spawn_at(cx + 45, cy + 45, ShapeType::CIRCLE, 16, 3.0);

        w_rear->is_motor = true;
        w_rear->angular_velocity = 1.2;
        w_rear->color = QColor(255, 50, 50);
        w_front->color = QColor(50, 50, 50);

        double body_len = (chassis_rear->position - chassis_front->position).length();
        canvas->rigid_links.emplace_back(chassis_rear, chassis_front, body_len);

        auto add_strut = [&](PhysicsObject* a, PhysicsObject* b, double stiffness) {
            double dist = (a->position - b->position).length();
            canvas->springs.emplace_back(a, b, dist, stiffness);
        };

        add_strut(chassis_rear, w_rear, 0.6);
        add_strut(chassis_front, w_front, 0.6);
        add_strut(chassis_rear, w_front, 0.3);
        add_strut(chassis_front, w_rear, 0.3);
        add_strut(w_rear, w_front, 0.5);
    }

    void update_physics_params() {
        canvas->gravity.y = grav_slider->value() / 100.0;
        canvas->air_resistance = wind_slider->value() / 1000.0;
        canvas->elasticity = elas_slider->value() / 100.0;

        canvas->ambient_setpoint = temp_slider->value();
        canvas->heat_transfer_rate = heat_slider->value() / 100.0;
        canvas->fluid_viscosity = visc_slider->value() / 100.0;

        for (auto& ball : canvas->balls) {
            if (ball->is_bouncy) {
                ball->elasticity = canvas->elasticity;
            }
        }
    }
};

#ifndef SAPPHIRE_NO_MAIN
int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return app.exec();
}
#endif
