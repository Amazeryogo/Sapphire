#pragma once
// ==========================================
// THERMODYNAMIC MATERIAL MODELS
// ==========================================
// Energies are in "sim joules": specific heats use real kJ/(kg·K) values and
// one unit of object mass is treated as 1 kg.  Latent heats keep their real
// ratio to each other and to the specific heats, but are uniformly scaled by
// kLatentScale so that phase changes happen in seconds instead of minutes.
//
// All functions here are pure, so they are unit-tested directly.

#include "vector.h"

#include <cmath>

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
inline const double kGasSpeedScale = 2.0 / std::sqrt(293.15);
constexpr double kGasCollisionRate = 0.03; // Langevin relaxation rate (1/frame)
// Hottest a gas particle can get from dissipated kinetic energy (a bomb
// blast).  Energy beyond this is treated as radiated away.
constexpr double kMaxGasTemperature = 2000.0; // °C

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

// Temperature rise of an ideal-gas particle when bulk kinetic energy dKE is
// dissipated into it.  The sim's particle speeds map onto real molecular
// speeds (v_rms ∝ sqrt(T/M)), so a particle's mean kinetic energy ke_eq
// stands for (3/2)kT, and its internal energy is kT/(γ-1):
//     dT = 1.5 (γ - 1) · T · dKE / ke_eq.
inline double gas_heating_from_kinetic(double dke, double ke_eq, double kelvin_t, double gamma) {
    if (ke_eq <= 0.0) return 0.0;
    return 1.5 * (gamma - 1.0) * kelvin_t * dke / ke_eq;
}
} // namespace thermo
