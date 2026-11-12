#pragma once
// Solid materials.  Density sets an object's mass (and so whether it floats),
// the rest are the usual handbook values.  No Qt dependency.

enum class Material : int { Rubber = 0, Wood, Steel, Glass, Foam, Count };

struct MaterialProps {
    const char* name;
    double density;        // relative to water (g/cm³)
    double restitution;    // bounce at 20 °C (rubber takes the Bounciness slider instead)
    double friction;       // Coulomb coefficient
    double specific_heat;  // kJ/(kg·K)
    double conductivity;   // thermal, relative to liquid water (0.6 W/m·K)
};

inline const MaterialProps& material_props(Material m) {
    static const MaterialProps table[int(Material::Count)] = {
        //  name      density  bounce  friction  c_p    k
        {"Rubber",    1.10,    0.80,   0.90,     2.00,  0.27},
        {"Wood",      0.65,    0.45,   0.45,     1.70,  0.25},
        {"Steel",     7.85,    0.60,   0.50,     0.49,  30.0},   // conductivity capped (real: 80×)
        {"Glass",     2.50,    0.65,   0.40,     0.84,  1.70},
        {"Foam",      0.10,    0.30,   0.60,     1.30,  0.06},
    };
    const int i = int(m);
    return table[(i >= 0 && i < int(Material::Count)) ? i : 0];
}

inline Material material_from_int(int i) {
    return (i >= 0 && i < int(Material::Count)) ? Material(i) : Material::Rubber;
}
