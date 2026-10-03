#pragma once
// Process-wide random number helpers.  seed_rng() makes runs reproducible
// (used by the tests).

#include <random>

inline std::mt19937& rng() {
    static std::mt19937 gen(std::random_device{}());
    return gen;
}

inline void seed_rng(unsigned seed) { rng().seed(seed); }

inline double random_double(double min, double max) {
    std::uniform_real_distribution<double> dist(min, max);
    return dist(rng());
}

inline int random_int(int min, int max) {
    std::uniform_int_distribution<int> dist(min, max);
    return dist(rng());
}

inline double random_normal() {
    std::normal_distribution<double> dist(0.0, 1.0);
    return dist(rng());
}
