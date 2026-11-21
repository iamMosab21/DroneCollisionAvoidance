#pragma once
#include <vector>
#include <cmath>

struct Drone {
    int id;
    float x, y;
    float vx, vy;
    bool crashed = false;
};

struct Acceleration {
    float ax = 0.0f;
    float ay = 0.0f;
};

inline Acceleration computeDroneSteering(const Drone& me,
                                          const std::vector<Drone>& allDrones) {
    (void)me;
    (void)allDrones;
    return Acceleration{};
}
