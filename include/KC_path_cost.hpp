#pragma once
#include <cmath>

// Replay the planner's float additions; retain a separate double path cost.
struct PathCostAudit {
    float replay = 0.0f;
    double precise = 0.0;
    void add(float edge) { replay = replay + edge; precise += double(edge); }
    bool matches(double reported) const {
        return std::isfinite(reported) && std::isfinite(replay) &&
               std::isfinite(precise) && reported == double(replay);
    }
};
