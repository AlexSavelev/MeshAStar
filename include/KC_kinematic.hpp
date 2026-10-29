#pragma once
#include "KC_structs.hpp"

// Obstacle-free lattice projected onto (Z/period Z)^2 x headings.
// Default edge weights are the original primitive costs. Optional costs, indexed
// by primitive ID, define a separate abstract metric (e.g. residual costs).
// With overrides, the caller establishes the lower-bound/cost-partition proof.
class KinematicTable {
    int period_, goal_heading_;
    vector<tuple<int, int, int, int, double>> transitions_;
    vector<double> distances_;
public:
    double build_time = 0.0;
    KinematicTable(ControlSet *control_set, int goal_heading, int period = 64, const vector<double> *costs = nullptr);
    double value(int relative_i, int relative_j, int heading) const;
    bool compatible(ControlSet *control_set, int goal_heading, int period, const vector<double> *costs = nullptr) const;
    int period() const { return period_; }
    int goal_heading() const { return goal_heading_; }
    size_t distance_bytes() const { return distances_.size() * sizeof(double); }
};
