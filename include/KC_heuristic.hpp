#pragma once

#include "KC_searching.hpp"
#include "KC_kinematic.hpp"
#include "KC_directional.hpp"
#include "KC_grid_field.hpp"
#include <memory>

enum class HeuristicMode { Euclidean, SafeEuclidean, Grid, Max, Kinematic, Additive, Directional };
HeuristicMode parse_heuristic_mode(const string &name);
const char *heuristic_mode_name(HeuristicMode mode);

// Costs are calibrated against ControlSet::cost(), i.e. the float edge costs
// actually used by the planner, rather than the input file's double lengths.
struct HeuristicCalibration {
    vector<double> trace_distances, primitive_costs;
    vector<double> residual_costs(double alpha) const;
    double alpha_limit = 1.0;
    double beta_limit = 1.0;
    int limiting_primitive = -1;
    int primitive_count = 0;
    double elapsed_seconds = 0.0;
    explicit HeuristicCalibration(ControlSet *control_set);
};

// Unweighted lattice heuristic. MeshA* uses this same function at endpoints.
// Grid diagonals require ONLY their endpoints to be free.
class LatticeHeuristic {
    Map *map_;
    int goal_i_, goal_j_;
    float radius_;
    HeuristicMode mode_;
    vector<double> distances_;
    std::unique_ptr<KinematicTable> owned_kinematic_;
    const KinematicTable *kinematic_ = nullptr;
    std::unique_ptr<DirectionalPotential> directional_;
    int goal_heading_;
public:
    double alpha, beta;
    HeuristicCalibration calibration;
    double calibration_time = 0.0;
    double grid_time = 0.0;
    double kinematic_time = 0.0;
    double directional_time = 0.0;

    LatticeHeuristic(Map *map, Vertex *goal, float radius, ControlSet *control_set,
                     HeuristicMode mode, double alpha = 0.9,
                     const KinematicTable *kinematic = nullptr, int period = 64,
                     int heading_tolerance = 0,
                     GridQueueMode grid_queue = GridQueueMode::Heap);
    double value(int i, int j, int heading = -1) const;
    double grid_distance(int i, int j) const;
};
