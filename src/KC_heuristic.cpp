#include "KC_heuristic.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

namespace {
const double INF = std::numeric_limits<double>::infinity();
const double DIAGONAL = std::sqrt(2.0);
using Clock = std::chrono::steady_clock;
double seconds(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}

// D_p: shortest path in the graph induced by the SET of trace cells.
// This is intentionally not the sum of steps along the trace sequence.
double trace_distance(Primitive *p) {
    using Cell = pair<int, int>;
    std::map<Cell, double> distance;
    for (size_t k = 0; k < p->swept_cells_i.size(); ++k)
        distance[{p->swept_cells_i[k], p->swept_cells_j[k]}] = INF;
    Cell start{0, 0}, goal{p->goal->i, p->goal->j};
    if (!distance.count(start) || !distance.count(goal))
        throw runtime_error("Primitive trace must contain both endpoints: " + to_string(p->id));
    using Item = pair<double, Cell>;
    priority_queue<Item, vector<Item>, greater<Item>> queue;
    distance[start] = 0.0;
    queue.push({0.0, start});
    while (!queue.empty()) {
        Item item = queue.top(); queue.pop();
        double d = item.first;
        Cell cell = item.second;
        if (d != distance[cell]) continue;
        if (cell == goal) return d;
        for (int di = -1; di <= 1; ++di) {
            for (int dj = -1; dj <= 1; ++dj) {
                if (di == 0 && dj == 0) continue;
                Cell next{cell.first + di, cell.second + dj};
                auto found = distance.find(next);
                if (found == distance.end()) continue;
                double nd = d + ((di && dj) ? DIAGONAL : 1.0);
                if (nd < found->second) {
                    found->second = nd;
                    queue.push({nd, next});
                }
            }
        }
    }
    throw runtime_error("Disconnected primitive trace: " + to_string(p->id));
}
} // namespace

HeuristicMode parse_heuristic_mode(const string &name) {
    if (name == "euclidean") return HeuristicMode::Euclidean;
    if (name == "safe-euclidean") return HeuristicMode::SafeEuclidean;
    if (name == "grid") return HeuristicMode::Grid;
    if (name == "max") return HeuristicMode::Max;
    if (name == "additive") return HeuristicMode::Additive;
    if (name == "kinematic") return HeuristicMode::Kinematic;
    if (name == "directional") return HeuristicMode::Directional;
    throw invalid_argument("Unknown heuristic: " + name);
}

const char *heuristic_mode_name(HeuristicMode mode) {
    switch (mode) {
        case HeuristicMode::Euclidean: return "euclidean";
        case HeuristicMode::SafeEuclidean: return "safe-euclidean";
        case HeuristicMode::Grid: return "grid";
        case HeuristicMode::Max: return "max";
        case HeuristicMode::Additive: return "additive";
        case HeuristicMode::Kinematic: return "kinematic";
        case HeuristicMode::Directional: return "directional";
    }
    throw invalid_argument("Invalid heuristic mode");
}

HeuristicCalibration::HeuristicCalibration(ControlSet *control_set) {
    auto start = Clock::now();
    trace_distances.resize(control_set->list_all_prims.size());
    primitive_costs.resize(control_set->list_all_prims.size());
    for (Primitive *p : control_set->list_all_prims) {
        if (!p) continue;
        ++primitive_count;
        double cost = control_set->cost(p->id);
        if (!std::isfinite(cost) || cost < 0.0)
            throw runtime_error("Invalid primitive cost");
        double d = trace_distance(p);
        trace_distances.at(p->id) = d;
        primitive_costs.at(p->id) = cost;
        if (d > 0 && cost / d < alpha_limit) {
            alpha_limit = cost / d;
            limiting_primitive = p->id;
        }
        double euclidean = std::hypot(p->goal->i, p->goal->j);
        if (euclidean > 0) beta_limit = min(beta_limit, cost / euclidean);
    }
    if (primitive_count == 0) throw runtime_error("Empty control set");
    elapsed_seconds = seconds(start);
}

vector<double> HeuristicCalibration::residual_costs(double alpha) const {
    if (!std::isfinite(alpha) || alpha <= 0 || alpha > alpha_limit)
        throw invalid_argument("Invalid residual cost partition alpha");
    vector<double> result(primitive_costs.size());
    for (size_t id=0; id<result.size(); ++id) {
        result[id] = primitive_costs[id] - alpha * trace_distances[id];
        // Never round a negative residual UP: that could exceed the edge budget.
        if (!std::isfinite(result[id]) || result[id] < 0)
            throw invalid_argument("Negative residual cost");
    }
    return result;
}

LatticeHeuristic::LatticeHeuristic(Map *map, Vertex *goal, float radius,
                                 ControlSet *control_set, HeuristicMode mode, double scale,
                                 const KinematicTable *kinematic, int period, int heading_tolerance,
                                 GridQueueMode grid_queue)
    : map_(map), goal_i_(goal->i), goal_j_(goal->j), radius_(radius), mode_(mode),
      goal_heading_(goal->theta), alpha(scale), beta(0.0), calibration(control_set) {
    calibration_time = calibration.elapsed_seconds;
    // Round down with a small margin; this does not eliminate float g/f rounding
    // in the original search implementation.
    beta = std::floor(calibration.beta_limit * 10000.0) / 10000.0;
    bool periodic = mode == HeuristicMode::Kinematic || mode == HeuristicMode::Additive;
    bool pose = periodic || mode == HeuristicMode::Directional;
    bool grid = mode == HeuristicMode::Grid || mode == HeuristicMode::Max || pose;
    if (pose && (radius >= 1.0f || heading_tolerance != 0))
        throw invalid_argument("Pose heuristic requires one goal cell and exact heading");
    if (mode == HeuristicMode::Directional && (kinematic || goal_heading_ < 0 || goal_heading_ >= NUM_HEADINGS))
        throw invalid_argument("Directional mode needs a valid goal heading and no periodic table");
    if (!std::isfinite(alpha) || alpha <= 0.0 || (grid && alpha > calibration.alpha_limit))
        throw invalid_argument("alpha must be positive and <= " + to_string(calibration.alpha_limit));
    if (!grid) return;
    auto start = Clock::now();
    grid_field::build(*map_, goal_i_, goal_j_, radius_, grid_queue, distances_);
    grid_time = seconds(start);
    if (mode == HeuristicMode::Directional) {
        start = Clock::now();
        directional_.reset(new DirectionalPotential(*control_set, beta));
        directional_time = seconds(start);
    }
    if (periodic) {
        vector<double> residual;
        if (mode == HeuristicMode::Additive) residual = calibration.residual_costs(alpha);
        const vector<double> *costs = mode == HeuristicMode::Additive ? &residual : nullptr;
        if (kinematic) {
            if (!kinematic->compatible(control_set, goal->theta, period, costs))
                throw invalid_argument("Incompatible precomputed kinematic table");
            kinematic_ = kinematic;
        } else {
            start = Clock::now();
            owned_kinematic_.reset(new KinematicTable(control_set, goal->theta, period, costs));
            kinematic_ = owned_kinematic_.get();
            kinematic_time = seconds(start);
        }
    }
}

double LatticeHeuristic::grid_distance(int i, int j) const {
    if (!map_->in_bounds(i, j)) return INF;
    if (distances_.empty()) throw logic_error("Grid field was not built");
    return distances_[i * map_->width + j];
}

double LatticeHeuristic::value(int i, int j, int heading) const {
    if (mode_ == HeuristicMode::Grid) return alpha * grid_distance(i, j);
    double di = i - goal_i_, dj = j - goal_j_;
    double d = std::sqrt(di * di + dj * dj);
    if (mode_ == HeuristicMode::Euclidean) return d;
    // R=0 is passed with EPS=0.01; integer cells then give an exact goal.
    double radius = radius_ < 1.0f ? 0.0 : double(radius_);
    double e = beta * max(0.0, d - radius);
    if (mode_ == HeuristicMode::SafeEuclidean) return e;
    double distance = grid_distance(i, j);
    double g = alpha * distance;
    // Vary the partition without weakening the previous spatial baseline.
    // The calibrated cap also keeps this valid for other control sets.
    double spatial_scale = mode_ == HeuristicMode::Additive
        ? max(alpha, min(0.9, calibration.alpha_limit)) : alpha;
    double spatial = max(e, spatial_scale * distance);
    if (mode_ == HeuristicMode::Additive)
        return max(spatial, g + kinematic_->value(i-goal_i_, j-goal_j_, heading));
    if (mode_ == HeuristicMode::Kinematic)
        return max(spatial, kinematic_->value(i-goal_i_, j-goal_j_, heading));
    if (mode_ == HeuristicMode::Directional)
        return max(spatial, directional_->value(goal_i_-i, goal_j_-j, heading, goal_heading_));
    return spatial;
}
