#include "KC_kinematic.hpp"
#include "common.hpp"
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>

namespace {
using Transition = tuple<int, int, int, int, double>;
vector<Transition> transitions(ControlSet *cs, const vector<double> *costs) {
    if (costs && costs->size() != cs->list_all_prims.size())
        throw invalid_argument("Wrong number of abstract costs");
    vector<Transition> result;
    for (auto p : cs->list_all_prims) if (p)
        result.emplace_back(p->start_theta, p->goal->theta,
                            p->goal->i, p->goal->j, costs ? costs->at(p->id) : double(cs->cost(p->id)));
    return result;
}
int wrap(int coordinate, int period) {
    int r = coordinate % period;
    return r < 0 ? r + period : r;
}
}

KinematicTable::KinematicTable(ControlSet *cs, int goal_heading, int period, const vector<double> *costs)
    : period_(period), goal_heading_(goal_heading) {
    auto start = std::chrono::steady_clock::now();
    if (period < 4 || period > 512 || goal_heading < 0 || goal_heading >= NUM_HEADINGS)
        throw invalid_argument("Kinematic table: period must be in [4,512], heading in [0,15]");
    transitions_ = transitions(cs, costs);
    if (transitions_.empty()) throw invalid_argument("Kinematic table: empty control set");
    vector<vector<Transition>> incoming(NUM_HEADINGS);
    for (auto edge : transitions_) {
        if (get<0>(edge) < 0 || get<0>(edge) >= NUM_HEADINGS ||
            get<1>(edge) < 0 || get<1>(edge) >= NUM_HEADINGS ||
            !std::isfinite(get<4>(edge)) || get<4>(edge) < 0)
            throw invalid_argument("Kinematic table: invalid primitive");
        incoming[get<1>(edge)].push_back(edge);
    }
    distances_.assign(period * period * NUM_HEADINGS, std::numeric_limits<double>::infinity());
    using Item = pair<double, int>;
    priority_queue<Item, vector<Item>, greater<Item>> open;
    distances_[goal_heading] = 0.0;
    open.push({0.0, goal_heading});
    while (!open.empty()) {
        auto item = open.top(); open.pop();
        double d = item.first;
        int id = item.second;
        if (d != distances_[id]) continue;
        int theta = id % NUM_HEADINGS;
        int cell = id / NUM_HEADINGS, i = cell / period, j = cell % period;
        // Reverse Dijkstra: reverse DISPLACEMENT and heading transition, not the
        // robot's physical primitive. Forward motion can be asymmetric.
        for (auto edge : incoming[theta]) {
            int pi = wrap(i - get<2>(edge), period), pj = wrap(j - get<3>(edge), period);
            int predecessor = (pi * period + pj) * NUM_HEADINGS + get<0>(edge);
            double nd = d + get<4>(edge);
            if (nd < distances_[predecessor]) {
                distances_[predecessor] = nd;
                open.push({nd, predecessor});
            }
        }
    }
    build_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

double KinematicTable::value(int di, int dj, int heading) const {
    if (heading < 0 || heading >= NUM_HEADINGS)
        throw invalid_argument("Kinematic heuristic needs an actual lattice heading");
    int index = (wrap(di, period_) * period_ + wrap(dj, period_)) * NUM_HEADINGS + heading;
    return distances_[index];
}

bool KinematicTable::compatible(ControlSet *cs, int heading, int period, const vector<double> *costs) const {
    return heading == goal_heading_ && period == period_ && transitions(cs, costs) == transitions_;
}
