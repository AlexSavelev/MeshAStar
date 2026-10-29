#include "KC_heuristic.hpp"
#include "KC_algorithms.hpp"
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

MyHEAP *HEAP = nullptr;

static void require(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}

static bool valid(Map &map, Primitive *p, int i, int j) {
    for (size_t k = 0; k < p->swept_cells_i.size(); ++k) {
        int ni = i + p->swept_cells_i[k], nj = j + p->swept_cells_j[k];
        if (!map.in_bounds(ni, nj) || !map.traversable(ni, nj)) return false;
    }
    return true;
}

static void check_path(SearchAlgorithm &a, Vertex &start, Vertex &goal) {
    int i = start.i, j = start.j, theta = start.theta;
    for (int id : a.path) {
        Primitive *p = a.control_set->list_all_prims[id];
        require(p->start_theta == theta && valid(*a.map, p, i, j), "Invalid reconstructed path");
        i += p->goal->i; j += p->goal->j; theta = p->goal->theta;
    }
    require(i == goal.i && j == goal.j && theta == goal.theta, "Wrong path endpoint");
}

int main() {
    MyHEAP heap;
    HEAP = &heap;
    ControlSet cs;
    cs.load_primitives("data/base_control_set.txt");
    HeuristicCalibration calibration(&cs);
    require(calibration.primitive_count == 384, "Missing primitives");
    require(std::abs(calibration.alpha_limit - 0.9025533755561258) < 1e-12, "Wrong alpha bound");
    require(std::abs(calibration.beta_limit - 0.9933301598151829) < 1e-12, "Wrong beta bound");

    Map map;
    map.empty_map(20, 20);
    Vertex goal(15, 15, 0);
    LatticeHeuristic open(&map, &goal, 0.01f, &cs, HeuristicMode::Grid);
    require(std::abs(open.grid_distance(0, 5) - (5 + 10 * std::sqrt(2.0))) < 1e-10,
            "Open-map distance is not octile");
    bool rejected = false;
    try { LatticeHeuristic bad(&map, &goal, 0.01f, &cs, HeuristicMode::Grid, 1.0); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Unsafe alpha accepted");

    // Counterexample to unscaled grid distance: only primitive 760's trace is free.
    map.cells.assign(20, vector<bool>(20, true));
    Primitive *p = cs.list_all_prims[760];
    for (size_t k = 0; k < p->swept_cells_i.size(); ++k)
        map.cells[5 + p->swept_cells_i[k]][5 + p->swept_cells_j[k]] = false;
    Vertex trace_goal(5 + p->goal->i, 5 + p->goal->j, p->goal->theta);
    LatticeHeuristic trace(&map, &trace_goal, 0.01f, &cs, HeuristicMode::Grid);
    require(std::abs(trace.grid_distance(5, 5) - (3 + 2 * std::sqrt(2.0))) < 1e-12,
            "Trace shortcuts are missing");
    require(trace.grid_distance(5, 5) > cs.cost(760), "Counterexample missing");
    require(trace.value(5, 5) <= cs.cost(760), "Scaled trace exceeds primitive cost");
    require(std::isinf(trace.value(19, 19)), "Unreachable cell should have infinite distance");

    // Diagonal must work even when BOTH adjacent axial cells are blocked.
    map.cells.assign(20, vector<bool>(20, true));
    map.cells[14][14] = map.cells[15][15] = false;
    LatticeHeuristic diagonal(&map, &goal, 0.01f, &cs, HeuristicMode::Grid);
    require(std::abs(diagonal.grid_distance(14, 14) - std::sqrt(2.0)) < 1e-12,
            "Unexpected no-corner-cutting rule");

    // Check consistency on every valid translated primitive on five seeded maps,
    // with exact and region goals. This tests map/primitive coordinate alignment.
    std::mt19937 rng(42);
    long checked = 0;
    for (int test = 0; test < 5; ++test) {
        map.empty_map(20, 20);
        if (test) for (int i = 0; i < 20; ++i) for (int j = 0; j < 20; ++j)
            map.cells[i][j] = rng() % 7 == 0;
        map.cells[15][15] = false;
        for (float radius : {0.01f, 2.01f}) {
            for (auto mode : {HeuristicMode::Grid, HeuristicMode::SafeEuclidean, HeuristicMode::Max}) {
                LatticeHeuristic h(&map, &goal, radius, &cs, mode);
                for (int i = 0; i < 20; ++i) for (int j = 0; j < 20; ++j) {
                    if (map.traversable(i, j) && float((i-15)*(i-15)+(j-15)*(j-15)) <= radius*radius)
                        require(h.value(i, j) == 0.0, "Nonzero heuristic at goal");
                    for (Primitive *q : cs.list_all_prims) {
                        if (!q || !valid(map, q, i, j)) continue;
                        double lhs = h.value(i, j);
                        double rhs = cs.cost(q->id) + h.value(i + q->goal->i, j + q->goal->j);
                        require(lhs <= rhs + 1e-10, "Inconsistent lattice heuristic");
                        ++checked;
                    }
                }
            }
        }
    }

    // End-to-end search regression: obstacle detour, exact heading, all algorithms.
    map.empty_map(24, 24);
    for (int i = 0; i < 18; ++i) map.cells[i][12] = true;
    Vertex start(6, 5, 0), finish(6, 18, 8);
    MeshInfo mesh;
    mesh.load("data/base_mesh_info.txt");
    SearchAlgorithm reference("zero", &map, &cs, nullptr, false, 0, 0, 0, ALGO_LATTICE_ASTAR);
    reference.solve(&start, &finish);
    require(reference.is_solved, "Reference case has no solution");
    check_path(reference, start, finish);
    for (auto mode : {HeuristicMode::Grid, HeuristicMode::SafeEuclidean, HeuristicMode::Max}) {
        for (int variant = 0; variant < 3; ++variant) {
            SearchAlgorithm a("test", &map, &cs, &mesh, variant == 1, 1, 0, 0,
                              variant == 2 ? ALGO_MESH_ASTAR : ALGO_LATTICE_ASTAR, mode);
            a.solve(&start, &finish);
            require(a.is_solved, "Heuristic search failed");
            require(std::abs(a.cost_path - reference.cost_path) < 1e-4, "Cost differs from h=0");
            check_path(a, start, finish);
        }
    }
    // No solution and start=goal, including w=0 with the grid option.
    map.cells.assign(24, vector<bool>(24, true));
    map.cells[start.i][start.j] = map.cells[finish.i][finish.j] = false;
    for (int variant = 0; variant < 3; ++variant) {
        for (float weight : {0.0f, 1.0f}) {
            SearchAlgorithm a("edge-case", &map, &cs, &mesh, variant == 1, weight, 0, 0,
                              variant == 2 ? ALGO_MESH_ASTAR : ALGO_LATTICE_ASTAR, HeuristicMode::Grid);
            a.solve(&start, &finish);
            require(!a.is_solved, "Disconnected query was solved");
            a.solve(&start, &start);
            require(a.is_solved && a.cost_path == 0.0, "Start=goal failed");
        }
    }
    std::cout << "PASS: " << checked << " edge consistency checks; calibration, geometry, search and path checks\n";
}
