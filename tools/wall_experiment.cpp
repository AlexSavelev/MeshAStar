// Controlled wall/gap or heading experiment. Run from MeshAStar.
// Usage: obj/wall_experiment OUTPUT_DIRECTORY REPEATS [wall|headings]
// Diagnostics use a separate subclass; the production search is not instrumented.
#include "KC_algorithms.hpp"
#include "KC_astar.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <stdexcept>
#include <sys/stat.h>

MyHEAP *HEAP = nullptr;
static const int SIDE = 128, WALL_COL = 64, GAP_HALF = 8;

static void require(bool ok, const string &message) {
    if (!ok) throw runtime_error(message);
}

struct ObservedLattice : StateLatticeParams {
    vector<int> visits;
    ObservedLattice(Vertex *start, Vertex *goal, Map *map, ControlSet *cs,
                    const LatticeHeuristic *h)
        : StateLatticeParams(start, goal, map, cs, 1.0f, 0.0f, 0, h),
          visits(map->width * map->height, 0) {}
    void get_successors(ptrSearchNode current, vector<ptrSearchNode> &list, bool lazy) {
        ++visits[current->vertex->i * task_map->width + current->vertex->j];
        StateLatticeParams::get_successors(current, list, lazy);
    }
};

static vector<int> observe(Map &map, Vertex &start, Vertex &goal, ControlSet &cs,
                           const LatticeHeuristic &h, int expected_steps) {
    ObservedLattice params(&start, &goal, &map, &cs, &h);
    auto result = AstarSearch(&params, false);
    require(result.path_found, "Diagnostic search failed");
    HEAP->delete_SearchNode(result.final_node);
    require(result.steps == expected_steps, "Diagnostic changes baseline expansion count");
    require(std::accumulate(params.visits.begin(), params.visits.end(), 0) == expected_steps - 1,
            "Visit count mismatch (goal is excluded)");
    return params.visits;
}

static void validate_path(SearchAlgorithm &a, Vertex &start, Vertex &goal) {
    require(a.is_solved, "Unsolved synthetic query");
    int i = start.i, j = start.j, theta = start.theta;
    for (int id : a.path) {
        Primitive *p = a.control_set->list_all_prims[id];
        require(p->start_theta == theta, "Invalid path heading");
        for (size_t k = 0; k < p->swept_cells_i.size(); ++k) {
            int ni = i + p->swept_cells_i[k], nj = j + p->swept_cells_j[k];
            require(a.map->in_bounds(ni, nj) && a.map->traversable(ni, nj), "Path collision");
        }
        i += p->goal->i; j += p->goal->j; theta = p->goal->theta;
    }
    require(i == goal.i && j == goal.j && theta == goal.theta, "Wrong path endpoint");
}

static void write_map(Map &map, const string &path) {
    ofstream out(path);
    require(bool(out), "Cannot write map");
    out << "type octile\nheight " << map.height << "\nwidth " << map.width << "\nmap\n";
    for (auto &row : map.cells) {
        for (bool occupied : row) out << (occupied ? '@' : '.');
        out << '\n';
    }
}

static void diagnostic(Map &map, Vertex &start, Vertex &goal, ControlSet &cs,
                       int baseline_steps, int max_steps, const string &path) {
    LatticeHeuristic euclidean(&map, &goal, 0.01f, &cs, HeuristicMode::SafeEuclidean);
    LatticeHeuristic combined(&map, &goal, 0.01f, &cs, HeuristicMode::Max);
    auto visits = observe(map, start, goal, cs, euclidean, baseline_steps);
    auto max_visits = observe(map, start, goal, cs, combined, max_steps);
    long count = 0, improved = 0;
    double sum_delta = 0.0, largest_delta = 0.0;
    ofstream out(path);
    require(bool(out), "Cannot write diagnostics");
    out << setprecision(12) << "{\"cells\":[\n";
    bool first = true;
    for (int i = 0; i < map.height; ++i) for (int j = 0; j < map.width; ++j) {
        if (!map.traversable(i, j)) continue;
        int id = i * map.width + j;
        double e = euclidean.value(i, j), g = combined.alpha * combined.grid_distance(i, j);
        double delta = max(0.0, g - e);
        require(std::isfinite(g), "Disconnected grid cell in wall experiment");
        count += visits[id];
        if (delta > 1e-9) improved += visits[id];
        sum_delta += visits[id] * delta;
        if (visits[id]) largest_delta = max(largest_delta, delta);
        if (!first) out << ",\n";
        first = false;
        // Cell coordinates, h_E, h_grid, expansions at this cell (all headings).
        out << '[' << i << ',' << j << ',' << e << ',' << g << ','
            << visits[id] << ',' << max_visits[id] << ']';
    }
    out << "\n],\"expanded_states\":" << count
        << ",\"improved_states\":" << improved
        << ",\"improved_fraction\":" << (count ? double(improved)/count : 0.0)
        << ",\"mean_delta_all_expanded\":" << (count ? sum_delta/count : 0.0)
        << ",\"max_delta_expanded\":" << largest_delta
        << ",\"start_h_e\":" << euclidean.value(start.i, start.j)
        << ",\"start_h_grid\":" << combined.alpha * combined.grid_distance(start.i, start.j)
        << ",\"start_grid_distance\":" << combined.grid_distance(start.i, start.j)
        << "}\n";
}

int main(int argc, char **argv) {
    try {
        string folder = argc > 1 ? argv[1] : "res/wall-experiment";
        int repeats = argc > 2 ? stoi(argv[2]) : 7;
        string experiment = argc > 3 ? argv[3] : "wall";
        require(experiment == "wall" || experiment == "headings", "Expected wall or headings mode");
        require(repeats > 0, "Repeats must be positive");
        mkdir(folder.c_str(), 0755);
        ofstream raw(folder + "/runs.jsonl");
        require(bool(raw), "Output directory must exist or have an existing parent");
        raw << setprecision(12);
        MyHEAP heap; HEAP = &heap;
        ControlSet cs; cs.load_primitives("data/base_control_set.txt");
        MeshInfo mesh; mesh.load("data/base_mesh_info.txt");
        struct Case { int offset, start_heading, goal_heading; };
        vector<Case> cases;
        if (experiment == "wall") {
            for (int offset : {-1, 0, 8, 16, 24, 32, 40, 48})
                cases.push_back({offset, 0, 0}); // -1 = empty-map control.
        } else {
            for (int st : {0, 4, 8, 12}) for (int gt : {0, 4, 8, 12})
                cases.push_back({24, st, gt});
        }
        std::mt19937 rng(20261025);
        std::shuffle(cases.begin(), cases.end(), rng);
        vector<HeuristicMode> modes{HeuristicMode::SafeEuclidean, HeuristicMode::Grid, HeuristicMode::Max};
        for (auto task : cases) {
            int offset = task.offset;
            Vertex start(64, 16, task.start_heading), goal(64, 112, task.goal_heading);
            cout << "Case offset=" << offset << ", headings=" << start.theta << "->" << goal.theta << endl;
            Map map; map.empty_map(SIDE, SIDE);
            if (offset >= 0) for (int i = 0; i < SIDE; ++i)
                map.cells[i][WALL_COL] = abs(i - (64 + offset)) > GAP_HALF;
            string name = experiment == "headings"
                ? "heading-" + to_string(start.theta) + "-" + to_string(goal.theta)
                : (offset < 0 ? "empty" : "gap-" + to_string(offset));
            string stem = folder + "/" + name;
            write_map(map, stem + ".map");
            map.filename = stem + ".map";
            SearchAlgorithm zero("reference", &map, &cs, nullptr, false, 0, 0, 0, ALGO_LATTICE_ASTAR);
            zero.solve(&start, &goal);
            validate_path(zero, start, goal);
            zero.dump_result_of_search(stem + "-reference.txt");
            int expected_steps[6] = {};
            double largest_cost_error = 0.0;
            for (int repeat = -1; repeat < repeats; ++repeat) {
                // One untimed warm-up of each variant; randomized order every repeat.
                vector<int> variants{0, 1, 2, 3, 4, 5};
                std::shuffle(variants.begin(), variants.end(), rng);
                for (int variant : variants) {
                    int algo = variant / 3;
                    HeuristicMode mode = modes[variant % 3];
                    SearchAlgorithm a("wall", &map, &cs, &mesh, false, 1, 0, 0, algo, mode);
                    a.solve(&start, &goal);
                    validate_path(a, start, goal); // Outside the measured solve().
                    double error = abs(a.cost_path - zero.cost_path);
                    largest_cost_error = max(largest_cost_error, error);
                    require(error < 0.001, "Solution cost differs from h=0 beyond float tolerance");
                    if (repeat == -1) expected_steps[variant] = a.steps;
                    require(a.steps == expected_steps[variant], "Nondeterministic expansion count");
                    if (repeat < 0) continue;
                    raw << "{\"offset\":" << offset << ",\"repeat\":" << repeat
                        << ",\"start_heading\":" << start.theta << ",\"goal_heading\":" << goal.theta
                        << ",\"algorithm\":\"" << (algo == 0 ? "LBA" : "Mesh")
                        << "\",\"heuristic\":\"" << heuristic_mode_name(mode)
                        << "\",\"steps\":" << a.steps << ",\"cost\":" << a.cost_path
                        << ",\"reference_cost\":" << zero.cost_path
                        << ",\"preprocessing_s\":" << a.heuristic_preprocessing_time
                        << ",\"calibration_s\":" << a.calibration_time
                        << ",\"grid_s\":" << a.grid_time
                        << ",\"search_s\":" << a.search_time
                        << ",\"total_s\":" << a.total_runtime
                        << ",\"end_to_end_s\":" << a.end_to_end_time << "}\n";
                    raw.flush();
                }
            }
            if (offset < 0) {
                require(expected_steps[0] == expected_steps[2], "Empty-map LBA max differs from baseline");
                require(expected_steps[3] == expected_steps[5], "Empty-map Mesh max differs from baseline");
            }
            diagnostic(map, start, goal, cs, expected_steps[0], expected_steps[2], stem + "-diagnostic.json");
            cout << "  verified; largest cost difference=" << largest_cost_error << endl;
        }
        cout << "PASS: " << cases.size() << " queries, " << cases.size()*repeats*6
             << " measured searches; paths/reference costs/diagnostics checked\n";
    } catch (const exception &e) {
        cerr << "Error: " << e.what() << '\n';
        return 1;
    }
}
