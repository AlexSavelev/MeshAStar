#include "KC_kinematic.hpp"
#include "KC_algorithms.hpp"
#include "common.hpp"
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <queue>
#include <stdexcept>

MyHEAP *HEAP = nullptr;
static void require(bool condition, const char *message) {
    if (!condition) throw runtime_error(message);
}
static int mod(int v, int p) { return (v % p + p) % p; }

// Independent FORWARD Dijkstra from selected states (production uses reverse edges).
static double forward(ControlSet &cs, int period, int i, int j, int theta, int goal_theta) {
    vector<double> distance(period*period*NUM_HEADINGS, std::numeric_limits<double>::infinity());
    using Item = pair<double, int>;
    priority_queue<Item, vector<Item>, greater<Item>> q;
    int start = (mod(i,period)*period+mod(j,period))*NUM_HEADINGS+theta;
    distance[start] = 0; q.push({0, start});
    while (!q.empty()) {
        auto item = q.top(); q.pop();
        double d = item.first; int id = item.second;
        if (d != distance[id]) continue;
        if (id == goal_theta) return d;
        int t = id % NUM_HEADINGS, cell = id / NUM_HEADINGS;
        for (int pid : cs.get_prims_by_heading(t)) {
            auto p = cs.list_all_prims[pid];
            int ni = mod(cell/period + p->goal->i, period);
            int nj = mod(cell%period + p->goal->j, period);
            int next = (ni*period+nj)*NUM_HEADINGS + p->goal->theta;
            double nd = d + cs.cost(pid);
            if (nd < distance[next]) { distance[next] = nd; q.push({nd,next}); }
        }
    }
    return std::numeric_limits<double>::infinity();
}

static bool valid(Map &map, Primitive *p, int i, int j) {
    for (size_t k = 0; k < p->swept_cells_i.size(); ++k) {
        int ni = i+p->swept_cells_i[k], nj = j+p->swept_cells_j[k];
        if (!map.in_bounds(ni,nj) || !map.traversable(ni,nj)) return false;
    }
    return true;
}

static void path_valid(SearchAlgorithm &a, Vertex start, Vertex goal) {
    for (int id : a.path) {
        auto p = a.control_set->list_all_prims[id];
        require(start.theta == p->start_theta && valid(*a.map,p,start.i,start.j), "Invalid path");
        start.i += p->goal->i; start.j += p->goal->j; start.theta = p->goal->theta;
    }
    require(start.i == goal.i && start.j == goal.j && start.theta == goal.theta, "Wrong endpoint");
}

int main() {
    MyHEAP heap; HEAP = &heap;
    ControlSet cs; cs.load_primitives("data/base_control_set.txt");
    long checked = 0;
    for (int goal : {0,4,8,12}) {
        KinematicTable small(&cs, goal, 8), large(&cs, goal, 16);
        require(large.value(0,0,goal) == 0, "Nonzero goal estimate");
        require(large.value(0,0,(goal+8)%16) > 0, "Heading not represented");
        for (int i = 0; i < 16; ++i) for (int j = 0; j < 16; ++j) {
            for (auto p : cs.list_all_prims) if (p) {
                double h = large.value(i,j,p->start_theta);
                double next = large.value(i+p->goal->i,j+p->goal->j,p->goal->theta);
                require(h <= cs.cost(p->id)+next+1e-10, "Inconsistent periodic heuristic");
                require(small.value(i,j,p->start_theta) <= h+1e-10, "Refined period weakens quotient bound");
                require(large.value(i-32,j+48,p->start_theta) == h, "Wrong negative modulo");
                ++checked;
            }
        }
        for (int theta : {0,3,8,13}) {
            require(std::abs(forward(cs,16,-3,7,theta,goal)-large.value(-3,7,theta)) < 1e-10,
                    "Reverse table disagrees with forward search");
        }
    }
    KinematicTable table(&cs, 8, 64);
    require(table.compatible(&cs,8,64), "Matching table rejected");
    require(!table.compatible(&cs,0,64) && !table.compatible(&cs,8,32), "Wrong table accepted");
    double old_length = cs.list_all_prims[384]->length;
    cs.list_all_prims[384]->length += 1;
    require(!table.compatible(&cs,8,64), "Changed primitive costs ignored");
    cs.list_all_prims[384]->length = old_length;

    Map map; map.empty_map(24,24);
    for (int i=0; i<18; ++i) map.cells[i][12]=true;
    Vertex start(6,5,0), goal(6,18,8);
    LatticeHeuristic h(&map,&goal,.01f,&cs,HeuristicMode::Kinematic,.9,&table,64);
    LatticeHeuristic base(&map,&goal,.01f,&cs,HeuristicMode::Max);
    bool rejected=false;
    try { h.value(6,5); } catch (const invalid_argument &) { rejected=true; }
    require(rejected,"Missing heading silently accepted");
    rejected=false;
    try { LatticeHeuristic bad(&map,&goal,2.01f,&cs,HeuristicMode::Kinematic); }
    catch (const invalid_argument &) { rejected=true; }
    require(rejected,"Unsupported spatial goal tolerance accepted");
    rejected=false;
    try { LatticeHeuristic bad(&map,&goal,.01f,&cs,HeuristicMode::Kinematic,.9,&table,64,1); }
    catch (const invalid_argument &) { rejected=true; }
    require(rejected,"Unsupported heading tolerance accepted");
    for (int i=0;i<24;++i) for(int j=0;j<24;++j) for(auto p:cs.list_all_prims) {
        if (!p || !valid(map,p,i,j)) continue;
        double value=h.value(i,j,p->start_theta);
        require(value >= base.value(i,j),"Combined bound weaker than baseline");
        require(value <= cs.cost(p->id)+h.value(i+p->goal->i,j+p->goal->j,p->goal->theta)+1e-9,
                "Combined heuristic inconsistent on a valid lattice edge");
        ++checked;
    }
    MeshInfo mesh; mesh.load("data/base_mesh_info.txt");
    for (int task=0; task<3; ++task) {
        if (task==1) { map.empty_map(32,32); start=Vertex(16,16,0); goal=Vertex(16,16,8); }
        if (task==2) { start=goal; }
        SearchAlgorithm zero("zero",&map,&cs,&mesh,false,0,0,0,ALGO_LATTICE_ASTAR);
        zero.solve(&start,&goal);
        require(zero.is_solved,"Reference case failed");
        for (bool reuse : {false,true}) for(int variant=0;variant<3;++variant) {
            SearchAlgorithm a("kin",&map,&cs,&mesh,variant==1,1,0,0,
                              variant==2?ALGO_MESH_ASTAR:ALGO_LATTICE_ASTAR,
                              HeuristicMode::Kinematic,.9,64,reuse?&table:nullptr);
            a.solve(&start,&goal);
            require(a.is_solved && std::abs(a.cost_path-zero.cost_path)<1e-4,"Search disagrees with h=0");
            path_valid(a,start,goal);
            require((a.kinematic_time==0)==reuse,"Wrong table preparation accounting");
        }
    }
    map.cells.assign(32,vector<bool>(32,true));
    start=Vertex(4,4,0); goal=Vertex(24,24,8);
    map.cells[4][4]=map.cells[24][24]=false;
    for(float weight:{0.0f,1.0f}) for(int variant=0;variant<3;++variant) {
        SearchAlgorithm a("unreachable",&map,&cs,&mesh,variant==1,weight,0,0,
                          variant==2?ALGO_MESH_ASTAR:ALGO_LATTICE_ASTAR,
                          HeuristicMode::Kinematic,.9,64,&table);
        a.solve(&start,&goal);
        require(!a.is_solved,"Unreachable task solved");
    }
    cout << "PASS: " << checked << " edge checks; forward/reverse distances, wrapping, cache validation, paths and costs\n";
}
