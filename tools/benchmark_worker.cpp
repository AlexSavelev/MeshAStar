// One process / one measured query. JSON protocol on stdout, diagnostics on stderr.
#include "KC_algorithms.hpp"
#include "KC_path_cost.hpp"
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <sys/resource.h>

MyHEAP *HEAP = nullptr;
static void check(bool ok, const char *s) { if (!ok) throw std::runtime_error(s); }
using Clock = std::chrono::steady_clock;
static double seconds(Clock::time_point t) {
    return std::chrono::duration<double>(Clock::now()-t).count();
}

int main(int argc, char **argv) { try {
    // map engine heuristic si sj st gi gj gt warmup(0/1)
    check(argc==11, "Usage: worker MAP LBA|Mesh HEURISTIC si sj st gi gj gt WARMUP");
    const string engine=argv[2], variant=argv[3];
    check(engine=="LBA" || engine=="Mesh", "Invalid engine");
    bool reference=variant=="h0";
    check(!reference || engine=="LBA", "h0 reference uses LBA");
    const bool buckets=variant=="max-buckets" || variant=="directional-buckets";
    const string base_variant=buckets?variant.substr(0,variant.size()-8):variant;
    HeuristicMode mode=reference?HeuristicMode::Euclidean:parse_heuristic_mode(base_variant);
    GridQueueMode grid_queue=buckets?GridQueueMode::Buckets:GridQueueMode::Heap;
    Vertex start(stoi(argv[4]),stoi(argv[5]),stoi(argv[6]));
    Vertex goal(stoi(argv[7]),stoi(argv[8]),stoi(argv[9]));
    check(stoi(argv[10])==0 || stoi(argv[10])==1, "Invalid warmup");
    check(start.theta>=0 && start.theta<16 && goal.theta>=0 && goal.theta<16, "Invalid heading");
    // The existing library writes diagnostics to cout. Keep stdout machine-readable.
    std::streambuf *output=std::cout.rdbuf(); std::cout.rdbuf(std::cerr.rdbuf());
    auto loading=Clock::now();
    MyHEAP heap;HEAP=&heap;
    check(bool(ifstream(argv[1])), "Map not found");
    check(bool(ifstream("data/base_control_set.txt")), "Primitives not found");
    check(bool(ifstream("data/base_mesh_info.txt")), "Mesh info not found");
    Map map;map.read_file_to_cells(argv[1]);
    check(map.in_bounds(start.i,start.j) && map.traversable(start.i,start.j), "Invalid start");
    check(map.in_bounds(goal.i,goal.j) && map.traversable(goal.i,goal.j), "Invalid goal");
    ControlSet cs;cs.load_primitives("data/base_control_set.txt");
    MeshInfo mesh;mesh.load("data/base_mesh_info.txt");
    double load_s=seconds(loading);
    // Fixed small warmup, not the measured query. No field is reused.
    double warmup_s=0;
    if(stoi(argv[10])) {
        auto before=Clock::now();Map small;small.empty_map(32,32);
        Vertex s(16,8,0),g(16,24,0);
        SearchAlgorithm warm("warm",&small,&cs,&mesh,false,reference?0:1,0,0,
                             engine=="Mesh"?1:0,mode);
        warm.grid_queue=grid_queue;
        warm.search_options.best_g=reference;warm.solve(&s,&g);
        check(warm.is_solved,"Warmup failed");warmup_s=seconds(before);
    }
    SearchAlgorithm a("benchmark",&map,&cs,&mesh,false,reference?0:1,0,0,
                      engine=="Mesh"?1:0,mode);
    a.search_options.best_g=reference;
    a.grid_queue=grid_queue;
    a.solve(&start,&goal);
    auto validation=Clock::now();
    PathCostAudit cost_audit;bool trace_ok=true;
    Vertex state=start;
    for(int id:a.path) {
        check(id>=0 && size_t(id)<cs.list_all_prims.size(),"Invalid primitive id");
        Primitive *p=cs.list_all_prims[id];check(p!=nullptr,"Missing primitive");
        if(state.theta!=p->start_theta)trace_ok=false;
        for(size_t k=0;k<p->swept_cells_i.size();++k) {
            int i=state.i+p->swept_cells_i[k],j=state.j+p->swept_cells_j[k];
            if(!map.in_bounds(i,j) || !map.traversable(i,j))trace_ok=false;
        }
        state.i+=p->goal->i;state.j+=p->goal->j;state.theta=p->goal->theta;
        cost_audit.add(cs.cost(id));
    }
    bool endpoint_ok=state.i==goal.i && state.j==goal.j && state.theta==goal.theta;
    double sum=cost_audit.precise;
    bool legacy_sum_ok=std::isfinite(a.cost_path) && std::abs(sum-a.cost_path)<=.001;
    bool sum_ok=cost_audit.matches(a.cost_path);
    bool valid=!a.is_solved || (trace_ok && endpoint_ok && sum_ok);
    double validation_s=seconds(validation);
    struct rusage usage{};getrusage(RUSAGE_SELF,&usage);
#ifdef __APPLE__
    uint64_t rss=usage.ru_maxrss;
#else
    uint64_t rss=uint64_t(usage.ru_maxrss)*1024;
#endif
    auto &s=a.search_stats;
    std::cout.rdbuf(output);
    std::cout<<setprecision(17)<<"{\"schema\":1,\"status\":\""<<(a.is_solved?"solved":"exhausted_unreachable")
      <<"\",\"grid_queue\":\""<<(buckets?"buckets":"heap")<<"\",\"heuristic_base\":\""<<base_variant
      <<"\",\"cost\":"<<a.cost_path<<",\"path_cost_double\":"<<sum
      <<",\"cost_validation\":\"float-replay-v1\",\"path_cost_float_replay\":"<<cost_audit.replay
      <<",\"legacy_sum_ok\":"<<(legacy_sum_ok?"true":"false")
      <<",\"path_valid\":"<<(valid?"true":"false")<<",\"trace_ok\":"<<(trace_ok?"true":"false")
      <<",\"endpoint_ok\":"<<(endpoint_ok?"true":"false")<<",\"sum_ok\":"<<(sum_ok?"true":"false")
      <<",\"load_s\":"<<load_s<<",\"warmup_s\":"<<warmup_s<<",\"validation_s\":"<<validation_s
      <<",\"end_to_end_s\":"<<a.end_to_end_time<<",\"search_s\":"<<a.search_time
      <<",\"setup_s\":"<<a.setup_time<<",\"cleanup_s\":"<<a.cleanup_time
      <<",\"reconstruction_s\":"<<a.trajectory_reconstruction_time
      <<",\"preprocessing_s\":"<<a.heuristic_preprocessing_time<<",\"grid_s\":"<<a.grid_time
      <<",\"calibration_s\":"<<a.calibration_time<<",\"directional_s\":"<<a.directional_time
      <<",\"steps\":"<<a.steps<<",\"checked_cells\":"<<a.checked_cells
      <<",\"candidates\":"<<s.candidates<<",\"pushes\":"<<s.pushes<<",\"pops\":"<<s.pops
      <<",\"closed_discards\":"<<s.closed_discards<<",\"stale_discards\":"<<s.stale_discards
      <<",\"dominated\":"<<s.dominated<<",\"peak_open\":"<<s.peak_open
      <<",\"closed_bytes\":"<<s.closed_bytes<<",\"peak_rss_bytes\":"<<rss<<",\"path\":[";
    for(size_t k=0;k<a.path.size();++k)std::cout<<(k?",":"")<<a.path[k];
    std::cout<<"]}"<<std::endl;
    return 0;
} catch(const std::exception &e) { std::cerr<<e.what()<<std::endl;return 2; } }
