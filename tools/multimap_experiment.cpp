// Preregistered scenarios: cold queries vs actual batches with reusable tables.
#include "KC_algorithms.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <random>
#include <stdexcept>

MyHEAP *HEAP = nullptr;
using Clock = std::chrono::steady_clock;
static double elapsed(Clock::time_point t) {
    return std::chrono::duration<double>(Clock::now()-t).count();
}
static void require(bool condition, const string &message) {
    if (!condition) throw runtime_error(message);
}
struct Task {
    string id, filename;
    Vertex start, goal;
    bool solved = false;
    double reference = -1;
    Task(): start(0,0,0), goal(0,0,0) {}
};
static void validate(SearchAlgorithm &a, const Task &t, bool reference=false) {
    if (!reference) {
        require(a.is_solved == t.solved, "Reachability mismatch: "+t.id);
        if (a.is_solved) require(abs(a.cost_path-t.reference)<.001, "Cost mismatch: "+t.id);
    }
    if (!a.is_solved) return;
    Vertex state=t.start;
    double sum=0;
    for (int id:a.path) {
        auto p=a.control_set->list_all_prims[id];
        require(p->start_theta==state.theta, "Path heading mismatch: "+t.id);
        for (size_t k=0;k<p->swept_cells_i.size();++k) {
            int i=state.i+p->swept_cells_i[k], j=state.j+p->swept_cells_j[k];
            require(a.map->in_bounds(i,j) && a.map->traversable(i,j), "Path collision: "+t.id);
        }
        sum+=a.control_set->cost(id);
        state.i+=p->goal->i;state.j+=p->goal->j;state.theta=p->goal->theta;
    }
    require(state.i==t.goal.i && state.j==t.goal.j && state.theta==t.goal.theta, "Wrong goal: "+t.id);
    require(abs(sum-a.cost_path)<.001, "Reconstructed cost mismatch: "+t.id);
}

int main(int argc, char **argv) {
    try {
        string folder=argc>1?argv[1]:"res/multimap-experiment";
        int repeats=argc>2?stoi(argv[2]):3;
        require(repeats>0, "Repeats must be positive");
        ifstream input(folder+"/tasks.tsv");
        require(bool(input), "Missing frozen tasks.tsv");
        vector<Task> tasks;
        Task t;
        while (input >> t.id >> t.filename >> t.start.i >> t.start.j >> t.start.theta
                     >> t.goal.i >> t.goal.j >> t.goal.theta) tasks.push_back(t);
        require(input.eof() && !tasks.empty(), "Invalid task manifest");
        require(!ifstream(folder+"/runs.jsonl").good(), "Output already exists; refusing to overwrite");
        ofstream raw(folder+"/runs.jsonl"), refs(folder+"/references.jsonl"), batches(folder+"/batches.jsonl"), prep(folder+"/preparation.jsonl");
        require(bool(raw)&&bool(refs)&&bool(batches)&&bool(prep), "Cannot create output files");
        raw<<setprecision(12);refs<<setprecision(12);batches<<setprecision(12);prep<<setprecision(12);
        MyHEAP heap; HEAP=&heap;
        ControlSet cs;cs.load_primitives("data/base_control_set.txt");
        MeshInfo mesh;mesh.load("data/base_mesh_info.txt");
        std::map<string,unique_ptr<Map>> maps;
        for (auto &task:tasks) {
            if (!maps.count(task.filename)) {
                unique_ptr<Map> m(new Map);m->read_file_to_cells(task.filename);maps[task.filename]=std::move(m);
            }
            auto m=maps.at(task.filename).get();
            require(m->in_bounds(task.start.i,task.start.j)&&m->traversable(task.start.i,task.start.j)
                    &&m->in_bounds(task.goal.i,task.goal.j)&&m->traversable(task.goal.i,task.goal.j), "Invalid endpoint");
            SearchAlgorithm zero("reference",m,&cs,&mesh,false,0,0,0,ALGO_LATTICE_ASTAR);
            zero.solve(&task.start,&task.goal);validate(zero,task,true);
            task.solved=zero.is_solved;task.reference=zero.cost_path;
            refs<<"{\"task\":\""<<task.id<<"\",\"solved\":"<<(task.solved?"true":"false")
                <<",\"cost\":"<<task.reference<<",\"steps\":"<<zero.steps<<",\"end_to_end_s\":"<<zero.end_to_end_time<<"}\n";
            refs.flush();cout<<"Reference "<<task.id<<" solved="<<task.solved<<" time="<<zero.end_to_end_time<<endl;
        }
        // One untimed warm-up of each implementation; no reuse in subsequent jobs.
        for (int algo=0;algo<2;++algo) for (int kind=0;kind<3;++kind) {
            auto &task=tasks.front();
            SearchAlgorithm a("warmup",maps.at(task.filename).get(),&cs,&mesh,false,1,0,0,algo,
                              kind?HeuristicMode::Kinematic:HeuristicMode::Max,.9,kind==2?256:64);
            a.solve(&task.start,&task.goal);validate(a,task);
        }
        std::map<tuple<int,int,int>,int> expected_steps;
        mt19937 rng(20261026);
        size_t count=0;
        for (int repeat=0;repeat<repeats;++repeat) {
            vector<int> order(tasks.size());iota(order.begin(),order.end(),0);shuffle(order.begin(),order.end(),rng);
            vector<int> jobs(10);iota(jobs.begin(),jobs.end(),0);shuffle(jobs.begin(),jobs.end(),rng);
            // Per algorithm: baseline, cold64, cold256, cached64, cached256.
            for (int job:jobs) {
                int algo=job/5, mode=job%5, kind=mode==0?0:(mode-1)%2+1;
                int period=kind==2?256:64;
                bool cached=mode>=3;
                string variant=kind==0?"max":(kind==1?"kin64":"kin256");
                string phase=mode==0?"baseline":(cached?"cached":"cold");
                string algorithm=algo==0?"LBA":"Mesh";
                std::map<int,unique_ptr<KinematicTable>> tables;
                double build_s=0, solve_s=0;
                size_t bytes=0;
                if (cached) for (int heading:{0,3,8,13}) {
                    auto begin=Clock::now();
                    unique_ptr<KinematicTable> table(new KinematicTable(&cs,heading,period));
                    double duration=elapsed(begin);build_s+=duration;bytes+=table->distance_bytes();
                    prep<<"{\"algorithm\":\""<<algorithm<<"\",\"variant\":\""<<variant
                        <<"\",\"repeat\":"<<repeat<<",\"goal_heading\":"<<heading
                        <<",\"build_s\":"<<duration<<",\"distance_bytes\":"<<table->distance_bytes()<<"}\n";
                    tables[heading]=std::move(table);
                }
                prep.flush();
                cout<<"Batch repeat="<<repeat<<" "<<algorithm<<" "<<variant<<" "<<phase<<endl;
                int position=0;
                for (int ti:order) {
                    auto &task=tasks[ti];
                    auto table=cached?tables.at(task.goal.theta).get():nullptr;
                    SearchAlgorithm a("multimap",maps.at(task.filename).get(),&cs,&mesh,false,1,0,0,algo,
                                      kind?HeuristicMode::Kinematic:HeuristicMode::Max,.9,period,table);
                    a.solve(&task.start,&task.goal);validate(a,task);
                    auto key=make_tuple(ti,algo,kind);
                    if (!expected_steps.count(key)) expected_steps[key]=a.steps;
                    require(a.steps==expected_steps[key], "Non-deterministic/cached search steps: "+task.id);
                    solve_s+=a.end_to_end_time;
                    raw<<"{\"task\":\""<<task.id<<"\",\"algorithm\":\""<<algorithm
                       <<"\",\"variant\":\""<<variant<<"\",\"phase\":\""<<phase
                       <<"\",\"repeat\":"<<repeat<<",\"position\":"<<position++
                       <<",\"solved\":"<<(a.is_solved?"true":"false")<<",\"cost\":"<<a.cost_path
                       <<",\"reference_cost\":"<<task.reference<<",\"steps\":"<<a.steps
                       <<",\"search_s\":"<<a.search_time<<",\"end_to_end_s\":"<<a.end_to_end_time
                       <<",\"preprocessing_s\":"<<a.heuristic_preprocessing_time
                       <<",\"kinematic_s\":"<<a.kinematic_time<<",\"grid_s\":"<<a.grid_time<<"}\n";
                    raw.flush();
                }
                batches<<"{\"algorithm\":\""<<algorithm<<"\",\"variant\":\""<<variant
                       <<"\",\"phase\":\""<<phase<<"\",\"repeat\":"<<repeat
                       <<",\"queries\":"<<tasks.size()<<",\"build_s\":"<<build_s
                       <<",\"solve_s\":"<<solve_s<<",\"compute_s\":"<<build_s+solve_s
                       <<",\"distance_bytes\":"<<bytes<<"}\n";
                batches.flush();count+=tasks.size();
                cout<<"Finished batch: "<<build_s+solve_s<<" s, "<<count<<" measured searches"<<endl;
            }
        }
        cout<<"PASS: "<<count<<" measured searches; paths, h=0 costs, cache equivalence checked"<<endl;
    } catch (const exception &e) { cerr<<"Error: "<<e.what()<<endl; return 1; }
}
