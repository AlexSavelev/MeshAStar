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


int main(){try{
 const string folder="res/search-structure-experiment";
 require(!ifstream(folder+"/runs.jsonl").good(),"Refusing overwrite");
 ifstream in(folder+"/tasks.tsv"), refs(folder+"/references.tsv");
 vector<Task> tasks;Task t;string id;int solved;
 while(in>>t.id>>t.filename>>t.start.i>>t.start.j>>t.start.theta>>t.goal.i>>t.goal.j>>t.goal.theta){
   require(bool(refs>>id>>solved>>t.reference)&&id==t.id,"Reference manifest mismatch");t.solved=solved;tasks.push_back(t);
 }
 require(tasks.size()==24,"Expected 24 tasks");
 MyHEAP heap;HEAP=&heap;ControlSet cs;cs.load_primitives("data/base_control_set.txt");MeshInfo mesh;mesh.load("data/base_mesh_info.txt");
 std::map<string,unique_ptr<Map>> maps;
 for(auto &task:tasks)if(!maps.count(task.filename)){unique_ptr<Map> m(new Map);m->read_file_to_cells(task.filename);maps[task.filename]=std::move(m);}
 for(int algo=0;algo<2;++algo)for(int variant=0;variant<3;++variant){
  auto &task=tasks.front();SearchAlgorithm a("warmup",maps.at(task.filename).get(),&cs,&mesh,false,1,0,0,algo,HeuristicMode::Max);
  a.search_options.compact_closed=variant!=0;a.search_options.best_g=variant==2;a.solve(&task.start,&task.goal);validate(a,task);
 }
 ofstream out(folder+"/runs.jsonl");out<<setprecision(12);
 std::map<tuple<int,int,int>,int> steps;
 std::map<pair<int,int>,vector<int>> paths;
 mt19937 rng(20261026);
 for(int repeat=0;repeat<3;++repeat){
  vector<int> order(tasks.size());iota(order.begin(),order.end(),0);shuffle(order.begin(),order.end(),rng);
  for(int ti:order){auto &task=tasks[ti];vector<int> jobs={0,1,2,3,4,5};shuffle(jobs.begin(),jobs.end(),rng);
   for(int job:jobs){int algo=job/3,variant=job%3;
    SearchAlgorithm a("profile",maps.at(task.filename).get(),&cs,&mesh,false,1,0,0,algo,HeuristicMode::Max);
    a.search_options.compact_closed=variant!=0;a.search_options.best_g=variant==2;a.solve(&task.start,&task.goal);validate(a,task);
    auto key=make_tuple(ti,algo,variant);
    if(!steps.count(key))steps[key]=a.steps;
    require(steps[key]==a.steps,"Repeat steps changed");
    if(variant<2){auto pk=make_pair(ti,algo);if(!paths.count(pk))paths[pk]=a.path;
      require(paths[pk]==a.path,"CLOSED layout changed path");
      auto other=make_tuple(ti,algo,1-variant);if(steps.count(other))require(steps[other]==a.steps,"CLOSED layout changed steps");}
    auto &s=a.search_stats;
    out<<"{\"task\":\""<<task.id<<"\",\"algorithm\":\""<<(algo?"Mesh":"LBA")<<"\",\"variant\":\""<<(variant==0?"legacy":variant==1?"compact":"best-g")
       <<"\",\"repeat\":"<<repeat<<",\"solved\":"<<(a.is_solved?"true":"false")<<",\"cost\":"<<a.cost_path<<",\"steps\":"<<a.steps
       <<",\"end_to_end_s\":"<<a.end_to_end_time<<",\"search_s\":"<<a.search_time<<",\"setup_s\":"<<a.setup_time<<",\"cleanup_s\":"<<a.cleanup_time
       <<",\"preprocessing_s\":"<<a.heuristic_preprocessing_time<<",\"closed_bytes\":"<<s.closed_bytes<<",\"candidates\":"<<s.candidates<<",\"pushes\":"<<s.pushes
       <<",\"pops\":"<<s.pops<<",\"dominated\":"<<s.dominated<<",\"closed_discards\":"<<s.closed_discards<<",\"stale_discards\":"<<s.stale_discards
       <<",\"peak_open\":"<<s.peak_open<<",\"peak_best_g\":"<<s.peak_best_g<<",\"checked_cells\":"<<a.checked_cells<<"}\n";out.flush();
   }
   cout<<"repeat="<<repeat<<" "<<task.id<<" PASS"<<endl;
  }
 }
 cout<<"PASS: 432 searches, h0 costs, path collisions, CLOSED order and repeat equivalence"<<endl;
}catch(const exception&e){cerr<<e.what()<<endl;return 1;}}
