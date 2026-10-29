// Fixed wall map, 16 heading pairs. Compare current max vs periodic pose tables.
// Cached trials and first-use trials are recorded separately, never mixed.
#include "KC_algorithms.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <sys/stat.h>

MyHEAP *HEAP=nullptr;
static void require(bool condition,const char *message) {
    if(!condition) throw runtime_error(message);
}
static void validate(SearchAlgorithm &a,Vertex start,Vertex goal,double reference) {
    require(a.is_solved && abs(a.cost_path-reference)<.001,"Cost differs from h=0");
    for(int id:a.path) {
        auto p=a.control_set->list_all_prims[id];
        require(p->start_theta==start.theta,"Wrong path heading");
        for(size_t k=0;k<p->swept_cells_i.size();++k) {
            int i=start.i+p->swept_cells_i[k],j=start.j+p->swept_cells_j[k];
            require(a.map->in_bounds(i,j)&&a.map->traversable(i,j),"Collision in path");
        }
        start.i+=p->goal->i;start.j+=p->goal->j;start.theta=p->goal->theta;
    }
    require(start.i==goal.i&&start.j==goal.j&&start.theta==goal.theta,"Wrong final state");
}

int main(int argc,char **argv) {
    try {
        string folder=argc>1?argv[1]:"res/kinematic-experiment";
        int repeats=argc>2?stoi(argv[2]):7;
        require(repeats>0,"Repeats must be positive");
        mkdir(folder.c_str(),0755);
        ofstream raw(folder+"/runs.jsonl"),preparation(folder+"/preparation.jsonl");
        require(bool(raw)&&bool(preparation),"Cannot open output files");
        raw<<setprecision(12);preparation<<setprecision(12);
        MyHEAP heap;HEAP=&heap;
        ControlSet cs;cs.load_primitives("data/base_control_set.txt");
        MeshInfo mesh;mesh.load("data/base_mesh_info.txt");
        Map map;map.empty_map(128,128);
        for(int i=0;i<128;++i) map.cells[i][64]=abs(i-88)>8;
        map.filename="wall: 128x128, col64, free rows80..96";
        std::map<pair<int,int>,unique_ptr<KinematicTable>> tables;
        vector<pair<int,int>> keys;
        for(int heading:{0,4,8,12}) for(int period:{64,256}) keys.push_back({heading,period});
        mt19937 rng(20261025);shuffle(keys.begin(),keys.end(),rng);
        for(int repeat=0;repeat<3;++repeat) {
            for(auto key:keys) {
                unique_ptr<KinematicTable> t(new KinematicTable(&cs,key.first,key.second));
                preparation<<"{\"goal_heading\":"<<key.first<<",\"period\":"<<key.second
                           <<",\"repeat\":"<<repeat<<",\"build_s\":"<<t->build_time
                           <<",\"distance_bytes\":"<<t->distance_bytes()<<"}\n";
                preparation.flush();
                tables[key]=std::move(t);
            }
        }
        cout<<"Prepared tables for four goal headings, periods64/256"<<endl;
        vector<pair<int,int>> cases;
        for(int st:{0,4,8,12}) for(int gt:{0,4,8,12}) cases.push_back({st,gt});
        shuffle(cases.begin(),cases.end(),rng);
        for(auto task:cases) {
            Vertex start(64,16,task.first),goal(64,112,task.second);
            cout<<"Headings "<<start.theta<<" -> "<<goal.theta<<endl;
            SearchAlgorithm zero("reference",&map,&cs,&mesh,false,0,0,0,ALGO_LATTICE_ASTAR);
            zero.solve(&start,&goal);validate(zero,start,goal,zero.cost_path);
            zero.dump_result_of_search(folder+"/reference-"+to_string(start.theta)+"-"+to_string(goal.theta)+".txt");
            LatticeHeuristic spatial(&map,&goal,.01f,&cs,HeuristicMode::Max);
            int expected[6]={};
            // -1: warm-up; 0..repeats-1: existing tables; repeats: build inside solve.
            for(int repeat=-1;repeat<=repeats;++repeat) {
                bool cold=repeat==repeats;
                vector<int> variants=cold?vector<int>{1,2,4,5}:vector<int>{0,1,2,3,4,5};
                shuffle(variants.begin(),variants.end(),rng);
                for(int variant:variants) {
                    int algo=variant/3,kind=variant%3;
                    int period=kind==2?256:64;
                    auto table=tables[{goal.theta,period}].get();
                    SearchAlgorithm a("kinematic experiment",&map,&cs,&mesh,false,1,0,0,algo,
                                      kind?HeuristicMode::Kinematic:HeuristicMode::Max,.9,period,
                                      kind&&!cold?table:nullptr);
                    a.solve(&start,&goal);validate(a,start,goal,zero.cost_path);
                    if(repeat==-1) expected[variant]=a.steps;
                    require(a.steps==expected[variant],"Cache changes search result");
                    if(repeat<0) continue;
                    double kin_start=kind?table->value(start.i-goal.i,start.j-goal.j,start.theta):0;
                    raw<<"{\"start_heading\":"<<start.theta<<",\"goal_heading\":"<<goal.theta
                       <<",\"algorithm\":\""<<(algo==0?"LBA":"Mesh")
                       <<"\",\"variant\":\""<<(kind==0?"max":(kind==1?"kin64":"kin256"))
                       <<"\",\"table_reused\":"<<(kind&&!cold?"true":"false")
                       <<",\"phase\":\""<<(cold?"cold":"steady")<<"\",\"repeat\":"<<repeat
                       <<",\"steps\":"<<a.steps<<",\"cost\":"<<a.cost_path
                       <<",\"reference_cost\":"<<zero.cost_path
                       <<",\"search_s\":"<<a.search_time<<",\"end_to_end_s\":"<<a.end_to_end_time
                       <<",\"preprocessing_s\":"<<a.heuristic_preprocessing_time
                       <<",\"kinematic_s\":"<<a.kinematic_time<<",\"grid_s\":"<<a.grid_time
                       <<",\"h_spatial_start\":"<<spatial.value(start.i,start.j)
                       <<",\"h_kin_start\":"<<kin_start<<"}\n";
                    raw.flush();
                }
            }
        }
        cout<<"PASS: "<<cases.size()*repeats*6<<" steady and "<<cases.size()*4
            <<" cold measured searches; all paths, costs and cache equivalence checked"<<endl;
    } catch(const exception &e) { cerr<<"Error: "<<e.what()<<endl;return 1; }
}
