// Separate, untimed replay of baseline expansions; compares full/residual P64.
#include "KC_algorithms.hpp"
#include "KC_astar.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <memory>
#include <stdexcept>
MyHEAP *HEAP=nullptr;
struct ObserveAdd:StateLatticeParams {
    const KinematicTable &full,&res;
    long count=0,better_full=0,better_add=0,add_over_full=0,full_over_add=0;
    double delta_full=0,delta_add=0;
    ObserveAdd(Vertex *s,Vertex *g,Map *m,ControlSet *cs,const LatticeHeuristic *h,
               const KinematicTable &f,const KinematicTable &r):StateLatticeParams(s,g,m,cs,1,0,0,h),full(f),res(r) {}
    void get_successors(ptrSearchNode n,vector<ptrSearchNode> &list,bool lazy) {
        auto v=n->vertex;double b=lattice_heuristic->value(v->i,v->j);
        double f=max(b,full.value(v->i-finish->i,v->j-finish->j,v->theta));
        double a=max(b,.9*lattice_heuristic->grid_distance(v->i,v->j)+res.value(v->i-finish->i,v->j-finish->j,v->theta));
        ++count;better_full+=f>b+1e-9;better_add+=a>b+1e-9;
        add_over_full+=a>f+1e-9;full_over_add+=f>a+1e-9;
        delta_full+=f-b;delta_add+=a-b;
        StateLatticeParams::get_successors(n,list,lazy);
    }
};
int main() {
    try {
        string folder="res/additive-experiment";
        ifstream in("res/multimap-experiment/diagnostic-tasks.tsv");
        ofstream out(folder+"/diagnostics.jsonl");
        if(!in||!out)throw runtime_error("Cannot open diagnostics files");
        out<<setprecision(12);
        MyHEAP heap;HEAP=&heap;ControlSet cs;cs.load_primitives("data/base_control_set.txt");
        auto residual=HeuristicCalibration(&cs).residual_costs(.9);
        std::map<int,unique_ptr<KinematicTable>> full,res;
        for(int g:{0,3,8,13}) {
            full[g].reset(new KinematicTable(&cs,g,64));res[g].reset(new KinematicTable(&cs,g,64,&residual));
        }
        string id,file;Vertex s(0,0,0),g(0,0,0);int expected,solved;
        while(in>>id>>file>>s.i>>s.j>>s.theta>>g.i>>g.j>>g.theta>>expected>>solved) {
            Map map;map.read_file_to_cells(file);
            LatticeHeuristic h(&map,&g,.01f,&cs,HeuristicMode::Max);
            ObserveAdd params(&s,&g,&map,&cs,&h,*full.at(g.theta),*res.at(g.theta));
            auto result=AstarSearch(&params,false);
            if(result.path_found)HEAP->delete_SearchNode(result.final_node);
            if(result.steps!=expected||result.path_found!=bool(solved))throw runtime_error("Diagnostic mismatch");
            out<<"{\"task\":\""<<id<<"\",\"solved\":"<<(solved?"true":"false")<<",\"expanded\":"<<params.count
               <<",\"stronger_full\":"<<params.better_full<<",\"stronger_add\":"<<params.better_add
               <<",\"add_over_full\":"<<params.add_over_full<<",\"full_over_add\":"<<params.full_over_add
               <<",\"delta_full_sum\":"<<params.delta_full<<",\"delta_add_sum\":"<<params.delta_add<<"}\n";
            out.flush();
        }
        cout<<"PASS: additive diagnostics match original baseline"<<endl;
    }catch(const exception &e){cerr<<e.what()<<endl;return 1;}
}
