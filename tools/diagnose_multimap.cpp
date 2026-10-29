// Run AFTER timing. Measure how often kinematic bounds strengthen baseline expansions.
#include "KC_algorithms.hpp"
#include "KC_astar.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>

MyHEAP *HEAP=nullptr;
struct Observed : StateLatticeParams {
    const KinematicTable *tables[2];
    long visits=0, improved[2]={0,0};
    double delta[2]={0,0};
    Observed(Vertex *s,Vertex *g,Map *m,ControlSet *cs,const LatticeHeuristic *h,
             const KinematicTable *a,const KinematicTable *b): StateLatticeParams(s,g,m,cs,1,0,0,h) {
        tables[0]=a;tables[1]=b;
    }
    void get_successors(ptrSearchNode node,vector<ptrSearchNode> &successors,bool lazy) {
        auto v=node->vertex;
        double h=lattice_heuristic->value(v->i,v->j,v->theta);
        ++visits;
        for (int k=0;k<2;++k) {
            double d=tables[k]->value(v->i-finish->i,v->j-finish->j,v->theta)-h;
            if (d>1e-9) {++improved[k];delta[k]+=d;}
        }
        StateLatticeParams::get_successors(node,successors,lazy);
    }
};
int main(int argc,char **argv) {
    try {
        string folder=argc>1?argv[1]:"res/multimap-experiment";
        ifstream input(folder+"/diagnostic-tasks.tsv");
        if (!input) throw runtime_error("Missing diagnostic-tasks.tsv");
        ofstream out(folder+"/diagnostics.jsonl"), bounds(folder+"/table-bounds.jsonl");
        if (!out||!bounds) throw runtime_error("Cannot write diagnostics");
        out<<setprecision(12);bounds<<setprecision(12);
        MyHEAP heap;HEAP=&heap;ControlSet cs;cs.load_primitives("data/base_control_set.txt");
        std::map<pair<int,int>,unique_ptr<KinematicTable>> tables;
        for (int gt:{0,3,8,13}) for (int period:{64,256}) {
            unique_ptr<KinematicTable> table(new KinematicTable(&cs,gt,period));
            double max_h=0;
            for (int i=0;i<period;++i) for (int j=0;j<period;++j) for (int theta=0;theta<16;++theta) {
                double h=table->value(i,j,theta);
                if (!std::isfinite(h)) throw runtime_error("Table has unreachable states; finite global bound invalid");
                max_h=max(max_h,h);
            }
            bounds<<"{\"goal_heading\":"<<gt<<",\"period\":"<<period<<",\"max_h\":"<<max_h
                  <<",\"euclidean_radius\":"<<max_h/.9933<<"}\n";
            tables[{gt,period}]=std::move(table);
        }
        string id,filename;Vertex start(0,0,0),goal(0,0,0);int steps,solved;
        std::map<string,unique_ptr<Map>> maps;
        while (input>>id>>filename>>start.i>>start.j>>start.theta>>goal.i>>goal.j>>goal.theta>>steps>>solved) {
            if (!maps.count(filename)) {
                unique_ptr<Map> m(new Map);m->read_file_to_cells(filename);maps[filename]=std::move(m);
            }
            auto m=maps.at(filename).get();
            LatticeHeuristic spatial(m,&goal,.01f,&cs,HeuristicMode::Max);
            Observed params(&start,&goal,m,&cs,&spatial,tables.at({goal.theta,64}).get(),tables.at({goal.theta,256}).get());
            auto result=AstarSearch(&params,false);
            if (result.path_found) HEAP->delete_SearchNode(result.final_node);
            if (result.steps!=steps || result.path_found!=bool(solved)) throw runtime_error("Diagnostic altered search: "+id);
            for (int k=0;k<2;++k) {
                int period=k==0?64:256;
                out<<"{\"task\":\""<<id<<"\",\"period\":"<<period<<",\"solved\":"<<(solved?"true":"false")
                   <<",\"steps\":"<<result.steps<<",\"expanded_states\":"<<params.visits
                   <<",\"improved_states\":"<<params.improved[k]
                   <<",\"improved_fraction\":"<<(params.visits?double(params.improved[k])/params.visits:0)
                   <<",\"mean_delta\":"<<(params.visits?params.delta[k]/params.visits:0)
                   <<",\"h_spatial_start\":"<<spatial.value(start.i,start.j)
                   <<",\"h_kin_start\":"<<params.tables[k]->value(start.i-goal.i,start.j-goal.j,start.theta)<<"}\n";
            }
            out.flush();cout<<"Diagnosed "<<id<<endl;
        }
        if (!input.eof()) throw runtime_error("Malformed diagnostic tasks");
        cout<<"PASS: diagnostic searches match baseline steps and reachability"<<endl;
    } catch (const exception &e) {cerr<<e.what()<<endl;return 1;}
}
