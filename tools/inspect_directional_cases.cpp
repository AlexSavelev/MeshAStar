#include "KC_algorithms.hpp"
#include "KC_astar.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <stdexcept>
MyHEAP *HEAP=nullptr;
struct ObservedCase:StateLatticeParams {
    ofstream &out;
    const LatticeHeuristic &spatial;
    const DirectionalPotential &table;
    ObservedCase(Vertex *s,Vertex *g,Map *m,ControlSet *cs,const LatticeHeuristic *h,
                 ofstream &o,const LatticeHeuristic &base,const DirectionalPotential &tab):
        StateLatticeParams(s,g,m,cs,1,0,0,h),out(o),spatial(base),table(tab) {}
    void get_successors(ptrSearchNode n,vector<ptrSearchNode> &list,bool lazy) {
        auto v=n->vertex;
        out<<v->i<<','<<v->j<<','<<v->theta<<','<<n->g<<','<<spatial.value(v->i,v->j)
           <<','<<table.value(finish->i-v->i,finish->j-v->j,v->theta,finish->theta);
        int winner=0;double best=-1e100, projection=0, penalty=0;
        for(int q=0;q<DirectionalPotential::N;++q){
            double z=table.beta*(table.vi[q]*(finish->i-v->i)+table.vj[q]*(finish->j-v->j));
            double add=table.d[q][v->theta][finish->theta];
            if(z+add>best){best=z+add;winner=q;projection=z;penalty=add;}
        }
        double di=v->i-finish->i,dj=v->j-finish->j;
        out<<','<<winner<<','<<projection<<','<<penalty<<','<<table.beta*sqrt(di*di+dj*dj)
           <<','<<.9*spatial.grid_distance(v->i,v->j)<<'\n';
        StateLatticeParams::get_successors(n,list,lazy);
    }
};
int main() {
    try {
        string folder="res/directional-case-study";
        ifstream in(folder+"/tasks.tsv");
        if(!in) throw runtime_error("Missing inputs");
        MyHEAP heap;HEAP=&heap;ControlSet cs;cs.load_primitives("data/base_control_set.txt");
        MeshInfo mesh;mesh.load("data/base_mesh_info.txt");
        string id,file;Vertex s(0,0,0),g(0,0,0);int expected[2];double ref;
        while(in>>id>>file>>s.i>>s.j>>s.theta>>g.i>>g.j>>g.theta>>expected[0]>>expected[1]>>ref) {
            Map map;map.read_file_to_cells(file);
            HeuristicCalibration cal(&cs);DirectionalPotential table(cs,floor(cal.beta_limit*10000)/10000);
            LatticeHeuristic base(&map,&g,.01f,&cs,HeuristicMode::Max);
            LatticeHeuristic kin(&map,&g,.01f,&cs,HeuristicMode::Directional);
            for(int kind=0;kind<2;++kind) {
                string prefix=folder+"/"+id+(kind?"-directional":"-base");
                ofstream out(prefix+".csv");out<<setprecision(12)<<"i,j,theta,g,h_spatial,h_directional,direction,projection,penalty,h_euclidean,h_grid\n";
                {
                    ObservedCase params(&s,&g,&map,&cs,kind?&kin:&base,out,base,table);
                    auto result=AstarSearch(&params,false);
                    if(!result.path_found) throw runtime_error("Diagnostic failed");
                    double cost=result.final_node->g;HEAP->delete_SearchNode(result.final_node);
                    if(result.steps!=expected[kind]||abs(cost-ref)>.001) throw runtime_error("Diagnostic differs from benchmark");
                }
                SearchAlgorithm search("case study",&map,&cs,&mesh,false,1,0,0,0,
                    kind?HeuristicMode::Directional:HeuristicMode::Max);
                search.solve(&s,&g);
                if(!search.is_solved||search.steps!=expected[kind]||abs(search.cost_path-ref)>.001)
                    throw runtime_error("Replay differs");
                ofstream path(prefix+"-path.csv");path<<"i,j\n";
                Vertex v=s;
                for(int prim:search.path) {
                    auto p=cs.list_all_prims[prim];
                    if(v.theta!=p->start_theta) throw runtime_error("Wrong heading");
                    for(size_t k=0;k<p->swept_cells_i.size();++k) {
                        int i=v.i+p->swept_cells_i[k],j=v.j+p->swept_cells_j[k];
                        if(!map.in_bounds(i,j)||!map.traversable(i,j)) throw runtime_error("Collision");
                        path<<i<<','<<j<<'\n';
                    }
                    v.i+=p->goal->i;v.j+=p->goal->j;v.theta=p->goal->theta;
                }
                if(v.i!=g.i||v.j!=g.j||v.theta!=g.theta) throw runtime_error("Wrong endpoint");
                cout<<"PASS "<<id<<" "<<kind<<" steps="<<search.steps<<endl;
            }
        }
    } catch(const exception &e){cerr<<e.what()<<endl;return 1;}
}
