#include "directional_potential.hpp"
#include "KC_algorithms.hpp"
#include "KC_astar.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <queue>
MyHEAP *HEAP=nullptr;
static void require(bool c,const char*m){if(!c)throw std::runtime_error(m);}
static void test(ControlSet&cs,const DirectionalPotential&p){
 uint64_t checks=0;
 // Independent forward Dijkstra on each heading graph, versus Floyd-Warshall.
 for(int v=0;v<p.N;++v)for(int start=0;start<NUM_HEADINGS;++start){
  vector<double> dist(NUM_HEADINGS,std::numeric_limits<double>::infinity());dist[start]=0;
  using Item=pair<double,int>;priority_queue<Item,vector<Item>,greater<Item>>q;q.push({0,start});
  while(!q.empty()){auto top=q.top();q.pop();if(top.first!=dist[top.second])continue;
   for(int id:cs.get_prims_by_heading(top.second)){auto prim=cs.list_all_prims[id];
    double cost=cs.cost(id)-p.beta*(p.vi[v]*prim->goal->i+p.vj[v]*prim->goal->j);
    int next=prim->goal->theta;double nd=top.first+cost;
    if(nd<dist[next]){dist[next]=nd;q.push({nd,next});}}
  }
  for(int g=0;g<NUM_HEADINGS;++g)require(std::abs(dist[g]-p.d[v][start][g])<1e-10,"Forward distance mismatch");
 }
 for(int g=0;g<NUM_HEADINGS;++g){
  require(p.value(0,0,g,g)==0,"Goal value");
  for(auto prim:cs.list_all_prims)if(prim){
   for(int v=0;v<p.N;++v){double r=cs.cost(prim->id)-p.beta*(p.vi[v]*prim->goal->i+p.vj[v]*prim->goal->j);
    require(p.d[v][prim->start_theta][g]<=r+p.d[v][prim->goal->theta][g]+1e-10,"Potential edge inequality");++checks;}
   for(int i:{-500,-17,0,19,500})for(int j:{-500,-17,0,19,500}){
    double from=p.value(i,j,prim->start_theta,g),to=p.value(i-prim->goal->i,j-prim->goal->j,prim->goal->theta,g);
    require(from<=cs.cost(prim->id)+to+1e-9,"Translated edge consistency");++checks;}
  }
 }
 cout<<"PASS: independent heading distances, goal values, "<<checks<<" edge consistency checks"<<endl;
}
struct Observe:StateLatticeParams {
 const DirectionalPotential &potential;
 uint64_t count=0,finite=0,stronger=0,crossed=0;
 double sum=0,maximum=0,optimal;
 Observe(Vertex*s,Vertex*g,Map*m,ControlSet*cs,const LatticeHeuristic*h,const DirectionalPotential&p,double opt)
 :StateLatticeParams(s,g,m,cs,1,0,0,h),potential(p),optimal(opt){}
 void get_successors(ptrSearchNode n,vector<ptrSearchNode>&list,bool lazy){
  ++count;auto v=n->vertex;double old=lattice_heuristic->value(v->i,v->j,v->theta);
  double next=std::max(old,potential.value(finish->i-v->i,finish->j-v->j,v->theta,finish->theta));
  if(std::isfinite(old)){++finite;double delta=next-old;sum+=delta;maximum=std::max(maximum,delta);stronger+=delta>1e-9;
   crossed+=optimal>=0&&n->g+old<=optimal+1e-5&&n->g+next>optimal+1e-5;}
  StateLatticeParams::get_successors(n,list,lazy);
 }
};
int main(){try{
 MyHEAP heap;HEAP=&heap;ControlSet cs;cs.load_primitives("data/base_control_set.txt");
 HeuristicCalibration calibration(&cs);double beta=std::floor(calibration.beta_limit*10000)/10000;
 auto start=std::chrono::steady_clock::now();DirectionalPotential p(cs,beta);
 double build=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
 test(cs,p);
 string folder="res/directional-experiment";
 require(!ifstream(folder+"/diagnostics.jsonl").good(),"Refusing overwrite");
 ofstream out(folder+"/diagnostics.jsonl"),meta(folder+"/preparation.json");out<<setprecision(12);meta<<setprecision(12);
 meta<<"{\"build_s\":"<<build<<",\"bytes\":"<<sizeof(p)<<",\"beta\":"<<beta<<",\"directions\":"<<p.N<<"}\n";
 ifstream in("res/multimap-experiment/diagnostic-tasks.tsv"),refs("res/search-structure-experiment/references.tsv");
 string id,file,rid;Vertex s(0,0,0),g(0,0,0);int expected,solved,rsolved;double optimal;
 while(in>>id>>file>>s.i>>s.j>>s.theta>>g.i>>g.j>>g.theta>>expected>>solved){
  require(bool(refs>>rid>>rsolved>>optimal)&&rid==id&&rsolved==solved,"References mismatch");
  Map map;map.read_file_to_cells(file);LatticeHeuristic h(&map,&g,.01f,&cs,HeuristicMode::Max);
  Observe params(&s,&g,&map,&cs,&h,p,optimal);auto result=AstarSearch(&params,false);
  if(result.path_found)HEAP->delete_SearchNode(result.final_node);
  require(result.steps==expected&&result.path_found==bool(solved),"Diagnostic baseline mismatch");
  out<<"{\"task\":\""<<id<<"\",\"solved\":"<<(solved?"true":"false")<<",\"expanded\":"<<params.count<<",\"finite\":"<<params.finite
     <<",\"stronger\":"<<params.stronger<<",\"delta_sum\":"<<params.sum<<",\"delta_max\":"<<params.maximum<<",\"crossed\":"<<params.crossed<<"}\n";out.flush();
  cout<<id<<" stronger="<<params.stronger<<"/"<<params.finite<<" delta="<<params.sum/std::max(uint64_t(1),params.finite)<<endl;
 }
}catch(const exception&e){cerr<<e.what()<<endl;return 1;}}
