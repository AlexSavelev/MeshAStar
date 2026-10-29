#include "KC_directional.hpp"
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
static void path_check(SearchAlgorithm &a,Vertex s,Vertex goal){
 if(!a.is_solved)return;
 double sum=0;
 for(int id:a.path){auto p=a.control_set->list_all_prims[id];require(s.theta==p->start_theta,"Heading");
  for(size_t k=0;k<p->swept_cells_i.size();++k){int i=s.i+p->swept_cells_i[k],j=s.j+p->swept_cells_j[k];
    require(a.map->in_bounds(i,j)&&a.map->traversable(i,j),"Collision");}
  s.i+=p->goal->i;s.j+=p->goal->j;s.theta=p->goal->theta;sum+=a.control_set->cost(id);
 }
 require(s.i==goal.i&&s.j==goal.j&&s.theta==goal.theta,"Goal");require(std::abs(sum-a.cost_path)<.001,"Path cost");
}

int main(){try{
 MyHEAP heap;HEAP=&heap;ControlSet cs;cs.load_primitives("data/base_control_set.txt");MeshInfo mesh;mesh.load("data/base_mesh_info.txt");
 HeuristicCalibration calibration(&cs);double beta=std::floor(calibration.beta_limit*10000)/10000;
 DirectionalPotential p(cs,beta);test(cs,p);
 bool bad=false;try{DirectionalPotential invalid(cs,2);}catch(const invalid_argument&){bad=true;}require(bad,"Unsafe beta accepted");
 Map map;map.empty_map(48,48);for(int i=0;i<48;++i)if(i<8||i>18)map.cells[i][24]=true;
 Vertex goal(24,40,3);
 for(int invalid=0;invalid<2;++invalid){bad=false;try{LatticeHeuristic h(&map,&goal,invalid?0.01:1,&cs,HeuristicMode::Directional,.9,nullptr,64,invalid?1:0);}catch(const invalid_argument&){bad=true;}require(bad,"Unsupported goal tolerance accepted");}
 LatticeHeuristic h(&map,&goal,.01f,&cs,HeuristicMode::Directional),base(&map,&goal,.01f,&cs,HeuristicMode::Max);
 bad=false;try{h.value(24,8);}catch(const invalid_argument&){bad=true;}require(bad,"Missing heading accepted");
 uint64_t edges=0;
 for(int i=2;i<46;i+=3)for(int j=2;j<46;j+=3)if(map.traversable(i,j))for(auto prim:cs.list_all_prims)if(prim){
  bool valid=true;for(size_t k=0;k<prim->swept_cells_i.size();++k){int ni=i+prim->swept_cells_i[k],nj=j+prim->swept_cells_j[k];if(!map.in_bounds(ni,nj)||!map.traversable(ni,nj)){valid=false;break;}}
  double from=h.value(i,j,prim->start_theta);require(from>=base.value(i,j),"Spatial dominance");
  if(valid){double to=h.value(i+prim->goal->i,j+prim->goal->j,prim->goal->theta);require(from<=cs.cost(prim->id)+to+1e-9,"Combined grid consistency");++edges;}
 }
 require(h.value(goal.i,goal.j,goal.theta)==0,"Combined goal value");
 for(int scenario=0;scenario<3;++scenario){
  if(scenario==2)for(int i=0;i<48;++i)map.cells[i][24]=true;
  Vertex s(24,8,scenario?8:0),g(24,40,scenario?3:0);if(scenario==1)g=s;
  SearchAlgorithm zero("h0",&map,&cs,&mesh,false,0,0,0,0);zero.solve(&s,&g);
  for(int algorithm=0;algorithm<3;++algorithm)for(int best=0;best<2;++best){
   SearchAlgorithm a("directional",&map,&cs,&mesh,algorithm==2,1,0,0,algorithm==1?1:0,HeuristicMode::Directional);
   a.search_options.best_g=best;a.solve(&s,&g);path_check(a,s,g);require(a.is_solved==zero.is_solved,"Reachability");
   if(a.is_solved)require(std::abs(a.cost_path-zero.cost_path)<.001,"Cost mismatch");
  }
 }
 cout<<"PASS: "<<edges<<" combined map edges, spatial dominance, invalid options, LBA/Lazy/Mesh and best-g paths"<<endl;
}catch(const exception&e){cerr<<e.what()<<endl;return 1;}}
