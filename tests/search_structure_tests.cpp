#include "KC_algorithms.hpp"
#include "KC_astar.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
MyHEAP *HEAP=nullptr;
static void check(bool ok,const char *msg){if(!ok)throw std::runtime_error(msg);}
static ptrSearchNode node(int i,int j,int layer,float g,float f){
 auto n=HEAP->new_SearchNode(HEAP->new_Vertex(i,j,layer));n->g=g;n->f=f;return n;
}
// Cheap invalid route to state 2 must not suppress the valid route via state 1.
struct LazyToy {
 SearchTree *ast;
 LazyToy(){SearchOptions o;o.best_g=true;ast=new SearchTree(3,1,1,o);}
 ~LazyToy(){delete ast;}
 ptrVertex get_start_vertex(){return HEAP->new_Vertex(0,0,0);}
 float heuristic(ptrSearchNode){return 0;}
 bool is_goal(ptrVertex v){return v->j==2;}
 bool lazy_check_collision(ptrSearchNode n){return !(n->vertex->j==2&&n->parent->vertex->j==0);}
 void get_successors(ptrSearchNode n,vector<ptrSearchNode>&list,bool){
   if(n->vertex->j==0){auto a=node(0,2,0,1,1);a->parent=n;list.push_back(a);
                     auto b=node(0,1,0,.5,.5);b->parent=n;list.push_back(b);}
   else {auto a=node(0,2,0,2,2);a->parent=n;list.push_back(a);}
 }
};
static void path_check(SearchAlgorithm &a,Vertex s,Vertex goal){
 if(!a.is_solved)return;
 double sum=0;
 for(int id:a.path){auto p=a.control_set->list_all_prims[id];check(s.theta==p->start_theta,"Heading");
  for(size_t k=0;k<p->swept_cells_i.size();++k){int i=s.i+p->swept_cells_i[k],j=s.j+p->swept_cells_j[k];
    check(a.map->in_bounds(i,j)&&a.map->traversable(i,j),"Collision");}
  s.i+=p->goal->i;s.j+=p->goal->j;s.theta=p->goal->theta;sum+=a.control_set->cost(id);
 }
 check(s.i==goal.i&&s.j==goal.j&&s.theta==goal.theta,"Goal");check(std::abs(sum-a.cost_path)<.001,"Path cost");
}
int main(){try{
 MyHEAP heap;HEAP=&heap;auto nodes=heap.index_free_nodes.size();auto vertices=heap.index_free_vertices.size();
 {
  SearchOptions o;o.best_g=true;SearchTree tree(7,3,5,o);
  check(tree.fast_closed.size()==14,"Bit rounding");
  tree.add_to_open(node(2,6,4,10,0));tree.add_to_open(node(2,6,4,12,12));
  tree.add_to_open(node(2,6,4,8,8));tree.add_to_open(node(2,6,4,8,8));
  tree.add_to_open(node(2,6,3,9,9));
  auto n=tree.get_best_node_from_open();check(n->g==8,"Stale g popped");tree.add_to_closed(n);
  n=tree.get_best_node_from_open();check(n->vertex->config_id==3,"Layers merged");tree.add_to_closed(n);
  check(tree.stats.dominated==2&&tree.stats.stale_discards==1,"Duplicate stats");
  check(tree.best_g.empty(),"Frontier map retains CLOSED");
 }
 {LazyToy toy;auto result=AstarSearch(&toy,true);check(result.path_found&&result.final_node->g==2,"Lazy valid alternative lost");
  check(toy.ast->stats.lazy_best_g_disabled,"Lazy guard missing");HEAP->delete_SearchNode(result.final_node);}
 check(heap.index_free_nodes.size()==nodes&&heap.index_free_vertices.size()==vertices,"Pool leak");
 ControlSet cs;cs.load_primitives("data/base_control_set.txt");MeshInfo mesh;mesh.load("data/base_mesh_info.txt");
 Map map;map.empty_map(48,48);
 for(int i=0;i<48;++i)if(i<8||i>18)map.cells[i][24]=true;
 for(int scenario=0;scenario<3;++scenario){
  if(scenario==2)for(int i=0;i<48;++i)map.cells[i][24]=true;
  Vertex s(24,8,scenario?8:0),g(24,40,scenario?3:0);if(scenario==1)g=s;
  SearchAlgorithm zero("h0",&map,&cs,&mesh,false,0,0,0,ALGO_LATTICE_ASTAR);zero.solve(&s,&g);path_check(zero,s,g);
  for(int algorithm=0;algorithm<3;++algorithm){
   int previous_steps=-1;vector<int> previous_path;
   for(int variant=0;variant<3;++variant){
    SearchAlgorithm a("test",&map,&cs,&mesh,algorithm==2,1,0,0,algorithm==1?1:0,HeuristicMode::Max);
    a.search_options.compact_closed=variant!=0;a.search_options.best_g=variant==2;
    a.solve(&s,&g);path_check(a,s,g);check(a.is_solved==zero.is_solved,"Reachability");
    if(a.is_solved)check(std::abs(a.cost_path-zero.cost_path)<.001,"h0 cost mismatch");
    if(variant==0){previous_steps=a.steps;previous_path=a.path;}
    if(variant==1||algorithm==2)check(a.steps==previous_steps&&a.path==previous_path,"Compact/Lazy order changed");
    if(algorithm==2&&variant==2)check(a.search_stats.lazy_best_g_disabled,"Lazy integration guard");
   }
  }
 }
 cout<<"PASS: compact CLOSED, stale/equal g, distinct layers, pool ownership, invalid lazy edge, path costs and no-path cases"<<endl;
}catch(const exception&e){cerr<<e.what()<<endl;return 1;}}
