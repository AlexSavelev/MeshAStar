#pragma once
#include "KC_structs.hpp"
#include "common.hpp"
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
// Spatial directions are NOT indices of robot headings.
// All directions remain fixed throughout the search (consistency of the max).
struct DirectionalPotential {
 static const int N=16;
 std::array<double,N> vi,vj;
 double d[N][NUM_HEADINGS][NUM_HEADINGS];
 double beta;
 explicit DirectionalPotential(ControlSet &cs,double scale):beta(scale){
  if(!std::isfinite(beta)||beta<0)throw std::invalid_argument("Invalid beta");
  const double pi=std::acos(-1.0),inf=std::numeric_limits<double>::infinity();
  for(int v=0;v<N;++v){
   vi[v]=std::sin(2*pi*v/N);vj[v]=std::cos(2*pi*v/N);
   for(int a=0;a<NUM_HEADINGS;++a)for(int b=0;b<NUM_HEADINGS;++b)d[v][a][b]=a==b?0:inf;
   for(auto p:cs.list_all_prims)if(p){
    double r=cs.cost(p->id)-beta*(vi[v]*p->goal->i+vj[v]*p->goal->j);
    if(!std::isfinite(r)||r<0)throw std::invalid_argument("Negative directional residual");
    auto &x=d[v][p->start_theta][p->goal->theta];x=std::min(x,r);
   }
   for(int k=0;k<NUM_HEADINGS;++k)for(int a=0;a<NUM_HEADINGS;++a)for(int b=0;b<NUM_HEADINGS;++b)
    d[v][a][b]=std::min(d[v][a][b],d[v][a][k]+d[v][k][b]);
  }
 }
 double value(int goal_di,int goal_dj,int heading,int goal_heading)const{
  if(heading<0||heading>=NUM_HEADINGS||goal_heading<0||goal_heading>=NUM_HEADINGS)throw std::invalid_argument("Invalid heading");
  double result=0;
  for(int v=0;v<N;++v)result=std::max(result,beta*(vi[v]*goal_di+vj[v]*goal_dj)+d[v][heading][goal_heading]);
  return result;
 }
};
