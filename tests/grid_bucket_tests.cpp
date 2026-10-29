#include "KC_heuristic.hpp"
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>

MyHEAP *HEAP = nullptr;
static uint64_t comparisons=0, fields=0, heuristics=0;
static void require(bool ok, const char *message) { if (!ok) throw std::runtime_error(message); }
static void compare(Map &map, int gi, int gj, float radius) {
    std::vector<double> a,b;
    grid_field::build(map,gi,gj,radius,GridQueueMode::Heap,a);
    grid_field::build(map,gi,gj,radius,GridQueueMode::Buckets,b);
    require(a.size()==b.size(),"Field size mismatch");
    for (size_t i=0;i<a.size();++i) {
        require(a[i]==b[i],"Heap/bucket field mismatch"); ++comparisons;
    }
    ++fields;
}
int main(int argc, char **argv) {
    MyHEAP heap; HEAP=&heap;
    Map map;std::mt19937 rng(20261029);
    // Random disconnected maps, one-cell grids, all blocked / no source,
    // region goals (multiple zero sources), and repeated ring wraparound.
    for(int n=0;n<80;++n) {
        int h=1+rng()%70,w=1+rng()%70;
        map.empty_map(w,h);
        for(int i=0;i<map.height;++i)for(int j=0;j<map.width;++j)
            map.cells[i][j]=(n%4==0 || (n%4!=1 && rng()%4==0));
        for(float r:{0.01f,2.5f})compare(map,map.height/2,map.width/2,r);
    }
    map.empty_map(1000,2);compare(map,0,0,.01f);
    require(fields==161,"Missing synthetic fields");
    ControlSet cs;cs.load_primitives("data/base_control_set.txt");
    map.empty_map(32,32);Vertex goal(24,24,3);
    for(auto mode:{HeuristicMode::Max,HeuristicMode::Directional}) {
        LatticeHeuristic a(&map,&goal,.01f,&cs,mode,.9,nullptr,64,0,GridQueueMode::Heap);
        LatticeHeuristic b(&map,&goal,.01f,&cs,mode,.9,nullptr,64,0,GridQueueMode::Buckets);
        for(int i=0;i<32;++i)for(int j=0;j<32;++j)for(int t=0;t<16;++t) {
            require(a.value(i,j,t)==b.value(i,j,t),"Heuristic mismatch");++heuristics;
        }
    }
    // Optional actual fixed query table, all goals (not just a pilot).
    if(argc==2) {
        std::ifstream in(argv[1]);require(bool(in),"Missing tasks TSV");
        std::string line,id,path,last;int si,sj,st,gi,gj,gt;
        while(std::getline(in,line)) {
            if(line.empty() || line[0]=='#')continue;
            std::istringstream row(line);
            if(!(row>>id>>path>>si>>sj>>st>>gi>>gj>>gt))continue;
            if(path!=last){map=Map();map.read_file_to_cells(path);last=path;}
            compare(map,gi,gj,.01f);
        }
    }
    std::cout<<"BUCKET CHECK fields="<<fields<<" exact_cell_comparisons="<<comparisons
             <<" exact_heuristic_comparisons="<<heuristics<<" failures=0\n";
}
