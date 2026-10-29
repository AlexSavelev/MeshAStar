#include "KC_algorithms.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
MyHEAP *HEAP=nullptr;
int main(){
    try {
        MyHEAP heap;HEAP=&heap;ControlSet cs;cs.load_primitives("data/base_control_set.txt");
        HeuristicCalibration calibration(&cs);
        ofstream out("res/alpha-experiment/table-bounds.jsonl");out<<setprecision(12);
        if(!out)throw runtime_error("Cannot write bounds");
        for(double alpha:{.3,.6,.75,.9}) {
            auto costs=calibration.residual_costs(alpha);
            for(int goal:{0,3,8,13}) {
                KinematicTable table(&cs,goal,64,&costs);double maximum=0;
                for(int i=0;i<64;++i)for(int j=0;j<64;++j)for(int theta=0;theta<16;++theta){
                    double h=table.value(i,j,theta);
                    if(!std::isfinite(h))throw runtime_error("Infinite table invalidates finite upper bound");
                    maximum=max(maximum,h);
                }
                out<<"{\"alpha\":"<<alpha<<",\"goal_heading\":"<<goal<<",\"max_residual_h\":"<<maximum<<",\"grid_distance_cutoff\":";
                if(alpha<.9)out<<maximum/(.9-alpha);else out<<"null";
                out<<"}\n";
            }
        }
        cout<<"PASS: maxima over all abstract states, 16 residual tables"<<endl;
    }catch(const exception &e){cerr<<e.what()<<endl;return 1;}
}
