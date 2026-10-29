#include "KC_path_cost.hpp"
#include <cassert>
#include <limits>
#include <iostream>
int main() {
    PathCostAudit empty; assert(empty.matches(0)); assert(!empty.matches(1));
    PathCostAudit long_path;
    for (int i=0;i<10000;++i) long_path.add(0.1f);
    assert(std::abs(long_path.precise-double(long_path.replay))>.001);
    assert(long_path.matches(double(long_path.replay)));
    assert(!long_path.matches(long_path.precise));
    assert(!long_path.matches(std::nextafter(long_path.replay,std::numeric_limits<float>::infinity())));
    assert(!long_path.matches(std::numeric_limits<double>::infinity()));
    assert(!long_path.matches(std::numeric_limits<double>::quiet_NaN()));
    PathCostAudit integer; integer.add(1);integer.add(2);integer.add(0);
    assert(integer.matches(3) && integer.precise==3);
    std::cout<<"PASS: empty/exact/long float sums, one-ULP corruption, nonfinite costs\n";
}
