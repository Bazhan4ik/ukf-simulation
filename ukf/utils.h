#include "types.h"


void savePointData(std::vector<std::vector<double>> &p1, std::vector<std::vector<double>> &p2, const V3d &o, const V6d &ukf) {
    std::vector<double> x;
    x.emplace_back(o.x());
    x.emplace_back(o.y());

    std::vector<double> y;
    y.emplace_back(ukf(0));
    y.emplace_back(ukf(1));

    p1.emplace_back(x);
    p2.emplace_back(y);
}