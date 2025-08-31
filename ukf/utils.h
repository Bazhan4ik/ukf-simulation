#pragma once
#include "types.h"
#include "../Eigen/Dense"



void opredict(V9d &state, int n) {
    std::cout << "predict:" << std::endl;
    if(n == 10) {
        std::cout << "\tx:" << state(0) << std::endl;
        std::cout << "\tdx:" << state(3) << std::endl;
    }
}

void oerror(const V3d &err, int n) {
    std::cout << "error:" << std::endl;
    if(n == 10) {
        std::cout << "\tx: " << err.x() << std::endl;
    }

}

void oraw(V6d &z, int n) {
    std::cout << "raw:" << std::endl;
    if(n == 1) {
        std::cout << "\tdx: " << z(0) << std::endl;
    }
    if(n == 10) {
        std::cout << "\tdx: " << z(0) << std::endl;
        std::cout << "\tvx: " << z(3) << std::endl;
    }
}

void oodom(V3d &o, int n) {
    std::cout << "odom:" << std::endl;
    if(n < 10) {

    }
    if(n == 10) {
        std::cout << "\tx: " << o.x() << std::endl;
        std::cout << "\ty: " << o.y() << std::endl;
        std::cout << "\ttheta: " << o.z() << std::endl;
    }
}

void ostate(V9d &state, int n) {
    std::cout << "state:" << std::endl;
    if(n < 10) {
        if(n > 0) {
            std::cout << "\ty: " << state(1) << std::endl;
        }
        if(n > 1) {
            std::cout << "\ttheta: " << state(2) << std::endl;
        }
        if(n > 2) {
            std::cout << "\tvx: " << state(3) << std::endl;
        }
        if(n > 3) {
            std::cout << "\tvx: " << state(3) << std::endl;
        }
    }
    
    if(n == 10) {
        std::cout << "\tx: " << state(0) << std::endl;
        std::cout << "\tdx: " << state(3) << std::endl;
        std::cout << "\tvx: " << state(6) << std::endl;
    }

}




void savePointData(std::vector<std::vector<double>> &p1, std::vector<std::vector<double>> &p2, const V3d &o, const V9d &ukf) {
    std::vector<double> x;
    x.emplace_back(o.x());
    x.emplace_back(o.y());

    std::vector<double> y;
    y.emplace_back(ukf(0));
    y.emplace_back(ukf(1));

    p1.emplace_back(x);
    p2.emplace_back(y);
}

void tabMatrix(Mx matrix) {
    for(int i = 0; i < matrix.rows(); i++) {
        std::cout << "\t" << matrix.row(i) << std::endl;
    }
}


static const Eigen::IOFormat fmt(4, 0, "  ", "\n", "\t|", "|");

void formatMatrix(Mx m) {

    std::cout << m.format(fmt) << std::endl;

    // for(int i = 0; i < m.cols(); i++) {
    //     auto sp = m.col(i);
    //     std::cout << "\t(" << i + 1 << ")  ";
    //     std::cout << m.format(fmt);
    // }

}