#pragma once
#include "../Eigen/Core"


// ukf
using V9d = Eigen::Vector<double, 9>;
using V6d = Eigen::Vector<double, 6>;
using V3d = Eigen::Vector<double, 3>;
using V4d = Eigen::Vector<double, 4>;
using V2d = Eigen::Vector<double, 2>;
// sigma point matrix
using MV9d = Eigen::Matrix<double, 9, 19>;
using MV6d = Eigen::Matrix<double, 6, 19>;
using MV3d = Eigen::Matrix<double, 3, 19>; // no velocity measurement sigma points
using M9d = Eigen::Matrix<double, 9, 9>;
using M6d = Eigen::Matrix<double, 6, 6>;
using M3d = Eigen::Matrix<double, 3, 3>;
using M2d = Eigen::Matrix<double, 2, 2>;
// cross covariance + kalman gain
using MPxyKd = Eigen::Matrix<double, 9, 6>;
using MPxyK3d = Eigen::Matrix<double, 9, 3>; // no velocity kalman gain or cross covariance
// ukf


M3d rotatePose(double phi) {
    double c = cos(phi);
    double s = sin(phi);
    M3d m;
    m <<
        c, s, 0,
        -s, c, 0,
        0, 0, 1;
    return m;
}