#pragma once
#include "../Eigen/Core"


// ukf
using V9d = Eigen::Vector<double, 9>; // measurement vector
using V6d = Eigen::Vector<double, 6>; // state vector
using V4d = Eigen::Vector<double, 4>; // current measurement vector with no distance and velocities

using M6d = Eigen::Matrix<double, 6, 6>; // Q, P
// using M9d = Eigen::Matrix<double, 9, 9>;    // R
using M3d = Eigen::Matrix<double, 3, 3>;    // R
                                            // right now only 4 displacements and gyro, but then velocities and ultrasonic ranges

using MV6d = Eigen::Matrix<double, 6, 19>; // sigma points
using MV3d = Eigen::Matrix<double, 3, 19>; // measurements from sigma pts


using M6x3 = Eigen::Matrix<double, 6, 3>;   // Kalman gain
                                            // cross covariance





using V3d = Eigen::Vector<double, 3>;
using V2d = Eigen::Vector<double, 2>;
// sigma point matrix
using MV9d = Eigen::Matrix<double, 9, 19>;
using M6d = Eigen::Matrix<double, 6, 6>;
using M2d = Eigen::Matrix<double, 2, 2>;
// cross covariance + kalman gain
using MPxyKd = Eigen::Matrix<double, 9, 6>;
using MPxyK3d = Eigen::Matrix<double, 9, 3>; // no velocity kalman gain or cross covariance
// ukf

using Mx = Eigen::MatrixXd;


M3d rotatePose(double phi) {
    double c = cos(phi);
    double s = sin(phi);
    M3d m;
    // counterclockwise
    m <<
        c, -s, 0,
        s, c, 0,
        0, 0, 1;
    return m;
}