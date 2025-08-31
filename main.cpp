#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include "reader.cpp"

#include "ukf/ukf.h"
#include "ukf/types.h"
#include "ukf/odom.h"
#include "ukf/utils.h"


int main() {
    std::vector<std::vector<double>> data = DataReader::readDataFile("test.txt", ' ');
    std::vector<std::vector<double>> dataActual = DataReader::readDataFile("z.txt", '\t');
    if (data.empty()) {
        return -1;
    }

    UKF ukf;
    Odometry odom;

    double lastV = 0.0;

    std::vector<std::vector<double>> points;
    std::vector<std::vector<double>> points2;

    ukf.x(0) = 0.00289288;
    ukf.x(3) = 0.00289288 / 0.01;

    odom.update({ 0.00289288, 0.0, 0.0 });



    for(size_t i = 0; i < data.size() && i < 20; i++) {
        auto p = data.at(i);
        double dx = p.at(0); double vx = p.at(3);
        double dy = p.at(1); double vy = p.at(4);
        double dtheta = p.at(2); double vtheta = p.at(5);

        V4d zd { dx, dy, dtheta, vtheta };
        V6d zv { dx, dy, dtheta, vx, vy, vtheta };


        std::cout << "\n\n\n==============" << std::endl;
        std::cout << "==============" << std::endl;
        std::cout << i << "  ========  " << i << std::endl;
        std::cout << "==============" << std::endl;
        std::cout << "==============" << std::endl;


        auto o = odom.update(zd.head<3>());

        ukf.predict();

        std::cout << "\n\nState after predict: " << std::endl;
        formatMatrix(ukf.x);

        // std::cout << "\n\nCovariance (P) after predict: " << std::endl;
        // formatMatrix(ukf.P);

        ukf.update(zd);

        std::cout << "\n\nOdom value:" << std::endl;
        formatMatrix(o);

        std::cout << "\n\nError:" << std::endl;
        std::cout << "\tx: " << o(0) - ukf.x(0) << std::endl;
        std::cout << "\tvx: " << zd(0) / 0.01 - ukf.x(3) << std::endl;

        savePointData(points, points2, o, V9d::Zero());
    }

    std::cout << "\n\n" << std::endl;


    DesmosExporter::exportXYTrajectory(points, "./../output/odom.txt");
    DesmosExporter::exportXYTrajectory(points2, "./../output/ukft.txt");






    return 1;
};