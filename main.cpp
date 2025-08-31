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

    

    // ukf.x(0) = 0.00289288; // x
    // ukf.x(1) = 0.0002535; // y
    // ukf.x(2) = 0.00019; // theta
    // ukf.x(3) = 0.00289288 / 0.01; // vx
    // ukf.x(4) = 0.0002535 / 0.01; // vy
    // ukf.x(5) = 0.019; // w

    // odom.update({ 0.00289288, 0.0002535, 0.00019 });


    for(size_t i = 0; i < dataActual.size() && i < 20; i++) {
        auto p = dataActual.at(i);
    // for(size_t i = 0; i < data.size() && i < 3; i++) {
    //     auto p = data.at(i);
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

        ukf.update(zd.head<3>());

        std::cout << "\n\nOdom value:" << std::endl;
        formatMatrix(o);

        std::cout << "\n\nError:" << std::endl;
        std::cout << "\tx: " << o(0) - ukf.x(0) << "   (" << (o(0) - ukf.x(0)) / o(0) * 100.0 << "%)" << std::endl;
        std::cout << "\ty: " << o(1) - ukf.x(1) << "   (" << (o(1) - ukf.x(1)) / o(1) * 100.0 << "%)" << std::endl;
        std::cout << "\ttheta: " << o(2) - ukf.x(2) << "   (" << (o(2) - ukf.x(2)) / o(2) * 100.0 << "%)" << std::endl;
        // std::cout << "\ttheta: " << o(2) << " . " << ukf.x(2) << "   (" << (o(2) - ukf.x(2)) / o(2) * 100.0 << "%)" << std::endl;
        std::cout << "\tvx: " << zd(0) / 0.01 - ukf.x(3) << "   (" << (zd(0) / 0.01 - ukf.x(3)) / zd(0)<< "%)" << std::endl;
        std::cout << "\tvy: " << zd(1) / 0.01 - ukf.x(4) << "   (" << (zd(1) / 0.01 - ukf.x(4)) / zd(1)<< "%)" << std::endl;
        std::cout << "\tw: " << zd(2) / 0.01 - ukf.x(5) << "   (" << (zd(2) / 0.01 - ukf.x(5)) / zd(2)<< "%)" << std::endl;

        savePointData(points, points2, o, ukf.x);
    }

    std::cout << "\n\n" << std::endl;
    std::cout << ukf.skipped << std::endl;


    DesmosExporter::exportXYTrajectory(points, "./../output/odom.txt");
    DesmosExporter::exportXYTrajectory(points2, "./../output/ukft.txt");






    return 1;
};