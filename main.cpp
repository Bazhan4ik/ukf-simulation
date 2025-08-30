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


// Example usage
void main() {
    std::vector<std::vector<double>> data = DataReader::readDataFile("data.txt");
    if (data.empty()) {
        return;
    }

    UKF ukf;
    Odometry odom;

    double lastV = 0.0;

    std::vector<std::vector<double>> points;
    std::vector<std::vector<double>> points2;

    for(size_t i = 10; i < data.size() && i < 15; i++) {
        auto p = data.at(i);
        double dx = p.at(0); double vx = p.at(3);
        double dy = p.at(1); double vy = p.at(4);
        double dtheta = p.at(2); double vtheta = p.at(5);

        V4d zd { dx, dy, dtheta, vtheta };
        V6d zv { dx, dy, dtheta, vx, vy, vtheta };

        auto o = odom.update(zd.head<3>());
        ukf.updated(zd.head<3>());
        ukf.predict();



        std::cout << "\n" << std::endl;
        std::cout << (i) << std::endl;
        // std::cout << ukf.P << std::endl;
        std::cout << "\nstate: " << ukf.x.transpose() << std::endl;
        std::cout << "odom :" << o(0) << "\t" << o(1) << "\t" << o(2) << std::endl;
        std::cout << "error :" << o(0) - ukf.x(0) << "\t" << o(1) - ukf.x(1) << "\t" << o(2) - ukf.x(2) << std::endl;
        std::cout << "\n" << std::endl;



        savePointData(points, points2, o, ukf.x);
    }


    // DesmosExporter::exportXYTrajectory(points);
    // DesmosExporter::exportXYTrajectory(points2, "ukft.txt");






    return;
};