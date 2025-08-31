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

    std::vector<std::vector<double>> points;
    std::vector<std::vector<double>> points2;


    for(size_t i = 0; i < dataActual.size() && i < 20; i++) {
        auto p = dataActual.at(i);
        double dx = p.at(0); double vx = p.at(3);
        double dy = p.at(1); double vy = p.at(4);
        double dtheta = p.at(2); double vtheta = p.at(5);

        V4d zd { dx, dy, dtheta, vtheta };
        V6d zv { dx, dy, dtheta, vx, vy, vtheta };

        auto o = odom.update(zd.head<3>());

        ukf.predict();
        ukf.update(zd.head<3>());

        savePointData(points, points2, o, ukf.x);
    }

    std::cout << "\n\n" << std::endl;

    DesmosExporter::exportXYTrajectory(points, "./../output/odom.txt");
    DesmosExporter::exportXYTrajectory(points2, "./../output/ukft.txt");






    return 1;
};