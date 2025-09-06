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
    // std::vector<std::vector<double>> data = DataReader::readDataFile("test.txt", ' ');
    // std::vector<std::vector<double>> jerkyData = DataReader::readDataFile("newdata.txt", ' ');
    std::vector<std::vector<double>> data = DataReader::readDataFile("weird-data.txt", ' ');
    // std::vector<std::vector<double>> dataActual = DataReader::readDataFile("z.txt", '\t');

    UKF ukf;
    Odometry odom;

    ukf.sp.outputParameters();

    std::vector<std::vector<double>> points;
    std::vector<std::vector<double>> points2;

    // odom.current = V3d { 23.25, 23.25, M_PI / 2.0 };
    // ukf.x = V6d { 23.25, 23.25, M_PI / 2.0, 0.0, 0.0, 0.0 };

    // odom.update({ initial.at(0), initial.at(1), initial.at(2) + M_PI / 2.0 });
    int dist = 0;
    for(size_t i = 0; i < data.size() && i < 1; i++) {
        auto p = data.at(i);
        double dx = p.at(0);
        double dy = p.at(1);
        double dtheta = p.at(2);

        V3d zd { dx, dy, dtheta };

        auto o = odom.update(zd);

        ukf.predict();

        ukf.update(zd);

        // std::cout << std::endl;
        // formatMatrix(o.transpose());
        // formatMatrix(ukf.x.transpose());


        if(dist++ % 5 == 0) {
            savePointData(points, points2, o, ukf.x);
        }
    }


    std::cout << "\n\nEnd: " << std::endl;
    formatMatrix(odom.current.transpose());
    formatMatrix(ukf.x.transpose());

    DesmosExporter::exportXYTrajectory(points, "./../output/odom.txt");
    DesmosExporter::exportXYTrajectory(points2, "./../output/ukft.txt");






    return 1;
};