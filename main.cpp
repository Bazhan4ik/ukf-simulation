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

#include "range-localization/range.h"


int main() {
    std::vector<std::vector<double>> data = DataReader::readDataFile("test.txt", ' ');
    std::vector<std::vector<double>> jerkyData = DataReader::readDataFile("more-data.txt", ' ');
    std::vector<std::vector<double>> dataActual = DataReader::readDataFile("z.txt", '\t');
    if (data.empty()) {
        return -1;
    }

    UKF ukf;
    Odometry odom;

    Map m;

    // double dist = m.detectDistance(V2d { 60, 40 }, M_PI + 41.5 * M_PI / 180.0);
    double dist = m.detectDistance(V2d { 60, 27 }, M_PI);
    std::cout << dist << std::endl;



    // ukf.sp.outputParameters();

    std::vector<std::vector<double>> points;
    std::vector<std::vector<double>> points2;






    DesmosExporter::exportXYTrajectory(points, "./../output/odom.txt");
    DesmosExporter::exportXYTrajectory(points2, "./../output/ukft.txt");






    return 1;
};