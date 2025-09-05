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





double padeApprox(double x) {
    return 0.5 * (1.0 + std::tanh(std::sqrt(2.0 / M_PI) * x * (1.0 + 0.0448464638169 * x * x)));
}

const double zmax = 100.0; // inches
const double rho = 0.02;
const double cov = 0.8;
const double arand = 0.2;

const double invSqrt2piCov = 1.0 / (std::sqrt(2.0 * M_PI) * cov);
const double negHalfOverCovSquared = -0.5 / std::pow(cov, 2);


double ultrasonicProbabilityModel(double zhit, double z) {


    double phit = 0;
    double difference = z - zhit;
    if(z > 0 && zmax > z) phit = invSqrt2piCov * std::exp(negHalfOverCovSquared * difference * difference);
    double pmax = z > zmax ? 1 : 0;
    double prand = 0;
    if(0 <= z && zmax >= z) prand = 0.01;
    double pshort = 0;
    if(0 <= z && zmax >= z) pshort = rho * std::exp(-rho * z);

    double ahit = std::exp(-rho * zhit) * (1 - arand);
    double amax = ahit * padeApprox((zhit - zmax) / cov);
    double ashort = 1 - arand;

    return ahit * phit + ashort * pshort + amax * pmax + arand * prand;
}


double rangeSensorVariance(double zhit, double z) {
    double p = ultrasonicProbabilityModel(zhit, z);
    
    double distanceFactor;
    if (zhit < 40.0) {
        distanceFactor = 0.1;  // Makes variance 10x smaller than base
    }
    else if (zhit < 80.0) {
        double t = (z - 10.0) / 40.0;  // 0 to 1
        distanceFactor = 0.1 + t * (1.0 - 0.1);  // 0.1 to 1.0
    }
    else {
        // Long range: trust wheels more
        distanceFactor = 2.0;  // Makes variance 2x larger than base
    }
    
    // Base scaling to match your wheel variance scale
    double var = 0.01;  // Similar to wheel variance at medium confidence
    
    return var * distanceFactor / p;
}


// Sensor dist1 (
//     'A', 'B', 6.3125, M_PI / 2.0
// );
// Sensor dist2 (
//     'C', 'D', 6.25, M_PI
// );
// Sensor dist3 (
//     'E', 'F', 6.3125, -M_PI / 2.0
// );



int main() {
    std::vector<std::vector<double>> data = DataReader::readDataFile("test.txt", ' ');
    std::vector<std::vector<double>> jerkyData = DataReader::readDataFile("more-data.txt", ' ');
    std::vector<std::vector<double>> dataActual = DataReader::readDataFile("z.txt", '\t');
    std::vector<std::vector<double>> distanceData = DataReader::readDataFile("distance-data.txt", ' ');
    if (data.empty()) {
        return -1;
    }

    UKF ukf;
    Odometry odom;

    odom.current = V3d { 23.25, 23.25, M_PI / 2.0 };
    ukf.x = V6d { 23.25, 23.25, M_PI / 2.0, 0.0, 0.0, 0.0 };

    Map m;

    std::vector<std::vector<double>> points;
    std::vector<std::vector<double>> points2;


    for(int i = 0; i < distanceData.size() && i < 5; i++) {
        auto p = distanceData.at(i);
        auto dx = p.at(0); auto dy = p.at(1); auto dtheta = p.at(2);
        auto s1 = p.at(3); auto s2 = p.at(4); auto s3 = p.at(5);

        V3d z { dx, dy, dtheta };
        odom.update(z);

        // find robot position, given distance sensor measurements
        
        // pointed at the wall
        auto mz = m.detectDistance(odom.current.head<2>(), odom.current(2) + M_PI / 2.0);
        double zhit = mz.first;
        s1 += 6.3125;
        auto mz2 = m.detectDistance(odom.current.head<2>(), odom.current(2) + M_PI);
        double zhit2 = mz2.first;
        s2 += 6.25;
        auto mz3 = m.detectDistance(odom.current.head<2>(), odom.current(2) - M_PI / 2.0);
        double zhit3 = mz3.first;
        s3 += 6.3125;



        std::cout << "\nZ hit:\t" << zhit << "\t\t";
        std::cout << zhit2 << "\t";
        std::cout << zhit3 << std::endl;

        std::cout << "Z:\t" << s1 << "\t\t";
        std::cout << s2 << "\t";
        std::cout << s3 << std::endl;

        // std::cout << "p(z, zhit):\t" << rangeSensorVariance(zhit, s1) << "\t\t";
        // std::cout << rangeSensorVariance(zhit2, s2) << "\t";
        // std::cout << rangeSensorVariance(zhit3, s3) << std::endl;

        std::cout << "Points: " << std::endl;
        std::cout << "\t"; formatMatrix(mz.second.transpose());
        std::cout << "\t"; formatMatrix(mz2.second.transpose());
        std::cout << "\t"; formatMatrix(mz3.second.transpose());

        std::cout << "Pose: ";
        formatMatrix(odom.current.transpose());








        

        // ukf.predict();

        // ukf.update(z);







        odom.update(z);
        if(i % 4 == 0) {
            savePointData(points, points2, odom.current, ukf.x);
        }
    }




    DesmosExporter::exportXYTrajectory(points, "./../output/odom.txt");
    DesmosExporter::exportXYTrajectory(points2, "./../output/ukft.txt");






    return 1;
};