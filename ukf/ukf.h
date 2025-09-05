#include "sigma.h"
#include "matrixSqrt.h"
#include "posdef.h"
#include "types.h"
#include "utils.h"
#include "../range-localization/range.h"




class UKF {
public:
    // state
    V6d x;
        // global position: x, y, theta
        // global displacementsn: dx, dy, dtheta

    // sigma points
    SigmaPoints sp;

    double dt = 0.01; // epoch in seconds

    M6d P;
    M6d Q;

    // measurement model:
    //      dx, dy, dtheta
    //      distance sensor x3
    M6d R;

    Map m;

    UKF() {
        P = M6d::Identity() * 0.09;
        Q = M6d::Identity() * 0.0001;

        // double std = 0.005;
        double std = 0.001;



        Q(0,0) = pow(std, 2);
        Q(1,1) = pow(std, 2);
        Q(3,3) = pow(std, 2);
        Q(4,4) = pow(std, 2);

        Q(2,2) = P(2, 2) = pow(0.0004, 2);
        Q(5, 5) = P(5, 5) = pow(0.0004, 2);

        R = M6d::Zero();
        R.diagonal() <<
            1e-4, 1e-4, 1e-8, 1e-1, 1e-1, 1e-1;
        x <<
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0;
    };


    /*
    
        ===========
        UPDATE STEP
        ===========
    
    */
    auto
    update(const V6d &z) {
        auto sigmaPts = generateSigmaPoints();

        V6d meanZ = V6d::Zero();

        MV6d sigmasZ = MV6d::Zero();
        for(int i = 0; i < sp.nps; i++) {
            sigmasZ.col(i) = measurementModel(sigmaPts.col(i));
            meanZ += sp.getw(i) * sigmasZ.col(i);
        }

        M6d Pz = R;
        M6d Pxy = M6d::Zero();
        for(int i = 0; i < sp.nps; i++) {
            V6d error = sigmasZ.col(i) - meanZ;
            Pz += sp.getPw(i) * error * error.transpose();
            V6d meanError = sigmaPts.col(i) - x;
            Pxy += sp.getPw(i) * meanError * error.transpose();
        }


        Pz = ensurePositiveDefinite(Pz);
        Pz = 0.5 * (Pz + Pz.transpose());

        V6d y = z - meanZ;
        // std::cout << "Innovation: ";
        // formatMatrix(y.transpose());
        M6d K = computeKalmanGain(Pxy, Pz);

        x += K * y;


        M6d KPzKt = K * Pz * K.transpose();
        M6d newP = P - KPzKt;
        newP = 0.5 * (newP + newP.transpose());

        P = ensurePositiveDefinite(newP);
    }


    V6d measurementModel(const V6d &point) {
        V3d globalVelocities = point.tail<3>();

        V3d localDisplacements = rotatePose(-(point(2) - globalVelocities(2))) * globalVelocities;
        // V3d localDisplacements = rotatePose(-(point(2) - globalVelocities(2) * dt)) * globalVelocities * dt;
        

        double d1 = m.detectDistance(x.head<2>(), x(2) + M_PI / 2.0).first;
        double d2 = m.detectDistance(x.head<2>(), x(2) + M_PI).first;
        double d3 = m.detectDistance(x.head<2>(), x(2) - M_PI / 2.0).first;
        


        V6d result;
        result <<
            localDisplacements(0), localDisplacements(1), localDisplacements(2),
            d1, d2, d3;
        
        return result;
    }


    /*
    
        ============
        PREDICT STEP
        ============
    
    */
    void
    predict() {
        auto sigmaPoints = this->generateSigmaPoints();

        // transformed sigma points;
        MV6d tsp;

        V6d predictedMean = V6d::Zero();
        for(int i = 0; i < sp.nps; i++) {
            tsp.col(i) = this->processModel(sigmaPoints.col(i));
            predictedMean += sp.getw(i) * tsp.col(i);
        }

        M6d predictedCovariance = Q;
        for(int i = 0; i < sp.nps; i++) {
            auto error = tsp.col(i) - predictedMean;
            predictedCovariance += sp.getPw(i) * error * error.transpose();
        }

        predictedCovariance = 0.5 * (predictedCovariance + predictedCovariance.transpose());
        predictedCovariance = ensurePositiveDefinite(predictedCovariance);

        x = predictedMean;
        P = predictedCovariance;
    }

    V6d processModel(const V6d &point) {
        V3d globalD = point.tail<3>();

        double deltaTheta = globalD(2);

        double c;
        double s;
        if(fabs(deltaTheta) < 0.00005) { // approximate trig if theta if too small
            // CHECK THE TRANSFORM MATRICES ANDE THREI SIGNS
            c = std::pow(deltaTheta, 2) / 6.0 - 1.0;
            s = dt * deltaTheta / 2.0;
            // c = std::pow(deltaTheta, 2) * dt / 6.0 - dt;
            // s = dt * deltaTheta / 2.0;
        } else {
            // c = sin(deltaTheta) / globalVelocities(2);
            // s = (1.0 - cos(deltaTheta)) / globalVelocities(2);
            c = sin(deltaTheta) / deltaTheta;
            s = (1.0 - cos(deltaTheta)) / deltaTheta;
        }

        M3d exponentiate;
        exponentiate <<
            c, -s, 0,
            s, c, 0,
            0, 0, 1;
            // 0, 0, dt;

        V3d newGlobalPosition = exponentiate * globalD + point.head<3>();

        V6d result;
        result <<
            // new position calculated from spoint's velocities
            newGlobalPosition(0), newGlobalPosition(1), newGlobalPosition(2),
            // local deltas are what velocity if moved dt
            globalD(0), globalD(1), globalD(2);

        return result;
    }

    MV6d generateSigmaPoints() {
        MV6d result = MV6d::Zero();

        M6d scaledSqrt = matrixSqrt(sp.scale * P);

        result.col(0) = x;
        for(int i = 1; i < sp.n + 1; i++) {
            result.col(i) = x + scaledSqrt.col(i - 1);
            result.col(i + sp.n) = x - scaledSqrt.col(i - 1);
        }

        return result;
    }

    M6d computeKalmanGain(const M6d& Pxy, const M6d& Pz) {
        Eigen::FullPivLU<M6d> lu(Pz);
        if (!lu.isInvertible()) {
            // regularize
            M6d regularized = Pz + 1e-6 * M6d::Identity();
            return Pxy * regularized.inverse();
        }
        return Pxy * lu.inverse();
    }
};