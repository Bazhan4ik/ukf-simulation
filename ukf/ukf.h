#include "sigma.h"
#include "matrixSqrt.h"
#include "posdef.h"
#include "types.h"




class UKF {
public:
    // state
    V9d x;
        // global position: x, y, theta
        // local displacement: dx, dy, dtheta
        // local velocity: vx, vy, w
    // sigma points
    SigmaPoints sp;

    double dt = 0.01; // epoch in seconds

    M9d P;
    M9d Q;
    M6d R;

    UKF() {
        P = M9d::Identity();
        P.diagonal() <<
            0.01, 0.01, 0.01,
            0.1, 0.1, 0.1,
            0.5, 0.5, 0.5;
        // P.block<3, 3>(0, 0) *= 0.01; // displacement initial covariance = 0.5
        // P.block<3, 3>(3, 3) *= 0.5; // displacement initial covariance = 0.5
        // P.block<3, 3>(6, 6) *= 1.0; // velocity initial covariance = 1
        Q = M9d::Zero();
        Q.diagonal() <<
            0.1, 0.1, 0.1,
            0.0001, 0.0001, 0.0001,
            0.0001, 0.0001, 0.0001;
        R = M6d::Zero();
        R.diagonal() <<
            0.00003, 0.00003, 0.00003,
            0.00003, 0.00003, 0.00003;
        x <<
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0;
            // 20.0, -5.0, M_PI / 18.0,
            // 2.2, -0.4, M_PI / 36.0,
            // 8.5, 1.2, M_PI / 4.0;
    };

    bool useVelocity = true;


    /*
    
        ===========
        UPDATE STEP
        ===========
    
    */
    auto
    updatev(const V6d &z) {
        useVelocity = true;
        auto sigmaPts = generateSigmaPoints();

        V6d meanZ = V6d::Zero();

        MV6d sigmasZ = MV6d::Zero();
        for(int i = 0; i < sp.nps; i++) {
            sigmasZ.col(i) = measurementModel(sigmaPts.col(i));
            meanZ += sp.getw(i) * sigmasZ.col(i);
        }

        M6d Pz = R;
        MPxyKd Pxy = MPxyKd::Zero();
        for(int i = 0; i < sp.nps; i++) {
            V6d error = sigmasZ.col(i) - meanZ;
            Pz += sp.getPw(i) * error * error.transpose();
            V9d meanError = sigmaPts.col(i) - x;
            Pxy += sp.getPw(i) * meanError * error.transpose();
        }



        Pz = 0.5 * (Pz + Pz.transpose());
        Pz = ensurePositiveDefinite(Pz);


        V6d y = z - meanZ;
        MPxyKd K = computeKalmanGain(Pxy, Pz);

        x += K * y;


        M9d KPzKt = K * Pz * K.transpose();
        M9d newP = P - KPzKt;
        newP = 0.5 * (newP + newP.transpose());

        P = ensurePositiveDefinite(newP);

        // std::cout << x.transpose() << std::endl;
        std::cout << x(0) << "\t" << x(3) << "\t" << x(6) << std::endl;
        // std::cout << P << std::endl << std::endl;

        return x;
    }
    // no velocity update
    auto
    updated(const V3d &z) {
        useVelocity = false;
        auto sigmaPts = generateSigmaPoints();

        V3d meanZ = V3d::Zero();

        MV3d sigmasZ = MV3d::Zero();
        for(int i = 0; i < sp.nps; i++) {
            sigmasZ.col(i) = measurementModelDisplacement(sigmaPts.col(i));
            meanZ += sp.getw(i) * sigmasZ.col(i);
        }

        M3d Pz = R.block<3, 3>(0, 0);
        MPxyK3d Pxy = MPxyK3d::Zero();
        for(int i = 0; i < sp.nps; i++) {
            V3d error = sigmasZ.col(i) - meanZ;
            Pz += sp.getPw(i) * error * error.transpose();
            V9d meanError = sigmaPts.col(i) - x;
            Pxy += sp.getPw(i) * meanError * error.transpose();
        }

        Pz = 0.5 * (Pz + Pz.transpose());
        Pz = ensurePositiveDefinite(Pz);
        
        
        V3d y = z - meanZ;
        MPxyK3d K = computeKalmanGain(Pxy, Pz);

        x += K * y;


        M9d KPzKt = K * Pz * K.transpose();
        M9d newP = P - KPzKt;
        newP = 0.5 * (newP + newP.transpose());

        P = ensurePositiveDefinite(newP);

        // std::cout << x(0) << "\t" << x(3) << "\t" << x(6) << std::endl;
        // std::cout << P << std::endl << std::endl;

        return x;
    }



    V6d measurementModel(const V9d &point) {
        V3d localDisplacement = point.segment<3>(3);
        V3d localVelocity = point.segment<3>(6);

        return point.tail<6>();
    }
    // if velocity was not updated
    V3d measurementModelDisplacement(const V9d &point) {
        V3d localDisplacement = point.segment<3>(3);
        V3d localVelocity = point.segment<3>(6);

        return point.segment<3>(3);
    }


    /*
    
        ============
        PREDICT STEP
        ============
    
    */
    V9d
    predict() {
        auto sigmaPoints = this->generateSigmaPoints();

        // transformed sigma points;
        MV9d tsp;

        V9d predictedMean = V9d::Zero();
        for(int i = 0; i < sp.nps; i++) {
            tsp.col(i) = this->processModel(sigmaPoints.col(i));
            predictedMean += sp.getw(i) * tsp.col(i);
        }

        M9d predictedCovariance = Q;
        for(int i = 0; i < sp.nps; i++) {
            auto error = tsp.col(i) - predictedMean;
            predictedCovariance += sp.getPw(i) * error * error.transpose();
        }

        predictedCovariance = 0.5 * (predictedCovariance + predictedCovariance.transpose());
        predictedCovariance = ensurePositiveDefinite(predictedCovariance);

        x = predictedMean;
        P = predictedCovariance;

        return x;
    }

    V9d processModel(const V9d &point) {
        V3d localDelta = point.segment<3>(3).eval();
        // if(useVelocity) localDelta = point.segment<3>(6).eval() * dt; // if velocity was updated use velocity * time
        // else localDelta = point.segment<3>(3).eval(); // else use constant displacement


        double c;
        double s;
        if(localDelta.z() < 0.0005) { // approximate trig if theta if too small
            c = 1.0 - std::pow(localDelta.z(), 2) / 6.0;
            s = -localDelta.z() / 2.0;
        } else {
            c = sin(localDelta.z()) / localDelta.z();
            s = (cos(localDelta.z()) - 1.0) / localDelta.z();
        }

        M3d exponentiate;
        exponentiate <<
            c, s, 0,
            -s, c, 0,
            0, 0, 1;

        V3d newGlobalPosition = rotatePose(point.z()) * exponentiate * localDelta + point.head<3>();

        V9d result;
        result <<
            // new position calculated from spoint's velocities
            newGlobalPosition(0), newGlobalPosition(1), newGlobalPosition(2),
            // local deltas are what velocity if moved dt
            localDelta(0), localDelta(1), localDelta(2),
            // keep velocities constant
            point(6), point(7), point(8);

        return result;
    }

    MV9d generateSigmaPoints() {
        MV9d result = MV9d::Zero();

        M9d scaledSqrt = matrixSqrt(sp.scale * P);

        result.col(0) = x;
        for(int i = 1; i < sp.n + 1; i++) {
            result.col(i) = x + scaledSqrt.col(i - 1);
            result.col(i + sp.n) = x - scaledSqrt.col(i - 1);
        }

        return result;
    }

    MPxyKd computeKalmanGain(const MPxyKd& Pxy, const M6d& Pz) {
        Eigen::FullPivLU<M6d> lu(Pz);
        if (!lu.isInvertible()) {
            // regularize
            M6d regularized = Pz + 1e-6f * M6d::Identity();
            return Pxy * regularized.inverse();
        }
        return Pxy * lu.inverse();
    }

    MPxyK3d computeKalmanGain(const MPxyK3d& Pxy, const M3d& Pz) {
        Eigen::FullPivLU<M3d> lu(Pz);
        if (!lu.isInvertible()) {
            // regularize
            M3d regularized = Pz + 1e-6f * M3d::Identity();
            return Pxy * regularized.inverse();
        }
        return Pxy * lu.inverse();
    }
};