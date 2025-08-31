#include "sigma.h"
#include "matrixSqrt.h"
#include "posdef.h"
#include "types.h"
#include "utils.h"




class UKF {
public:
    // state
    V6d x;
        // global position: x, y, theta
        // global velocities: vx, vy, w

    // sigma points
    SigmaPoints sp;

    double dt = 0.01; // epoch in seconds

    M6d P;
    M6d Q;

    // measurement model:
        // displacements
        // dx, dy, dtheta
        // actual velocities (vx vy only every 40ms)
        // vx, vy, w
        // distance sensors range
        // dl, dr, db
    M3d R;

    UKF() {
        P = M6d::Identity() * 0.0000009;
        P(3, 3) = 0.01; // bigger initial covariance for velocities
        P(4, 4) = 0.01;

        P(3, 3) = 0.0001; // bigger initial covariance for velocities
        P(4, 4) = 0.0001;

        // P(5, 5) = 0.02;
        // P(2, 2) = 2e-6;
        P(5, 5) = 1e-6;
        P(2, 2) = 1e-6;

        R = M3d::Zero();
        R.diagonal() <<
            7.5e-7, 7.5e-7, 1e-16;
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
    update(const V3d &z) {
        auto sigmaPts = generateSigmaPoints();

        V3d meanZ = V3d::Zero();

        MV3d sigmasZ = MV3d::Zero();
        for(int i = 0; i < sp.nps; i++) {
            sigmasZ.col(i) = measurementModel(sigmaPts.col(i));
            meanZ += sp.getw(i) * sigmasZ.col(i);
        }

        M3d Pz = R.eval();
        M6x3 Pxy = M6x3::Zero();
        for(int i = 0; i < sp.nps; i++) {
            V3d error = sigmasZ.col(i) - meanZ;
            Pz += sp.getPw(i) * error * error.transpose();
            V6d meanError = sigmaPts.col(i) - x;
            Pxy += sp.getPw(i) * meanError * error.transpose();
        }


        Pz = ensurePositiveDefinite(Pz);
        Pz = 0.5 * (Pz + Pz.transpose());

        V3d y = z - meanZ;
        M6x3 K = computeKalmanGain(Pxy, Pz);

        x += K * y;

        M6d KPzKt = K * Pz * K.transpose();
        M6d newP = P - KPzKt;
        newP = 0.5 * (newP + newP.transpose());

        P = ensurePositiveDefinite(newP);

        return x;
    }


    V3d measurementModel(const V6d &point) {
        // converts global velocity into
        // local displacement and angular velocity 
        V3d globalVelocities = point.tail<3>();


        double deltaTheta = globalVelocities(2) * dt; // find local orientation change

        
        double c;
        double s;
        if(fabs(deltaTheta) < 0.00005) { // approximate trig if theta if too small
            c = dt - std::pow(deltaTheta, 2) * dt / 6.0;
            s = -dt * deltaTheta / 2.0;
        } else {
            c = sin(deltaTheta) / globalVelocities(2);
            s = (cos(deltaTheta) - 1.0) / globalVelocities(2);
        }

        M3d exponentiate;
        exponentiate <<
            c, s, 0,
            -s, c, 0,
            0, 0, dt;

        V3d localDisplacements = exponentiate * globalVelocities;

        V3d result;
        result <<
            localDisplacements(0), localDisplacements(1), localDisplacements(2);
        
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

        M6d predictedCovariance = M6d::Zero();
        for(int i = 0; i < sp.nps; i++) {
            auto error = tsp.col(i) - predictedMean;
            predictedCovariance += sp.getPw(i) * error * error.transpose();
        }
        predictedCovariance += computeQ(predictedCovariance);

        predictedCovariance = 0.5 * (predictedCovariance + predictedCovariance.transpose());
        predictedCovariance = ensurePositiveDefinite(predictedCovariance);

        x = predictedMean;
        P = predictedCovariance;
    }

    M6d computeQ(M6d &cov) {
        M6d Q = M6d::Identity() * 0.000009;
        // Q(2, 2) = 2e-6;
        // Q(5, 5) = 0.02; // inertial std ~0.0005
        Q(5, 5) = 0.0002; // inertial std ~0.0005
        Q(3, 3) = Q(4, 4) = 0.0625; // global velocity std ~0.2
        Q(0, 3) = Q(3, 0) = 0.95 * std::sqrt(P(0, 0) * P(3, 3)); // keep correlation ~ 95%
        Q(1, 4) = Q(4, 1) = 0.95 * std::sqrt(P(1, 1) * P(4, 4)); // keep correlation ~ 95%
        Q(2, 5) = Q(5, 2) = 1.0 * std::sqrt(P(2, 2) * P(5, 5)); // keep correlation ~ 95%

        return Q;
    }

    V6d processModel(const V6d &point) {
        V3d globalVelocities = point.tail<3>();

        double deltaTheta = globalVelocities(2) * dt;

        double c;
        double s;
        if(fabs(deltaTheta) < 0.00005) { // approximate trig if theta if too small
            c = dt - std::pow(deltaTheta, 2) * dt / 6.0;
            s = -dt * deltaTheta / 2.0;
        } else {
            c = sin(deltaTheta) / globalVelocities(2);
            s = (cos(deltaTheta) - 1.0) / globalVelocities(2);
        }

        M3d exponentiate;
        exponentiate <<
            c, -s, 0,
            s, c, 0,
            0, 0, dt;

        V3d newGlobalPosition = exponentiate * globalVelocities + point.head<3>();

        V6d result;
        result <<
            // new position calculated from spoint's velocities
            newGlobalPosition(0), newGlobalPosition(1), newGlobalPosition(2),
            // local deltas are what velocity if moved dt
            globalVelocities(0), globalVelocities(1), globalVelocities(2);

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

    M6x3 computeKalmanGain(const M6x3& Pxy, const M3d& Pz) {
        Eigen::FullPivLU<M3d> lu(Pz);
        if (!lu.isInvertible()) {
            // regularize
            M3d regularized = Pz + 1e-6 * M3d::Identity();
            return Pxy * regularized.inverse();
        }
        return Pxy * lu.inverse();
    }
};