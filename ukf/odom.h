#pragma once
#include "types.h"

class Odometry {
public:
    V3d current;

    V3d update(V3d z) {
        // negative sign because imu rotates clockwise
        double deltaTheta = z(2);
        
        // Local displacement vector
        V2d localDelta(z(0), z(1));
        
        // Current global orientation
        double globalTheta = current.z();
        
        // Global displacement vector
        V2d globalDelta;
        
        // if the robot moves straight (doesn't rotate)
        if (std::abs(deltaTheta) < 0.0005) {
            // Taylor series approximation for small angles
            // R ≈ I - θ*[0 -1; 1 0] - θ²/2*I + θ³/6*[0 -1; 1 0]
            // Simplified to: R ≈ [1-θ²/6, -θ/2; θ/2, 1-θ²/6]
            
            double cosTerm = 1.0 - std::pow(deltaTheta, 2) / 6.0;
            double sinTerm = -deltaTheta / 2.0;
            
            M2d rotationMatrix;
            rotationMatrix << cosTerm, sinTerm,
                            -sinTerm, cosTerm;
            
            // Global rotation matrix for current pose
            M2d globalRotation;
            globalRotation << std::cos(globalTheta), -std::sin(globalTheta),
                            std::sin(globalTheta),  std::cos(globalTheta);
            
            globalDelta = globalRotation * rotationMatrix * localDelta;
            current(0) += globalDelta.x();
            current(1) += globalDelta.y();
            return current;
        }
        
        // For non-zero rotation, use exact transformation
        double sinDelta = std::sin(deltaTheta);
        double cosDelta = std::cos(deltaTheta);
        
        // Transformation matrix for arc motion
        M2d transformMatrix;
        transformMatrix << sinDelta / deltaTheta, -(1.0 - cosDelta) / deltaTheta,
                        (1.0 - cosDelta) / deltaTheta, sinDelta / deltaTheta;
        
        // Global rotation matrix for current pose
        M2d globalRotation;
        globalRotation << std::cos(globalTheta), -std::sin(globalTheta),
                        std::sin(globalTheta),  std::cos(globalTheta);
        
        globalDelta = globalRotation * transformMatrix * localDelta;
        
        current(0) += globalDelta.x();
        current(1) += globalDelta.y();
        current(2) += deltaTheta;
        return current;
    }
};