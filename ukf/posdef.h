#include "types.h"

M9d ensurePositiveDefinite(const M9d& cov) {
    Eigen::SelfAdjointEigenSolver<M9d> eigenSolver(cov);
    if (eigenSolver.info() != Eigen::Success) { // not positive definite
        // regularize
        return cov + 1e-6 * M9d::Identity();
    }
    
    auto eigenvals = eigenSolver.eigenvalues();
    const double minEigenval = 1e-8;
    
    if (eigenvals.minCoeff() < minEigenval) {
        eigenvals = eigenvals.cwiseMax(minEigenval);
        auto V = eigenSolver.eigenvectors();
        return V * eigenvals.asDiagonal() * V.transpose();
    }
    
    return cov;
}
M6d ensurePositiveDefinite(const M6d& cov) {
    Eigen::SelfAdjointEigenSolver<M6d> eigenSolver(cov);
    if (eigenSolver.info() != Eigen::Success) { // not positive definite
        // regularize
        return cov + 1e-6 * M6d::Identity();
    }
    
    auto eigenvals = eigenSolver.eigenvalues();
    const double minEigenval = 1e-8;
    
    if (eigenvals.minCoeff() < minEigenval) {
        eigenvals = eigenvals.cwiseMax(minEigenval);
        auto V = eigenSolver.eigenvectors();
        return V * eigenvals.asDiagonal() * V.transpose();
    }
    
    return cov;
}

M3d ensurePositiveDefinite(const M3d& cov) {
    Eigen::SelfAdjointEigenSolver<M3d> eigenSolver(cov);
    if (eigenSolver.info() != Eigen::Success) { // not positive definite
        // regularize
        return cov + 1e-6 * M3d::Identity();
    }
    
    auto eigenvals = eigenSolver.eigenvalues();
    const double minEigenval = 1e-8;
    
    if (eigenvals.minCoeff() < minEigenval) {
        eigenvals = eigenvals.cwiseMax(minEigenval);
        auto V = eigenSolver.eigenvectors();
        return V * eigenvals.asDiagonal() * V.transpose();
    }
    
    return cov;
}

// Matrix3 ensurePositiveDefinite3(const Matrix3& cov) {
//     Eigen::SelfAdjointEigenSolver<Matrix3> eigenSolver(cov);
//     if (eigenSolver.info() != Eigen::Success) {
//         // Fallback: add regularization
//         std::cout << "Matrix not positive definite" << std::endl;
//         return cov + 1e-6f * Matrix3::Identity();
//     }
    
//     auto eigenvals = eigenSolver.eigenvalues();
//     const float min_eigenval = 1e-8f;
    
//     if (eigenvals.minCoeff() < min_eigenval) {
//         // Clamp negative eigenvalues
//         eigenvals = eigenvals.cwiseMax(min_eigenval);
//         auto V = eigenSolver.eigenvectors();
//         return V * eigenvals.asDiagonal() * V.transpose();
//     }
    
//     return cov; // Already positive definite
// }