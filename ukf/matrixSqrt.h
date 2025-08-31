#include "../Eigen/Eigenvalues"
#include "types.h"

M6d matrixSqrt(const M6d& covariance) {

    // First try Cholesky - fastest for positive definite matrices
    Eigen::LLT<M6d> cholesky(covariance);
    
    if (cholesky.info() == Eigen::Success) {
        return cholesky.matrixL();  // Returns lower triangular L where P = L*L^T
    }
    
    // Cholesky failed - matrix is not positive definite
    // Fall back to eigenvalue decomposition with regularization
    Eigen::SelfAdjointEigenSolver<M6d> eigenSolver(covariance);
    
    if (eigenSolver.info() != Eigen::Success) {
        // Even eigenvalue decomposition failed - add regularization
        const double regularization = 1e-6;
        M6d regularized = covariance + regularization * M6d::Identity();
        return matrixSqrt(regularized);  // Recursive call with regularized matrix
    }
    
    // Clamp negative eigenvalues to small positive values
    auto eigenvals = eigenSolver.eigenvalues();
    const double min_eigenval = 1e-8;
    eigenvals = eigenvals.cwiseMax(min_eigenval);
    
    // Reconstruct: sqrt(P) = V * sqrt(D) * V^T, but we want L such that L*L^T = P
    M6d V = eigenSolver.eigenvectors();
    M6d sqrtD = eigenvals.cwiseSqrt().asDiagonal();
    
    return V * sqrtD;  // This gives us the symmetric square root
}