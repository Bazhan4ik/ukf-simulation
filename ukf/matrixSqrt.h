#include "../Eigen/Core"
#include "../Eigen/Eigenvalues"

using M9d = Eigen::Matrix<double, 9, 9>;

M9d matrixSqrt(const M9d& covariance) {

    // First try Cholesky - fastest for positive definite matrices
    Eigen::LLT<M9d> cholesky(covariance);
    
    if (cholesky.info() == Eigen::Success) {
        return cholesky.matrixL();  // Returns lower triangular L where P = L*L^T
    }
    
    // Cholesky failed - matrix is not positive definite
    // Fall back to eigenvalue decomposition with regularization
    Eigen::SelfAdjointEigenSolver<M9d> eigenSolver(covariance);
    
    if (eigenSolver.info() != Eigen::Success) {
        // Even eigenvalue decomposition failed - add regularization
        const float regularization = 1e-6f;
        M9d regularized = covariance + regularization * M9d::Identity();
        return matrixSqrt(regularized);  // Recursive call with regularized matrix
    }
    
    // Clamp negative eigenvalues to small positive values
    auto eigenvals = eigenSolver.eigenvalues();
    const float min_eigenval = 1e-8f;
    eigenvals = eigenvals.cwiseMax(min_eigenval);
    
    // Reconstruct: sqrt(P) = V * sqrt(D) * V^T, but we want L such that L*L^T = P
    M9d V = eigenSolver.eigenvectors();
    M9d sqrtD = eigenvals.cwiseSqrt().asDiagonal();
    
    return V * sqrtD;  // This gives us the symmetric square root
}