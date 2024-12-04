#include "LUSolve.hh"
#include <stdexcept> // For runtime_error

// Constructor
LUSolve::LUSolve() = default;

// Destructor
LUSolve::~LUSolve() = default;

// Solve method
Eigen::VectorXd LUSolve::Solve() {
    Eigen::MatrixXd A = this->GetA();
    Eigen::VectorXd b = this->GetB();

    // Perform LU decomposition
    Eigen::FullPivLU<Eigen::MatrixXd> lu(A);

    // Check if the matrix is invertible
    if (lu.isInvertible()) {
        return lu.solve(b); // Solve the system Ax = b
    } else {
        throw std::runtime_error("Matrix is singular, cannot solve.");
    }
}
