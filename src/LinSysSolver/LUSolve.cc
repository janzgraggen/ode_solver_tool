#include "LUSolve.hh"
#include <stdexcept> // For runtime_error

/**
 * @class LUSolve
 * @brief Implements a linear system solver using LU decomposition.
 *
 * This class extends the `LinSysSolver` base class and overrides the `Solve`
 * method to solve the linear system \( Ax = b \) using Eigen's LU decomposition.
 */

// Constructor
/**
 * @brief Default constructor for the LUSolve class.
 */
LUSolve::LUSolve(Logger& logger_) : LinSysSolver(logger_) {};

// Destructor
/**
 * @brief Default destructor for the LUSolve class.
 */
LUSolve::~LUSolve() = default;

// Solve method
/**
 * @brief Solves the linear system \( Ax = b \) using LU decomposition.
 *
 * This method retrieves the coefficient matrix \( A \) and the right-hand side vector \( b \),
 * performs an LU decomposition using Eigen's `FullPivLU` class, and solves the system.
 * If the matrix \( A \) is not invertible, it throws a runtime error.
 *
 * @return Eigen::VectorXd The solution vector \( x \).
 * @throws std::runtime_error if the coefficient matrix \( A \) is singular.
 */
Eigen::VectorXd LUSolve::Solve() {
    // Retrieve the coefficient matrix (A) and the right-hand side vector (b)
    Eigen::MatrixXd A = this->GetA();
    Eigen::VectorXd b = this->GetB();

    // Perform LU decomposition
    Eigen::FullPivLU<Eigen::MatrixXd> lu(A);

    // Check if the matrix is invertible
    if (lu.isInvertible()) {
        // Solve the system Ax = b
        return lu.solve(b);
    } else {
        // Throw an error if the matrix is singular
        logger.error("Matrix is singular, cannot solve.");
        return Eigen::VectorXd(); // Return an empty vector
    }
}
