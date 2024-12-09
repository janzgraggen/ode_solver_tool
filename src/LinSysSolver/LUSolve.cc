/**
 * @file LUSolve.cc
 * @brief Defines the implementation of the `LUSolve` class.
 *
 * This source file contains the method definitions for the `LUSolve` class,
 * which is responsible for solving a system of linear equations \( Ax = b \) using
 * LU decomposition with the Eigen library's `FullPivLU` decomposition method.
 *
 * It retrieves the coefficient matrix \( A \) and the right-hand side vector \( b \)
 * and performs the decomposition. If \( A \) is not invertible, it logs an error.
 *
 * Author: janzgraggen  
 * Date: 27/11/2024
 */

#include "LUSolve.hh"

// Constructor
/**
 * @brief Default constructor for the LUSolve class.
 * @param logger_ Reference to a Logger object for logging purposes.
 */
LUSolve::LUSolve(Logger& logger_) : LinSysSolver(logger_) {}

// Destructor
/**
 * @brief Default destructor for the LUSolve class.
 */
LUSolve::~LUSolve() = default;

// Solve method
/**
 * @brief Solves the system of linear equations \( Ax = b \) using LU decomposition.
 *
 * This method:
 * - Retrieves the coefficient matrix \( A \) and the right-hand side vector \( b \).
 * - Performs an LU decomposition using Eigen's `FullPivLU` decomposition method.
 * - Solves for the system \( Ax = b \) if \( A \) is invertible.
 * - Logs an error and returns an empty vector if \( A \) is singular.
 *
 * @return Eigen::VectorXd The solution vector \( x \).
 * @throws std::runtime_error If the coefficient matrix \( A \) is singular.
 */
Eigen::VectorXd LUSolve::Solve() {
    // Retrieve the coefficient matrix (A) and the right-hand side vector (b)
    Eigen::MatrixXd A = this->GetA();
    Eigen::VectorXd b = this->GetB();

    // Perform LU decomposition using Eigen's FullPivLU
    Eigen::FullPivLU<Eigen::MatrixXd> lu(A);

    // Check if the matrix is invertible
    if (lu.isInvertible()) {
        // Solve the system Ax = b
        return lu.solve(b);
    } else {
        // Log an error if the matrix is singular
        logger.error("Matrix is singular, cannot solve.");
        return Eigen::VectorXd();  // Return an empty vector
    }
}