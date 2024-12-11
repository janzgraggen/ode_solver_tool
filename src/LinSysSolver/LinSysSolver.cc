/**
 * @file LinSysSolver.cc
 * @brief Defines the implementation for the `LinSysSolver` class.
 *
 * This source file contains the method definitions for the abstract base class
 * `LinSysSolver`. It provides functionality for setting and retrieving the
 * coefficient matrix \( A \) and the right-hand side vector \( b \). Specific
 * solving algorithms are meant to be implemented in derived classes.
 *
 * Author: janzgraggen
 * Date: 27/11/2024
 */

#include "LinSysSolver.hh"

/**
 * @brief Default constructor for the LinSysSolver class.
 *
 * Initializes the `LinSysSolver` instance with a reference to the `Logger` object
 * for logging purposes.
 *
 * @param logger_ Reference to a Logger object.
 */
LinSysSolver::LinSysSolver(Logger& logger_) : logger(&logger_) {}

/**
 * @brief Virtual destructor for the LinSysSolver class.
 *
 * Ensures proper cleanup of derived classes through polymorphism.
 */
LinSysSolver::~LinSysSolver() {}

/**
 * @brief Sets the coefficient matrix \( A \).
 *
 * Stores the provided coefficient matrix \( A \) in the class member.
 *
 * @param A The coefficient matrix to be set.
 */
void LinSysSolver::SetA(const Eigen::MatrixXd& A) {
    this->A = A;
}

/**
 * @brief Sets the right-hand side vector \( b \).
 *
 * Stores the provided vector \( b \) as the right-hand side of the system.
 *
 * @param b The vector representing the right-hand side of the system.
 */
void LinSysSolver::SetB(const Eigen::VectorXd& b) {
    this->b = b;
}

/**
 * @brief Retrieves the coefficient matrix \( A \).
 *
 * Provides a constant reference to the coefficient matrix for access without copying.
 *
 * @return A constant reference to the coefficient matrix \( A \).
 */
const Eigen::MatrixXd& LinSysSolver::GetA() const {
    return A;
}

/**
 * @brief Retrieves the right-hand side vector \( b \).
 *
 * Provides a constant reference to the right-hand side vector \( b \) for access
 * without copying.
 *
 * @return A constant reference to the right-hand side vector \( b \).
 */
const Eigen::VectorXd& LinSysSolver::GetB() const {
    return b;
}
