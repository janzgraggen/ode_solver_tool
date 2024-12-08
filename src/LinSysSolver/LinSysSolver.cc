//
// Created by janzgraggen on 27/11/2024.
//
#include "LinSysSolver.hh"

/**
 * @class LinSysSolver
 * @brief Abstract base class for solving linear systems of equations \( Ax = b \).
 *
 * This class provides a generic interface for setting and getting the coefficient matrix \( A \)
 * and the right-hand side vector \( b \). Specific solving strategies should be implemented
 * in derived classes.
 */

// Constructor
/**
 * @brief Default constructor for the LinSysSolver class.
 */
LinSysSolver::LinSysSolver(Logger& logger_) : logger(logger_) {}

// Destructor
/**
 * @brief Virtual destructor for the LinSysSolver class.
 *
 * Allows proper cleanup of derived classes through polymorphism.
 */
LinSysSolver::~LinSysSolver() {}

// Setters
/**
 * @brief Sets the coefficient matrix \( A \).
 * @param A The coefficient matrix to be set.
 */
void LinSysSolver::SetA(const Eigen::MatrixXd& A) {
    this->A = A;
}

/**
 * @brief Sets the right-hand side vector \( b \).
 * @param b The vector to be set as the right-hand side of the equation.
 */
void LinSysSolver::SetB(const Eigen::VectorXd& b) {
    this->b = b;
}

// Getters
/**
 * @brief Retrieves the coefficient matrix \( A \).
 * @return A constant reference to the coefficient matrix.
 */
const Eigen::MatrixXd& LinSysSolver::GetA() const {
    return A;
}

/**
 * @brief Retrieves the right-hand side vector \( b \).
 * @return A constant reference to the right-hand side vector.
 */
const Eigen::VectorXd& LinSysSolver::GetB() const {
    return b;
}
