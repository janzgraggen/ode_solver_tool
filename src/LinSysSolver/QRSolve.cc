/**
* @file QRSolve.cc
 * @brief Defines the implementation for the `QRSolve` class.
 *
 * This source file contains the method definitions for the `QRSolve` class,
 * which extends the `LinSysSolver` base class. It provides functionality to solve
 * a system of linear equations \( Ax = b \) using the Eigen library's `colPivHouseholderQr()` decomposition.
 *
 * The method `Solve()` retrieves the coefficient matrix \( A \) and the right-hand side
 * vector \( b \) through getter functions from the base class `LinSysSolver` and
 * computes the solution vector \( x \) efficiently using Eigen's decomposition methods.
 *
 * Author: janzgraggen
 * Date: 27/11/2024
 */

#include "QRSolve.hh"

/**
 * @brief Constructor for the `QRSolve` class.
 *
 * Initializes the `QRSolve` instance by calling the constructor of the base class
 * `LinSysSolver` and passing the provided Logger reference for logging purposes.
 *
 * @param logger_ Reference to a Logger instance.
 */
QRSolve::QRSolve(Logger& logger_) : LinSysSolver(logger_) {}

/**
 * @brief Solves the system of linear equations \( Ax = b \) using QR decomposition.
 *
 * This method:
 * - Retrieves the coefficient matrix \( A \) and the right-hand side vector \( b \)
 *   from the base class `LinSysSolver`.
 * - Uses the Eigen library's `colPivHouseholderQr()` decomposition to solve the system
 *   \( Ax = b \) and compute the solution vector \( x \).
 *
 * @return Eigen::VectorXd The computed solution vector \( x \).
 */
Eigen::VectorXd QRSolve::solveSys() {
  // Retrieve the coefficient matrix (A) and the right-hand side vector (b)
  Eigen::MatrixXd A = this->getA();
  Eigen::VectorXd b = this->getB();

  // Solve the system using Eigen's QR decomposition
  return A.colPivHouseholderQr().solve(b);
}
