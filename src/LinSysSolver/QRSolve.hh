/**
* @file QRSolve.hh
 * @brief Header file for the `QRSolve` class.
 *
 * This header file declares the `QRSolve` class, which extends the
 * `LinSysSolver` base class. It provides a solver for the system of linear
 * equations \( Ax = b \) using the Eigen library's `colPivHouseholderQr()` decomposition.
 *
 * Author: janzgraggen
 * Date: 27/11/2024
 */

#ifndef QRSOLVE_HH
#define QRSOLVE_HH

#include "LinSysSolver.hh"
#include <Eigen/Dense>

/**
 * @class QRSolve
 * @brief A class to solve a linear system \( Ax = b \) using QR decomposition.
 *
 * Inherits from `LinSysSolver` and overrides the `Solve()` method to solve
 * a system of linear equations using Eigen's `colPivHouseholderQr()` decomposition.
 */
class QRSolve : public LinSysSolver {
public:
 /**
  * @brief Constructs a `QRSolve` object.
  *
  * Initializes the `QRSolve` instance with a reference to the `Logger`.
  *
  * @param logger_ Reference to a Logger object for logging purposes.
  */
 QRSolve(Logger& logger_);

 /**
  * @brief Destructor for `QRSolve`.
  *
  * Ensures proper cleanup of resources. No specific resources are allocated
  * by this class directly.
  */
 ~QRSolve() override = default;

 /**
  * @brief Solves the system of linear equations \( Ax = b \) using QR decomposition.
  *
  * Overrides the `Solve()` method from the base class `LinSysSolver`.
  *
  * @return Eigen::VectorXd The computed solution vector \( x \).
  */
 Eigen::VectorXd solveSys() override;
};

#endif // QRSOLVE_HH