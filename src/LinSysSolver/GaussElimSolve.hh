/**
* @file GaussElimSolve.hh
 * @brief Header file for the `GaussElimSolve` class.
 *
 * This header file declares the `GaussElimSolve` class, which extends the
 * `LinSysSolver` base class. It provides a solver for the system of linear
 * equations \( Ax = b \) using the Gaussian elimination method with the Eigen
 * library's `colPivHouseholderQr()` decomposition.
 *
 * Author: janzgraggen
 * Date: 27/11/2024
 */

#ifndef GAUSSELIMSOLVE_HH
#define GAUSSELIMSOLVE_HH

#include "LinSysSolver.hh"
#include <Eigen/Dense>

/**
 * @class GaussElimSolve
 * @brief A class to solve a linear system \( Ax = b \) using Gaussian elimination.
 *
 * Inherits from `LinSysSolver` and overrides the `Solve()` method to solve
 * a system of linear equations using the Gaussian elimination approach with
 * Eigen's `colPivHouseholderQr()` decomposition.
 */
class GaussElimSolve : public LinSysSolver {
public:
 /**
  * @brief Constructs a `GaussElimSolve` object.
  *
  * Initializes the `GaussElimSolve` instance with a reference to the `Logger`.
  *
  * @param logger_ Reference to a Logger object for logging purposes.
  */
 GaussElimSolve(Logger& logger_);

 /**
  * @brief Destructor for `GaussElimSolve`.
  *
  * Ensures proper cleanup of resources. No specific resources are allocated
  * by this class directly.
  */
 ~GaussElimSolve() override = default;

 /**
  * @brief Solves the system of linear equations \( Ax = b \) using Gaussian elimination.
  *
  * Overrides the `Solve()` method from the base class `LinSysSolver`.
  *
  * @return Eigen::VectorXd The computed solution vector \( x \).
  */
 Eigen::VectorXd solveSys() override;
};

#endif // GAUSSELIMSOLVE_HH