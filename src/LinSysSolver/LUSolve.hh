/**
* @file LUSolve.hh
 * @brief Header file for the `LUSolve` class.
 *
 * This header declares the `LUSolve` class, which extends the `LinSysSolver` base class.
 * It provides a solver for the system of linear equations \( Ax = b \) using LU decomposition
 * with the Eigen library's `FullPivLU` method. The class handles both invertible and singular matrices.
 *
 * Author: janzgraggen
 * Date: 27/11/2024
 */

#ifndef __LU_SOLVE_HH__
#define __LU_SOLVE_HH__

#include "LinSysSolver.hh"

/**
 * @class LUSolve
 * @brief A class to solve linear systems \( Ax = b \) using LU decomposition.
 *
 * Inherits from the `LinSysSolver` base class and overrides the `Solve()` method
 * to solve the system \( Ax = b \) using LU decomposition provided by Eigen's `FullPivLU`.
 */
class LUSolve : public LinSysSolver {
public:
 /**
  * @brief Default constructor for the LUSolve class.
  * @param logger_ Reference to a Logger object for logging purposes.
  */
 LUSolve(Logger& logger_);

 /**
  * @brief Destructor for the LUSolve class.
  *
  * Ensures proper cleanup of resources in derived classes.
  */
 ~LUSolve() override;

 /**
  * @brief Solves the linear system \( Ax = b \) using LU decomposition.
  *
  * Overrides the `Solve` method in the `LinSysSolver` base class.
  * - Retrieves the coefficient matrix \( A \) and the right-hand side vector \( b \).
  * - Performs LU decomposition using Eigen's `FullPivLU` method.
  * - Throws an error if the matrix \( A \) is singular.
  *
  * @return Eigen::VectorXd The solution vector \( x \).
  * @throws std::runtime_error If the matrix \( A \) is singular.
  */
 Eigen::VectorXd solveSys() override;
};

#endif // __LU_SOLVE_HH__