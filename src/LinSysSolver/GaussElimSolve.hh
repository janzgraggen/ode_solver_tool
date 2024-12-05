//
// Created by janzgraggen on 27/11/2024.
//
#ifndef __GAUSSELIMSOLVE_HH__
#define __GAUSSELIMSOLVE_HH__

#include "LinSysSolver.hh"

/**
 * @class GaussElimSolve
 * @brief Implements a linear system solver using Gaussian elimination.
 *
 * This class extends the LinSysSolver base class and overrides the Solve method
 * to solve the system \( Ax = b \) using Eigen's colPivHouseholderQr decomposition.
 */
class GaussElimSolve : public LinSysSolver {
public:
    /**
     * @brief Default destructor for GaussElimSolve.
     *
     * Ensures proper cleanup of resources, though no specific resources are allocated
     * by this class directly.
     */
    ~GaussElimSolve() override = default;

    /**
     * @brief Solves the linear system \( Ax = b \) using Gaussian elimination.
     *
     * This method overrides the Solve method in the LinSysSolver base class.
     *
     * @return Eigen::VectorXd The solution vector \( x \).
     */
    Eigen::VectorXd Solve() override;
};

#endif // __GAUSSELIMSOLVE_HH__
