/**
 * @file BackwardEuler.hh
 * @brief Header file for the BackwardEuler class.
 *
 * This header defines the `BackwardEuler` class, which implements the Backward Euler method
 * for solving systems of ordinary differential equations (ODEs). It is suitable for stiff
 * differential equations and is built on top of the `Implicit` class.
 *
 * Author: janzgraggen
 * Date: 27/11/2024
 */

#ifndef __BACKWARDEULER_HH__
#define __BACKWARDEULER_HH__

#include "Implicit.hh"
#include <functional>
#include <Eigen/Dense>

/**
 * @typedef F_TYPE
 * @brief Alias for a callable object that computes a vector transformation.
 *
 * Represents a function \( F(y) \) where:
 * - `Eigen::VectorXd` is the input vector \( y \).
 * - Returns an `Eigen::VectorXd` as the output result.
 */
using F_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;

/**
 * @class BackwardEuler
 * @brief Implements the Backward Euler method for solving ODEs.
 *
 * The Backward Euler method is an implicit time-stepping scheme,
 * suitable for stiff differential equations. It extends the `Implicit` class
 * and provides flexibility in configuring solvers and right-hand side systems.
 */
class BackwardEuler : public Implicit {
public:

    /**
     * @brief Constructor for the BackwardEuler class.
     * @param logger_ Logger instance to log relevant messages and debug information.
     */
    explicit BackwardEuler(Logger& logger_);

    /**
     * @brief Computes the function \( F(y_1, y_0, t_0) \) for the Backward Euler method.
     *
     * This method represents the implicit relationship for advancing the solution by one time step.
     *
     * @param y1 The solution vector at the next time step.
     * @param y0 The solution vector at the current time step.
     * @param t0 The current time.
     * @return The evaluated function \( F(y_1, y_0, t_0) \).
     */
    virtual Eigen::VectorXd F(Eigen::VectorXd y1, Eigen::VectorXd y0, double t0);

    /**
     * @brief Generates a callable object representing \( F(y_1) \) for root-finding.
     *
     * This method encapsulates the Backward Euler relationship for a specific \( y_0 \) and \( t_0 \).
     * The returned callable object can be used with root-finding solvers like Newton-Raphson.
     *
     * @param y0 The solution vector at the current time step.
     * @param t0 The current time.
     * @return A callable object representing \( F(y_1) \) for root-finding.
     */
    virtual F_TYPE makeFstep(Eigen::VectorXd y0, double t0) override;

    /**
     * @brief Configures the Backward Euler solver based on settings from a configuration reader.
     *
     * Reads configuration settings such as:
     * - Whether the right-hand side is linear.
     * - The solver to use for linear systems or root-finding.
     * - Additional parameters like time step size, tolerances, etc.
     *
     * @param Rdr A `Reader` object containing configuration settings.
     */
    void setConfig(const Reader& Rdr) override;

    /**
     * @brief Advances the solution by one time step using the Backward Euler method for linear systems.
     *
     * For linear systems, it solves the implicit equation:
     * \[
     (I - hA)y_1 = hy_0 + b
     \]
     * where:
     * - \( A \) is the matrix representing the linear system's coefficients.
     * - \( b \) is the vector from the system's right-hand side.
     *
     * This method uses the configured solver (Gaussian elimination or LU decomposition).
     *
     * @param y The solution vector at the current time step.
     * @param t The current time.
     * @return The solution vector at the next time step.
     * @throws std::runtime_error If the configured linear solver is not recognized.
     */
    Eigen::VectorXd calcLinStep(const Eigen::VectorXd y, double t) override;
};

#endif // __BACKWARDEULER_HH__