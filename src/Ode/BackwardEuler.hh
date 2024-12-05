//
// Created by [Your Name] on [Date].
//

#ifndef __BACKWARDEULER_HH__
#define __BACKWARDEULER_HH__

#include "Implicit.hh"

/**
 * @typedef F_TYPE
 * @brief Alias for a function that computes a vector transformation.
 *
 * Represents a function \( F(y) \) where:
 * - `Eigen::VectorXd` is the input vector \( y \).
 * - Returns an `Eigen::VectorXd` as the result.
 */
using F_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;

/**
 * @class BackwardEuler
 * @brief Implements the Backward Euler method for solving ODEs.
 *
 * The Backward Euler method is an implicit time-stepping scheme,
 * suitable for stiff differential equations.
 */
class BackwardEuler : public Implicit {
public:
    /**
     * @brief Computes the function \( F(y_1, y_0, t_0) \) for the Backward Euler method.
     *
     * This function represents the implicit relationship for advancing the solution.
     *
     * @param y1 The solution vector at the next time step.
     * @param y0 The solution vector at the current time step.
     * @param t0 The current time.
     * @return The evaluated function \( F(y_1, y_0, t_0) \).
     */
    virtual Eigen::VectorXd F(Eigen::VectorXd y1, Eigen::VectorXd y0, double t0);

    /**
     * @brief Generates a function \( F \) that can be used for root-finding during the time step.
     *
     * This function encapsulates the Backward Euler relationship for a specific \( y_0 \) and \( t_0 \).
     *
     * @param y0 The solution vector at the current time step.
     * @param t0 The current time.
     * @return A callable object representing \( F(y) \) for root-finding.
     */
    virtual F_TYPE makeFstep(Eigen::VectorXd y0, double t0) override;

    /**
     * @brief Configures the solver with parameters from a configuration file.
     *
     * Reads parameters relevant to the Backward Euler solver, such as time step, tolerances, etc.
     *
     * @param Rdr A `Reader` object for accessing the configuration.
     */
    void SetConfig(const Reader& Rdr) override;

    /**
     * @brief Advances the solution using the Backward Euler method for linear systems.
     *
     * This method is called if the right-hand side of the ODE is linear.
     *
     * @param y The solution vector at the current time step.
     * @param t The current time.
     * @return The solution vector at the next time step.
     */
    Eigen::VectorXd LinStep(const Eigen::VectorXd y, double t) override;
};

#endif // __BACKWARDEULER_HH__
