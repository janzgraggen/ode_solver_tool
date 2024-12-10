/**
 * @file ForwardEuler.hh
 * @brief Header file for the Forward Euler class.
 *
 * This file declares the `FwdEuler` class, which implements the Forward Euler method,
 * an explicit time-stepping scheme for solving ordinary differential equations (ODEs).
 * The method computes the next solution step based on the current state and derivative.
 *
 * Author: natha
 * Date: 25/11/2024
 */

#ifndef FORWARD_EULER_HH
#define FORWARD_EULER_HH

#include "Explicit.hh"

/**
 * @class FwdEuler
 * @brief Implements the Forward Euler method for solving ODEs.
 *
 * The Forward Euler method is a simple, explicit numerical integration technique.
 * It is best suited for non-stiff problems and provides a straightforward step update:
 * \[
 * y_{n+1} = y_n + h \cdot f(y_n, t_n)
 * \]
 */
class FwdEuler : public Explicit {
public:
    /**
     * @brief Constructs the Forward Euler solver.
     *
     * Initializes the solver with a logger for diagnostics and debugging.
     *
     * @param logger_ A `Logger` object used for logging.
     */
    FwdEuler(Logger& logger_);

    /**
     * @brief Destructor for the Forward Euler solver.
     *
     * Ensures proper cleanup of resources.
     */
    ~FwdEuler();

    /**
     * @brief Configures the solver based on settings provided in a configuration reader.
     *
     * Reads settings like time step size, tolerance, and other solver-specific parameters.
     *
     * @param Rdr A `Reader` object containing configuration settings.
     */
    void SetConfig(const Reader& Rdr) override;

    /**
     * @brief Advances the solution by one time step using the Forward Euler method.
     *
     * Computes the next solution step based on the current solution and the derivative.
     *
     * @param y The solution vector at the current time step.
     * @param t The current time.
     * @return The solution vector at the next time step.
     */
    Eigen::VectorXd Step(const Eigen::VectorXd& y, double t) override;
};

#endif // FORWARD_EULER_HH