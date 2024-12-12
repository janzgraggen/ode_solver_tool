/**
* @file ForwardEuler.cc
 * @brief Implementation of the Forward Euler method for solving ODEs.
 *
 * This file contains the implementation of the `FwdEuler` class, which provides
 * the Forward Euler method, an explicit time-stepping scheme for solving ordinary
 * differential equations (ODEs). The method computes the next step directly based
 * on the current solution and the derivative.
 *
 * Author: natha
 * Date: 25/11/2024
 */

#include "ForwardEuler.hh"

/**
 * @brief Constructs the Forward Euler solver.
 *
 * Initializes the solver with a logger for debugging and recording messages.
 *
 * @param logger_ A `Logger` object used for logging.
 */
FwdEuler::FwdEuler(Logger& logger_) : Explicit(logger_) {}

/**
 * @brief Destructor for the Forward Euler solver.
 */
FwdEuler::~FwdEuler() {}

/**
 * @brief Configures the solver based on settings provided in the configuration reader.
 *
 * This method reads and sets global configurations for the solver.
 *
 * @param Rdr A `Reader` object containing configuration settings.
 */
void FwdEuler::setConfig(const Reader& Rdr) {
    setGlobalConfig(Rdr);  // Call the base class method
}

/**
 * @brief Advances the solution by one time step using the Forward Euler method.
 *
 * This method implements the Forward Euler update:
 * \[
 * y_{n+1} = y_n + h \cdot f(y_n, t_n)
 * \]
 * where:
 * - \( y_n \) is the solution vector at the current step.
 * - \( h \) is the time step size.
 * - \( f(y_n, t_n) \) is the derivative computed by the right-hand-side function.
 *
 * @param y The solution vector at the current time step.
 * @param t The current time.
 * @return The solution vector at the next time step.
 */
Eigen::VectorXd FwdEuler::calcStep(const Eigen::VectorXd& y, double t) {
    Eigen::VectorXd f = getRightHandSide()(y, t);  // Call the vectorized RHS function
    return y + getStepSize() * f;  // Forward Euler step update
}