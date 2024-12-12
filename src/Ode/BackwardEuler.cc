/**
 * @file BackwardEuler.hh
 * @brief Header file for the BackwardEuler class.
 *
 * This header defines the `BackwardEuler` class, which implements the Backward Euler method
 * for solving systems of ordinary differential equations (ODEs). It supports linear and implicit solvers.
 * The method is built on top of the `Implicit` class and offers flexibility with configuration reading.
 *
 * Author: janzgraggen
 * Date: 27/11/2024
 */

#include "BackwardEuler.hh"
#include "../LinSysSolver/GaussElimSolve.hh"
#include "../LinSysSolver/LUSolve.hh"


/**
 * @typedef F_TYPE
 * @brief Alias for a callable object that computes \( F(y) \), typically for root-finding or solving implicit equations.
 */

/**
 * @brief Constructor for BackwardEuler, initializes with the logger.
 * @param logger_ Logger instance to log messages.
 */
BackwardEuler::BackwardEuler(Logger& logger_) : Implicit(logger_) {}

/**
 * @brief Computes the function \( F(y_1, y_0, t_0) \) for the Backward Euler method.
 *
 * This function represents the implicit equation for the Backward Euler time-stepping scheme:
 * \[
 * F(y_1, y_0, t_0) = y_1 - y_0 - h \cdot f(y_1, t_0)
 * \]
 * where:
 * - \( y_1 \) is the unknown at the next time step.
 * - \( y_0 \) is the known solution at the current time step.
 * - \( h \) is the time step size.
 *
 * @param y1 Solution vector at the next time step.
 * @param y0 Solution vector at the current time step.
 * @param t0 Current time.
 * @return Evaluated \( F(y_1, y_0, t_0) \).
 */
Eigen::VectorXd BackwardEuler::F(Eigen::VectorXd y1, Eigen::VectorXd y0, double t0) {
    return y1 - y0 - getStepSize() * getRightHandSide()(y1, t0);
}

/**
 * @brief Generates a callable object representing \( F(y_1) \) for root-finding.
 *
 * This method creates a function \( F \) where \( y_0 \) and \( t_0 \) are fixed parameters.
 * The returned function can be used with root-finding solvers like Newton-Raphson.
 *
 * @param y0 Solution vector at the current time step.
 * @param t0 Current time.
 * @return A callable object representing \( F(y_1) \).
 */
F_TYPE BackwardEuler::makeFstep(Eigen::VectorXd y0, double t0) {
    return [this, y0, t0](const Eigen::VectorXd& y1) { return this->F(y1, y0, t0); };
}

/**
 * @brief Configures the Backward Euler solver based on settings from a configuration reader.
 *
 * Reads configuration settings such as:
 * - Whether the right-hand side is linear.
 * - The solver to use for linear systems or root-finding.
 * - Additional parameters like the RHS system or global configurations.
 *
 * @param Rdr A `Reader` object containing configuration settings.
 */
void BackwardEuler::setConfig(const Reader& Rdr) {
    ImplicitSettings settings = Rdr.getImplicitSettings();
    setRhsIsLinear(settings.rhs_is_linear);
    setLinearSystemSolver(settings.linear_system_solver.value());

    if (getRhsIsLinear()) {
        setRhsSystem(settings.rhs_system.value());
    } else {
        setRootFinder(settings.root_finder.value());
    } 
    if (! getRhsIsLinear()){
        if (settings.tolerance.has_value() &&
        settings.max_iterations.has_value()) {
            setTolerance(settings.tolerance.value());
            setMaxIterations(settings.max_iterations.value());
            if (getTolerance() > 1e-6) {
                logger->warning("{in BackwardEuler::setConfig()} Big tolerance, result might be not precise: " + std::to_string(getTolerance()));
            }if (getMaxIterations() < 50 ) {
                logger->warning("{in BackwardEuler::setConfig()} Small number of iterations, result might be not precise: " + std::to_string(getMaxIterations()));
            }if (getMaxIterations() > 1e6) {
                logger->warning("{in BackwardEuler::setConfig()} Big number of iterations, possibly long run time: " + std::to_string(getMaxIterations()));
            }
            if (settings.dx.has_value() && getRootFinder() == "NewtonRaphson") {
                setDx(settings.dx.value());
                if (getDx() > 1e-3) {
                    logger->warning("{in BackwardEuler::setConfig()} Big dx, result might be not precise: " + std::to_string(getDx()));
                }
            } else {
                logger->warning("{in BackwardEuler::setConfig()} Missing parameter for root finder. Using default value.");
            }
        } else {
            logger->warning("{in BackwardEuler::setConfig()} Missing parameter for nonlinear solver. Using default values.");
        }
    }
    

    // Call the base class method to set global configurations. (After potential linear system setup -> setRhsSystem based on it)
    setGlobalConfig(Rdr);
}

/**
 * @brief Advances the solution by one time step using the Backward Euler method for linear systems.
 *
 * For linear systems, the implicit equation:
 * \[
 * (I - hA)y_1 = hy_0 + b
 * \]
 * is solved, where:
 * - \( A \) is the matrix representing the system's linear terms.
 * - \( b \) is the vector from the system's right-hand side.
 *
 * This method uses the configured linear solver to compute \( y_1 \).
 *
 * @param y Solution vector at the current time step.
 * @param t Current time.
 * @return Solution vector at the next time step.
 * @throws std::runtime_error If the configured linear solver is invalid.
 */
Eigen::VectorXd BackwardEuler::calcLinStep(const Eigen::VectorXd y, double t) {
    // Construct the matrix \( I - hA \)
    const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(
        getRhsSystem().A.rows(), getRhsSystem().A.cols());
    const Eigen::MatrixXd A = identity - getStepSize() * getRhsSystem().A;

    // Construct the vector \( b + hy \)
    const Eigen::VectorXd b = getStepSize() * getRhsSystem().b + y;

    // Solve the system according to the configured solver
    if (getLinearSystemSolver() == "GaussianElimination") {
        GaussElimSolve solver(*logger);
        solver.setA(A);
        solver.setB(b);
        return solver.solveSys();
    } else if (getLinearSystemSolver() == "LU") {
        LUSolve solver(*logger);
        solver.setA(A);
        solver.setB(b);
        return solver.solveSys();
    } else {
        logger->error("{in BackwardEuler::calcLinStep()} Invalid linear system solver, getLinearSystemSolver() returns: " + getLinearSystemSolver());
        return Eigen::VectorXd();
    }
}