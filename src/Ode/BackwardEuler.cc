//
// Created by [Your Name] on [Date].
//

#include "BackwardEuler.hh"
#include "../LinSysSolver/GaussElimSolve.hh"
#include "../LinSysSolver/LUSolve.hh"

/**
 * @typedef F_TYPE
 * @brief Alias for a callable object that computes \( F(y) \), typically for root-finding or solving implicit equations.
 */

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
    return y1 - y0 - GetStepSize() * GetRightHandSide()(y1, t0);
}

/**
 * @brief Generates a callable object representing \( F(y_1) \) for root-finding.
 *
 * This method creates a function \( F \) where \( y_0 \) and \( t_0 \) are fixed parameters.
 * The returned function can be used with solvers like Newton-Raphson to find \( y_1 \).
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
 * This method reads settings such as:
 * - Whether the right-hand side is linear.
 * - The solver to use for linear systems or root-finding.
 * - Additional parameters like the RHS system or global configurations.
 *
 * @param Rdr A `Reader` object containing configuration settings.
 */
void BackwardEuler::SetConfig(const Reader& Rdr) {
    SetRhsIsLinear(Rdr.getImplicitSettings().rhs_is_linear);
    SetLinearSystemSolver(Rdr.getImplicitSettings().linear_system_solver.value());

    if (GetRhsIsLinear()) {
        SetRhsSystem(Rdr.getImplicitSettings().rhs_system.value());
    } else {
        SetRootFinder(Rdr.getImplicitSettings().root_finder.value());
    }

    // Call the base class method to set global configurations.
    SetGlobalConfig(Rdr);
}

/**
 * @brief Advances the solution by one time step using the Backward Euler method for linear systems.
 *
 * For linear systems, the implicit equation:
 * \[
 * (I - hA)y_1 = hy_0 + b
 * \]
 * is solved, where:
 * - \( A \) is the matrix from the linear RHS system.
 * - \( b \) is the vector from the linear RHS system.
 *
 * This method uses the configured linear solver to compute \( y_1 \).
 *
 * @param y Solution vector at the current time step.
 * @param t Current time.
 * @return Solution vector at the next time step.
 * @throws std::runtime_error If the configured linear solver is invalid.
 */
Eigen::VectorXd BackwardEuler::LinStep(const Eigen::VectorXd y, double t) {
    // Create \( I - hA \)
    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(GetRhsSystem().A.rows(), GetRhsSystem().A.cols());
    Eigen::MatrixXd A = identity - GetStepSize() * GetRhsSystem().A;

    // Create \( b + hy \)
    Eigen::VectorXd b = GetStepSize() * GetRhsSystem().b + y;

    // Solve the linear system based on the chosen solver
    if (GetLinearSystemSolver() == "GaussianElimination") {
        GaussElimSolve solver;
        solver.SetA(A);
        solver.SetB(b);
        return solver.Solve();
    } else if (GetLinearSystemSolver() == "LU") {
        LUSolve solver;
        solver.SetA(A);
        solver.SetB(b);
        return solver.Solve();
    } else {
        throw std::runtime_error("Invalid linear system solver: " + GetLinearSystemSolver());
    }
}