/**
 * @file Implicit.cc
 * @brief Implementation of the `Implicit` base class for implicit ODE solvers.
 *
 * This file provides the implementation of the `Implicit` class, which serves as a base
 * for implicit numerical methods for solving ordinary differential equations (ODEs).
 * It includes functionality for handling linear and nonlinear systems, root finding,
 * and configuration management.
 *
 * Author: janzgraggen
 * Date: 28/11/2024
 */

#include "Implicit.hh"
#include "../RootFinder/RootFinder.hh"
#include "../RootFinder/NewtonRaphson.hh"

// Type alias definitions for function types and strings
using f_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)>;
using str = std::string;

// Constructor
Implicit::Implicit(Logger& logger_) : OdeSolver(logger_), tol(0.0), max_iter(0), dx(1e-6) {}

/**
 * @brief Checks if the right-hand side function is linear.
 * @return `true` if the right-hand side is linear, `false` otherwise.
 */
bool Implicit::GetRhsIsLinear() const {
    return rhs_is_linear;
}

/**
 * @brief Sets whether the right-hand side function is linear.
 * @param is_linear A boolean indicating if the function is linear.
 */
void Implicit::SetRhsIsLinear(bool is_linear) {
    rhs_is_linear = is_linear;
}

/**
 * @brief Retrieves the name of the linear system solver.
 * @return The solver's name as a string.
 */
str Implicit::GetLinearSystemSolver() const {
    return linear_system_solver;
}

/**
 * @brief Retrieves the name of the root-finder method.
 * @return The root-finder's name as a string.
 */
str Implicit::GetRootFinder() const {
    return root_finder;
}

/**
 * @brief Retrieves the linear system representation for implicit methods.
 * @return The `LinearSystem` object representing the right-hand side.
 */
LinearSystem Implicit::GetRhsSystem() const {
    return rhs_system;
}


/**
 * @brief Retrieves the tolerance for root-finding algorithms.
 * @return The tolerance value as a double.
 */
double Implicit::GetTolerance() const {
    return tol;   
}

/**
 * @brief Retrieves the maximum number of iterations for root-finding.
 * @return The maximum number of iterations as an integer.
 */

int Implicit::GetMaxIterations() const {
    return max_iter;

}

/**
 * @brief Retrieves the step size for numerical differentiation.
 * @return The step size value as a double.
 */
double Implicit::GetDx() const {
    return dx;
}

/**
 * @brief Sets the step size for numerical differentiation.
 * @param dx The step size value to set.
 */
void Implicit::SetDx(double dx) {
    this->dx = dx;
}

/**
 * @brief Sets the number of iterations for root-finding.
 * @param max_iter The maximum number of iterations to set.
 */
void Implicit::SetMaxIterations(int max_iter) {
    this->max_iter = max_iter;
}

/**
 * @brief Sets the tolerance for root-finding algorithms.
 * @param tol The tolerance value to set.
 */

void Implicit::SetTolerance(double tol) {
    this->tol = tol;
}

/**
 * @brief Sets the root-finder method for nonlinear systems.
 * @param root_finder_in The name of the root-finder to use.
 */
void Implicit::SetRootFinder(str root_finder_in) {
    root_finder = root_finder_in;
}

/**
 * @brief Sets the linear system representation for the right-hand side.
 * @param system The `LinearSystem` object representing the system.
 */
void Implicit::SetRhsSystem(LinearSystem system) {
    rhs_system = system;
}

/**
 * @brief Sets the linear system solver to use.
 * @param solver The name of the solver as a string.
 */
void Implicit::SetLinearSystemSolver(str solver) {
    linear_system_solver = solver;
}

/**
 * @brief Sets the right-hand side function for the ODE system.
 *
 * If the system is linear, the function is configured using the linear system representation.
 * Otherwise, it uses the provided function.
 *
 * @param f The right-hand side function \( f(y, t) \).
 */
void Implicit::SetRightHandSide(const f_TYPE& f) {
    if (GetRhsIsLinear()) {
        f_rhs = [this](const Eigen::VectorXd& y, double t) {
            return this->GetRhsSystem().A * y + this->GetRhsSystem().b;
        };
    } else {
        this->f_rhs = f;
    }
}

/**
 * @brief Advances the solution for nonlinear systems using the root finder.
 *
 * This method uses the root-finding algorithm specified in the configuration
 * to solve the nonlinear implicit relationship.
 *
 * @param y Current solution vector.
 * @param t Current time.
 * @return The solution vector at the next time step.
 * @throws std::runtime_error if an invalid root finder method is specified.
 */
Eigen::VectorXd Implicit::NonLinStep(const Eigen::VectorXd y, double t) {
    RootFinder* solver = nullptr;
    if (GetRootFinder() == "NewtonRaphson") {
        solver = new NewtonRaphson(*logger, makeFstep(y, t));
        solver->setInitialGuess(y);
        solver->SetLinearSystemSolver(GetLinearSystemSolver());
        
        if (GetTolerance() && GetMaxIterations()) { //< they are initialized to 0
            solver->setTolerance(GetTolerance());
            solver->setMaxIterations(GetMaxIterations());
            if (GetDx() && GetRootFinder() == "NewtonRaphson") {
                solver->setDx(GetDx());
            } else {
                logger->warning("{in Implicit::NonLinStep()} Missing parameter for nonlinear solver: dx. Using default values.");
            }

        } else {
            logger->warning("{in Implicit::NonLinStep()} Missing parameter for nonlinear solver. Using default values.");
        }
        
        Eigen::VectorXd solution = solver->Solve();

        // Clean up resources
        delete solver;
        return solution;
    } else {
        logger->error("Invalid root finder method: " + GetRootFinder());
        return Eigen::VectorXd(); // Return an empty vector
    }
}

/**
 * @brief Advances the solution by one time step.
 *
 * This method determines whether to use linear or nonlinear methods based on
 * the system configuration and calls the appropriate step function.
 *
 * @param y Current solution vector.
 * @param t Current time.
 * @return The solution vector at the next time step.
 */
Eigen::VectorXd Implicit::Step(const Eigen::VectorXd& y, double t) {
    if (GetRhsIsLinear()) {
        return LinStep(y, t);
    } else {
        return NonLinStep(y, t);
    }
}
