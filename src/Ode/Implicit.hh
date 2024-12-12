/**
 * @file Implicit.hh
 * @brief Header file for the `Implicit` base class for ODE solvers.
 *
 * This file defines the `Implicit` class, providing a framework for solving
 * ordinary differential equations (ODEs) using implicit numerical methods.
 * It supports both linear and nonlinear systems and includes abstractions
 * for root finding and linear system solving.
 *
 * Author: janzgraggen
 * Date: 28/11/2024
 */

#ifndef IMPLICIT_HH
#define IMPLICIT_HH

#include "OdeSolver.hh"
#include "../Utils/LinSysStruct.hh"

/**
 * @typedef f_TYPE
 * @brief Type definition for a function representing the ODE right-hand side.
 *
 * Represents a function \( f(y, t) \) where:
 * - `Eigen::VectorXd` is the state vector \( y \).
 * - `double` is the current time \( t \).
 * - Returns an `Eigen::VectorXd` as the result.
 */
using f_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)>;

/**
 * @typedef F_TYPE
 * @brief Type definition for a function representing a root-finding function.
 *
 * Represents a function \( F(y) \) used in nonlinear solvers where:
 * - Input: `Eigen::VectorXd` as the state vector \( y \).
 * - Output: `Eigen::VectorXd` as the computed result.
 */
using F_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;

/**
 * @typedef str
 * @brief Alias for a standard string type.
 */
using str = std::string;

/**
 * @class Implicit
 * @brief Base class for implicit ODE solvers.
 *
 * Provides a framework for implementing implicit ODE solvers, with support for
 * both linear and nonlinear systems. Derived classes must implement specific
 * solver methods tailored to their requirements.
 */
class Implicit : public OdeSolver {
private:
    bool rhs_is_linear; //!< Flag indicating if the right-hand side function is linear.
    str linear_system_solver; //!< Name of the linear system solver to use.

    /**
     * @brief Linear system representation for implicit solvers.
     *
     * Used when the right-hand side of the ODE is linear.
     */
    LinearSystem rhs_system;

    str root_finder; //!< Name of the root-finder to use for nonlinear systems.
    double tol; //!< Tolerance for root-finding algorithms.
    int max_iter; //!< Maximum number of iterations for root-finding.
    double dx; //!< Step size for numerical differentiation.
public:

    Implicit(Logger& logger_);
    /**
     * @brief Default virtual destructor for Implicit class.
     */
    ~Implicit() override = default;

    // Getters

    /**
     * @brief Checks if the right-hand side function is linear.
     * @return `true` if the right-hand side is linear, `false` otherwise.
     */
    bool getRhsIsLinear() const;

    /**
     * @brief Retrieves the name of the linear system solver.
     * @return A string containing the solver's name.
     */
    str getLinearSystemSolver() const;

    /**
     * @brief Retrieves the name of the root finder.
     * @return A string containing the root finder's name.
     */
    str getRootFinder() const;

    /**
     * @brief Gets the current linear system representation.
     * @return The `LinearSystem` object.
     */
    LinearSystem getRhsSystem() const;

    /**
     * @brief Retrieves the tolerance for root-finding algorithms.
     * @return The tolerance value as a double.
     */
    double getTolerance() const;

    /**
     * @brief Retrieves the maximum number of iterations for root-finding.
     * @return The maximum number of iterations as an integer.
     */
    int getMaxIterations() const;

    /**
     * @brief Retrieves the step size for numerical differentiation.
     * @return The step size value as a double.
     */
    double getDx() const;

    // Setters

    /**
     * @brief Sets the step size for numerical differentiation.
     * @param dx The step size value to set.
     */
    void setDx(double dx);

    /**
     * @brief Sets the number of iterations for root-finding.
     * @param max_iter The maximum number of iterations to set.
     */
    void setMaxIterations(int max_iter);

    /**
     * @brief Sets the tolerance for root-finding algorithms.
     * @param tol The tolerance value to set.
     */
    void setTolerance(double tol); 

    /**
     * @brief Sets the linearity flag of the right-hand side function.
     * @param linear `true` if the right-hand side is linear, `false` otherwise.
     */
    void setRhsIsLinear(bool linear);

    /**
     * @brief Sets the linear system solver to be used.
     * @param solver The name of the linear system solver as a string.
     */
    void setLinearSystemSolver(str solver);

    /**
     * @brief Sets the root finder to be used for nonlinear solvers.
     * @param finder The name of the root finder as a string.
     */
    void setRootFinder(str finder);

    /**
     * @brief Sets the linear system representation for implicit solvers.
     * @param system The `LinearSystem` object to use.
     */
    void setRhsSystem(LinearSystem system);

    // Overrides

    /**
     * @brief Sets the right-hand side function for the ODE system.
     *
     * Determines if the provided right-hand side function is linear,
     * and configures the solver accordingly.
     *
     * @param f The right-hand side function \( f(y, t) \).
     */
    void setRightHandSide(const f_TYPE& f) override;

    /**
     * @brief Advances the solution for linear systems using an implicit step.
     * @param y Current solution vector.
     * @param t Current time.
     * @return The solution vector at the next time step.
     */
    virtual Eigen::VectorXd calcLinStep(const Eigen::VectorXd y, double t) = 0;

    /**
     * @brief Advances the solution for nonlinear systems using an implicit step.
     * @param y Current solution vector.
     * @param t Current time.
     * @return The solution vector at the next time step.
     */
    Eigen::VectorXd calcNonLinStep(const Eigen::VectorXd y, double t);

    /**
     * @brief Creates a function \( F \) for a nonlinear step.
     *
     * This method is abstract and must be implemented by derived classes.
     *
     * @param y0 Initial solution vector.
     * @param t0 Initial time.
     * @return A function \( F(y) \) for root-finding or solving.
     */
    virtual F_TYPE makeFstep(Eigen::VectorXd y0, double t0) = 0;

    /**
     * @brief Advances the solution by one time step.
     * @param y Current solution vector.
     * @param t Current time.
     * @return The solution vector at the next time step.
     */
    Eigen::VectorXd calcStep(const Eigen::VectorXd& y, double t) override;
};

#endif // IMPLICIT_HH