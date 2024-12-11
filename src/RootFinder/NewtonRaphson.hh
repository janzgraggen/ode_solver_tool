/**
 * @file NewtonRaphson.hh
 * @brief Defines the `NewtonRaphson` class, which provides an implementation of the Newton-Raphson method
 *        to find roots of a system of nonlinear equations. This class extends the `RootFinder` class and
 *        supports different linear system solvers such as Gaussian Elimination and LU decomposition.
 *
 * The `NewtonRaphson` class is responsible for:
 * - Performing iterative root finding using the Newton-Raphson method.
 * - Computing the Jacobian matrix numerically.
 * - Utilizing different solvers for solving linear systems depending on the user's choice.
 *
 * @author janzgraggen
 * @date 27/11/2024
 */

#ifndef __NEWTON_RAPHSON__HH__
#define __NEWTON_RAPHSON__HH__

#include "RootFinder.hh"

using F_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&)> ;
using str = std::string;

class NewtonRaphson : public RootFinder {
private:
    /** @brief Step size for numerical differentiation in the Newton-Raphson method. */
    double dx;

    /** @brief Name of the linear system solver to use for solving implicit equations. */
    str linear_system_solver;

public:
    /**
     * @brief Default constructor to initialize the NewtonRaphson solver with a given function.
     *
     * @param logger_ Reference to the Logger object.
     * @param F_in The system of equations to solve.
     */
    NewtonRaphson(Logger& logger_, F_TYPE F_in);

    /**
     * @brief Parameterized constructor to initialize the Newton-Raphson solver with custom tolerance,
     *        step size, and maximum iteration constraints.
     *
     * @param logger_ Reference to the Logger object.
     * @param F_in The system of equations to solve.
     * @param tol The tolerance for convergence.
     * @param dx The step size for numerical differentiation.
     * @param maxIter The maximum number of iterations allowed.
     */
    NewtonRaphson(Logger& logger_, F_TYPE F_in, double tol, double dx, int maxIter);

    /**
     * @brief Destructor for cleaning up resources.
     */
    virtual ~NewtonRaphson();

    /**
     * @brief Sets the step size `dx` for numerical differentiation.
     *
     * @param dx The step size to be set.
     */
    void setDx(double dx) override;

    /**
     * @brief Sets the linear system solver to be used during the Newton-Raphson iterations.
     *
     * @param solver The solver type as a string (e.g., "Gaussian Elimination" or "LU").
     */
    void SetLinearSystemSolver(str solver) override;

    /**
     * @brief Retrieves the step size `dx` for numerical differentiation.
     *
     * @return The current step size `dx`.
     */
    double getDx() const override;

    /**
     * @brief Retrieves the name of the selected linear system solver.
     *
     * @return The solver type as a string.
     */
    str GetLinearSystemSolver() const override;

    /**
     * @brief Computes the numerical Jacobian matrix for a system of nonlinear equations.
     *
     * This method uses finite differences to approximate the Jacobian matrix by perturbing each input
     * component individually and measuring the changes in system output.
     *
     * @param x The input vector representing the current state of the system.
     * @return The computed Jacobian matrix.
     */
    Eigen::MatrixXd NumericalJacobian(Eigen::VectorXd& x);

    /**
     * @brief Solves the system of equations using the Newton-Raphson iterative method.
     *
     * This overridden method applies the Newton-Raphson method until convergence is met or the maximum
     * iteration count is reached, leveraging different linear system solvers as specified.
     *
     * @return The computed root of the system as an Eigen::VectorXd.
     */
    Eigen::VectorXd Solve() override;
};

#endif // __NEWTON_RAPHSON__HH__
