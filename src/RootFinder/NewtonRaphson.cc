/**
 * @file NewtonRaphson.cc
 * @brief Implementation of the NewtonRaphson class, which provides a method to find roots of a system of equations
 *        using the Newton-Raphson iterative method. It uses different linear system solvers like Gaussian Elimination and LU decomposition.
 *
 * This class is responsible for:
 * - Constructing the Newton-Raphson solver with custom tolerances and step sizes
 * - Calculating the numerical Jacobian matrix for the system
 * - Solving the system of equations iteratively until convergence or reaching the iteration limit
 *
 * @author janzgraggen
 * @date 27/11/2024
 */

#include "NewtonRaphson.hh"
#include "../LinSysSolver/LinSysSolver.hh"
#include "../LinSysSolver/GaussElimSolve.hh"
#include "../LinSysSolver/LUSolve.hh"

using F_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&)> ;
using str = std::string;

/**
 * @brief Default constructor for NewtonRaphson class.
 *
 * Constructs the NewtonRaphson object by initializing it with the logger and the function `F_in`. It also sets
 * the default step size `dx` to 1e-6.
 *
 * @param logger_ Reference to the Logger object.
 * @param F_in The system of equations to solve.
 */
NewtonRaphson::NewtonRaphson(Logger& logger_ ,F_TYPE F_in)
    : RootFinder(logger_, F_in), dx(1e-6) {  // Default step size dx set to 1e-6
}

/**
 * @brief Parameterized constructor for NewtonRaphson class.
 *
 * Constructs the NewtonRaphson object with custom tolerance, step size `dx`, and a limit on iterations.
 *
 * @param logger_ Reference to the Logger object.
 * @param F_in The system of equations to solve.
 * @param tol Custom tolerance for convergence.
 * @param dx Custom step size for numerical differentiation.
 * @param maxIter Custom maximum number of iterations.
 */
NewtonRaphson::NewtonRaphson(Logger& logger_, F_TYPE F_in, double tol, double dx, int maxIter)
    : RootFinder(logger_, F_in), dx(dx) {
    setTolerance(tol);
    setMaxIterations(maxIter);
}

/**
 * @brief Destructor for NewtonRaphson class.
 *
 * Cleans up resources when the NewtonRaphson object is destroyed.
 */
NewtonRaphson::~NewtonRaphson() {}

/**
 * @brief Sets the step size `dx` for numerical differentiation.
 *
 * @param dx The new step size value.
 */
void NewtonRaphson::setDx(double dx) {
    this->dx = dx;
}

/**
 * @brief Sets the linear system solver to use during the Newton-Raphson iterations.
 *
 * @param linear_system_solver_in The solver to use (e.g., Gaussian Elimination or LU decomposition).
 */
void NewtonRaphson::SetLinearSystemSolver(str linear_system_solver_in) {
    linear_system_solver = linear_system_solver_in;
}

/**
 * @brief Retrieves the current step size `dx` for numerical differentiation.
 *
 * @return The step size `dx`.
 */
double NewtonRaphson::getDx() const {
    return dx;
}

/**
 * @brief Retrieves the selected linear system solver.
 *
 * @return The solver as a string representing its name (e.g., "GaussianElimination").
 */
str NewtonRaphson::GetLinearSystemSolver() const {
    return linear_system_solver;
}

/**
 * @brief Computes the numerical Jacobian matrix of a system of equations at a given state `x`.
 *
 * This method uses finite differences to approximate the Jacobian matrix by perturbing each variable in turn
 * and observing the resulting changes in the system's output.
 *
 * @param x The input vector representing the current state of the system.
 * @return The computed Jacobian matrix.
 */
Eigen::MatrixXd NewtonRaphson::NumericalJacobian(Eigen::VectorXd& x) {
    double dx = getDx();
    int n = x.size();
    Eigen::MatrixXd J(callF(x).size(), n);

    Eigen::VectorXd xPlus = x, xMinus = x;

    for (int j = 0; j < n; j++) {
        xPlus(j) += dx;
        xMinus(j) -= dx;

        Eigen::VectorXd fPlus = callF(xPlus);
        Eigen::VectorXd fMinus = callF(xMinus);

        for (int i = 0; i < fPlus.size(); i++) {
            J(i, j) = (fPlus(i) - fMinus(i)) / (2 * dx);
        }

        xPlus(j) = x(j);
        xMinus(j) = x(j);
    }

    return J;
}

/**
 * @brief Solves the system of equations using the Newton-Raphson method.
 *
 * This method iteratively updates the solution until the convergence criterion (tolerance or iteration count)
 * is met. It dynamically allocates the appropriate linear system solver based on the solver string.
 *
 * @return The computed root as an Eigen::VectorXd.
 */
Eigen::VectorXd NewtonRaphson::Solve() {
    Eigen::VectorXd x = getInitialGuess();
    Eigen::VectorXd Fx = callF(x);

    LinSysSolver* solver = nullptr;

    if (GetLinearSystemSolver() == "GaussianElimination") {
        solver = new GaussElimSolve(logger);
    } else if (GetLinearSystemSolver() == "LU") {
        solver = new LUSolve(logger);
    } else {
        logger.error("Invalid linear system solver: " + GetLinearSystemSolver());
        return x;
    }

    while (Fx.norm() > getTolerance() && getIterationCount() < getMaxIterations()) {
        Eigen::MatrixXd J = NumericalJacobian(x);

        try {
            solver->SetA(J);
            solver->SetB(-Fx);

            Eigen::VectorXd delta = solver->Solve();
            x += delta;

            Fx = callF(x);
            setIterationCount(getIterationCount() + 1);

        } catch (std::exception& e) {
            logger.error("Error during solving: " + str(e.what()));
            break;
        }
    }

    if (getIterationCount() == getMaxIterations()) {
        logger.warning("Newton-Raphson did not converge within the maximum number of iterations.");
    }

    delete solver;
    solver = nullptr;

    return x;
}
