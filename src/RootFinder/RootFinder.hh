/**
 * @file RootFinder.hh
 * @brief Defines the abstract `RootFinder` class used to find roots of a system of equations.
 *
 * This class serves as a base class for various root-finding algorithms. It provides essential
 * methods and properties for setting up the root-finding process, managing convergence criteria,
 * and evaluating the system of equations represented by the function `F`.
 *
 * @author janzgraggen
 * @date 27/11/2024
 */

#ifndef ROOT_FINDER__HH
#define ROOT_FINDER__HH

#include <Eigen/Dense>
#include <functional>
#include "../Logger/Logger.hh"

/**
 * @typedef F_TYPE
 * @brief Type definition for a system function that takes an Eigen vector as input and returns an Eigen vector.
 */
using F_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&)> ;
using str = std::string;

class RootFinder {
private:
    /**
     * @brief Function representing the system of equations to find roots for.
     *
     * This function is evaluated to search for roots of the system of nonlinear equations.
     */
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> F;

    /** @brief Tolerance value for convergence checks. */
    double tolerance;

    /** @brief Maximum number of iterations allowed during the root-finding process. */
    int maxIterations;

    /** @brief Current number of iterations executed. */
    int iterationCount;

    /** @brief Flag indicating whether the root-finding process has converged. */
    bool converged;

    /** @brief Description of the current status of the root-finding process. */
    std::string status;

    /** @brief Initial guess provided for the root-finding algorithms. */
    Eigen::VectorXd initialGuess;

    /** @brief Last computed solution of the root-finding process. */
    Eigen::VectorXd lastSolution;

public:
    /** @brief Logger object for logging system messages. */
    Logger logger;

    /**
     * @brief Constructor to initialize the root-finding process with a system function.
     *
     * @param logger_ Reference to the Logger instance for logging messages.
     * @param F_in The system function representing the search for roots.
     */
    RootFinder(Logger& logger_, F_TYPE F);

    /**
     * @brief Constructor to initialize the root-finding process with custom tolerance and maximum iterations.
     *
     * @param logger_ Reference to the Logger instance for logging messages.
     * @param F_in The system function representing the search for roots.
     * @param tol Custom tolerance for convergence.
     * @param maxIter Custom maximum number of iterations allowed.
     */
    RootFinder(Logger& logger_, F_TYPE F_in, double tol, int maxIter);

    /**
     * @brief Destructor for cleaning up resources (if required).
     */
    virtual ~RootFinder();

    /**
     * @brief Sets the tolerance value for the convergence check.
     *
     * @param tol The tolerance value used to determine convergence.
     */
    void setTolerance(double tol);

    /**
     * @brief Sets the maximum number of iterations allowed in the root-finding process.
     *
     * @param maxIter The custom maximum iteration count.
     */
    void setMaxIterations(int maxIter);

    /**
     * @brief Sets the initial guess for the root-finding process.
     *
     * The guess serves as a starting point for iterative root-finding methods.
     *
     * @param guess An Eigen vector representing the initial guess.
     */
    void setInitialGuess(const Eigen::VectorXd& guess);

    /**
     * @brief Sets the current iteration count of the root-finding process.
     *
     * @param iter The iteration count to be updated.
     */
    void setIterationCount(int iter);
    
    /**
     * @brief Retrieves the step size for numerical differentiation. (implemented in derived classes that require it)
     *
     * @return The current iteration count.
     */
    virtual void setDx(double dx) = 0;


    /**
     * @brief Retrieves the step size for numerical differentiation. (implemented in derived classes that require it)
     *
     * @return The current iteration count.
     */
    virtual double getDx() const = 0;

    /**
     * @brief Retrieves the number of iterations executed so far.
     *
     * @return The current iteration count.
     */
    int getIterationCount() const;

    /**
     * @brief Gets the tolerance value set for convergence checks.
     *
     * @return The convergence tolerance value.
     */
    double getTolerance() const;

    /**
     * @brief Retrieves the maximum number of iterations allowed.
     *
     * @return The custom maximum iteration count.
     */
    int getMaxIterations() const;

    /**
     * @brief Returns the initial guess provided for root-finding.
     *
     * @return An Eigen vector representing the initial guess.
     */
    Eigen::VectorXd getInitialGuess() const;

    /**
     * @brief Retrieves the last computed solution in the root-finding process.
     *
     * @return An Eigen vector representing the most recent root solution.
     */
    Eigen::VectorXd getLastSolution() const;

    /**
     * @brief Checks if the root-finding process has successfully converged.
     *
     * @return True if the process has converged; false otherwise.
     */
    bool isConverged() const;

    /**
     * @brief Returns the status of the root-finding process.
     *
     * Provides a string describing the current state (e.g., "Not started", "Converged").
     *
     * @return A description of the current status of the process.
     */
    std::string getStatus() const;

    /**
     * @brief Calls the system function F to evaluate a candidate root.
     *
     * Evaluates the system of equations with a provided input vector.
     *
     * @param y1 An Eigen vector representing the candidate root to evaluate.
     * @return The output of the system function `F` evaluated at `y1`.
     */
    Eigen::VectorXd callF(Eigen::VectorXd y1);

    /**
     * @brief Abstract method to solve the root-finding problem.
     *
     * This method must be implemented by derived classes to specify the root-finding algorithm.
     *
     * @return An Eigen vector representing the root solution.
     */
    virtual Eigen::VectorXd solveRoot() = 0;

    /**
     * @brief Sets the linear system solver.
     *
     * Allows specifying which solver to use for solving implicit methods.
     *
     * @param solver The solver type as a string.
     */
    virtual void setLinearSystemSolver(str solver) = 0;

    /**
     * @brief Gets the linear system solver currently in use.
     *
     * @return The solver type as a string.
     */
    virtual str getLinearSystemSolver() const = 0;
};

#endif // ROOT_FINDER__HH

