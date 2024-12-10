/**
 * @file RootFinder.cc
 * @brief Implements the `RootFinder` class, which provides a base for solving root-finding problems.
 *
 * This class defines the core functionality to find the roots of a system of nonlinear equations.
 * It includes constructors, setters, getters, and methods to evaluate the system function `F`.
 * The `RootFinder` class is designed to be extended by more specialized root-finding algorithms
 * like Newton-Raphson or Bisection methods.
 *
 * @author janzgraggen
 * @date 27/11/2024
 */

#include "RootFinder.hh"

using F_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&)> ;
using str = std::string;

/**
 * @brief Constructor 1: Initializes the RootFinder with a system function.
 *
 * Sets up the root-finding process with default tolerance and iteration values.
 *
 * @param logger_ Reference to the Logger object for logging messages.
 * @param F_in The function representing the system of equations to find roots for.
 */
RootFinder::RootFinder(Logger& logger_, F_TYPE F_in)
    : logger(logger_), tolerance(1e-6), maxIterations(1000), iterationCount(0), converged(false), status("Not started"), F(F_in) {
}

/**
 * @brief Constructor 2: Initializes the RootFinder with a system function, custom tolerance, and maximum iterations.
 *
 * Allows for more control over the root-finding process by specifying the tolerance and maximum iteration count.
 *
 * @param logger_ Reference to the Logger object for logging messages.
 * @param F_in The system of equations for which roots are to be found.
 * @param tol Custom tolerance for convergence.
 * @param maxIter Custom maximum number of iterations allowed.
 */
RootFinder::RootFinder(Logger& logger_, F_TYPE F_in, double tol, int maxIter)
    : RootFinder(logger_, F_in) {  // Delegates to the first constructor
    setTolerance(tol);    // Set custom tolerance value
    setMaxIterations(maxIter);  // Set custom iteration limit
}

/**
 * @brief Destructor for cleaning up resources.
 *
 * Ensures proper cleanup of any resources allocated by the class.
 */
RootFinder::~RootFinder() {}

/**
 * @brief Sets the tolerance for convergence.
 *
 * Defines the stopping condition for the root-finding process based on the tolerance value.
 *
 * @param tol The desired tolerance value.
 */
void RootFinder::setTolerance(double tol) {
    tolerance = tol;
}

/**
 * @brief Sets the maximum number of iterations allowed during the root-finding process.
 *
 * Provides a constraint on the number of iterations to prevent infinite loops.
 *
 * @param maxIter The custom maximum iteration count.
 */
void RootFinder::setMaxIterations(int maxIter) {
    maxIterations = maxIter;
}

/**
 * @brief Sets the initial guess for the root-finding process.
 *
 * The initial guess acts as a starting point for iterative root-finding methods.
 *
 * @param guess The initial guess for the root as an Eigen::VectorXd.
 */
void RootFinder::setInitialGuess(const Eigen::VectorXd& guess) {
    initialGuess = guess;
}

/**
 * @brief Sets the current iteration count.
 *
 * Updates the iteration count after each iteration of the root-finding process.
 *
 * @param iter The current iteration count.
 */
void RootFinder::setIterationCount(int iter) {
    iterationCount = iter;
}

// ------------------------------------------------------------------------ //
// Getters
// ------------------------------------------------------------------------ //

/**
 * @brief Gets the tolerance value for convergence.
 *
 * Returns the tolerance used in convergence checks for the root-finding process.
 *
 * @return The tolerance value.
 */
double RootFinder::getTolerance() const {
    return tolerance;
}

/**
 * @brief Gets the current number of iterations executed.
 *
 * Returns the count of iterations performed in the root-finding process.
 *
 * @return The current iteration count.
 */
int RootFinder::getIterationCount() const {
    return iterationCount;
}

/**
 * @brief Gets the maximum number of iterations allowed.
 *
 * Returns the maximum iteration count set for the root-finding process.
 *
 * @return The maximum number of iterations.
 */
int RootFinder::getMaxIterations() const {
    return maxIterations;
}

/**
 * @brief Gets the last computed solution.
 *
 * Provides the most recent root computed by the root-finding process.
 *
 * @return The last computed root solution as an Eigen::VectorXd.
 */
Eigen::VectorXd RootFinder::getLastSolution() const {
    return lastSolution;
}

/**
 * @brief Gets the initial guess provided for root-finding.
 *
 * Returns the starting point provided for iterative methods to begin the search for roots.
 *
 * @return The initial guess for the root as an Eigen::VectorXd.
 */
Eigen::VectorXd RootFinder::getInitialGuess() const {
    return initialGuess;
}

/**
 * @brief Checks whether the root-finding process has converged.
 *
 * Determines whether the root-finding method has met the convergence criteria.
 *
 * @return True if the process has converged; otherwise, false.
 */
bool RootFinder::isConverged() const {
    return converged;
}

/**
 * @brief Gets the status of the root-finding process.
 *
 * Returns a description of the current state of the root-finding process (e.g., "Not started", "Converged").
 *
 * @return A string representing the status of the process.
 */
std::string RootFinder::getStatus() const {
    return status;
}

// ------------------------------------------------------------------------ //
// Methods for function evaluation
// ------------------------------------------------------------------------ //

/**
 * @brief Calls the system function F with a given candidate root.
 *
 * Evaluates the system of equations at the provided input vector.
 *
 * @param y1 The candidate root input vector to evaluate the system function.
 * @return The output of the system function `F` evaluated at `y1`.
 */
Eigen::VectorXd RootFinder::callF(Eigen::VectorXd y1) {
    return F(y1);
}