//
// Created by janzgraggen on 27/11/2024.
//
#include "RootFinder.hh"
using F_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&)> ;

/**
 * @file RootFinder.cc
 * @brief Implementation of the RootFinder class that provides a base for solving root-finding problems.
 */

/**
 * @brief Constructor 1: Initializes the RootFinder with a function.
 * 
 * @param F_in The function to be used for finding the root. It should take an Eigen::VectorXd and return an Eigen::VectorXd.
 */
RootFinder::RootFinder(F_TYPE F_in)
    : tolerance(1e-6), maxIterations(1000), iterationCount(0), converged(false), status("Not started"), F(F_in) {
}

/**
 * @brief Constructor 2: Initializes the RootFinder with a function and custom tolerance and maximum iterations.
 * 
 * @param F_in The function to be used for finding the root.
 * @param tol Custom tolerance for the root-finding process.
 * @param maxIter Custom maximum number of iterations allowed.
 */
RootFinder::RootFinder(F_TYPE F_in, double tol, int maxIter)
    : RootFinder(F_in) {  // Delegates to the first constructor
    setTolerance(tol);    // Set custom tolerance
    setMaxIterations(maxIter);  // Set custom maximum iterations
}

/**
 * @brief Destructor for the RootFinder class.
 */
RootFinder::~RootFinder() {}


/**
 * @brief Sets the tolerance for convergence.
 * 
 * @param tol The tolerance value for the root-finding process.
 */
void RootFinder::setTolerance(double tol) {
    tolerance = tol;
}

/**
 * @brief Sets the maximum number of iterations allowed for the root-finding process.
 * 
 * @param maxIter The maximum number of iterations.
 */
void RootFinder::setMaxIterations(int maxIter) {
    maxIterations = maxIter;
}

/**
 * @brief Sets the initial guess for the root-finding process.
 * 
 * @param guess The initial guess for the root as an Eigen::VectorXd.
 */
void RootFinder::setInitialGuess(const Eigen::VectorXd& guess) {
    initialGuess = guess;
}

/**
 * @brief Sets the current iteration count.
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
 * @brief Gets the tolerance for the root-finding process.
 * 
 * @return The tolerance value.
 */
double RootFinder::getTolerance() const {
    return tolerance;
}

/**
 * @brief Gets the current iteration count.
 * 
 * @return The current number of iterations.
 */
int RootFinder::getIterationCount() const {
    return iterationCount;
}

/**
 * @brief Gets the maximum number of iterations allowed.
 * 
 * @return The maximum number of iterations.
 */
int RootFinder::getMaxIterations() const {
    return maxIterations;
}

/**
 * @brief Gets the last computed solution.
 * 
 * @return The last solution found, represented as an Eigen::VectorXd.
 */
Eigen::VectorXd RootFinder::getLastSolution() const {
    return lastSolution;
}

/**
 * @brief Gets the initial guess for the root.
 * 
 * @return The initial guess provided for the root-finding process.
 */
Eigen::VectorXd RootFinder::getInitialGuess() const {
    return initialGuess;
}

/**
 * @brief Checks whether the root-finding process has converged.
 * 
 * @return True if the process has converged, false otherwise.
 */
bool RootFinder::isConverged() const {
    return converged;
}

/**
 * @brief Gets the status of the root-finding process.
 * 
 * @return A string description of the current status (e.g., "Not started", "Converged").
 */
std::string RootFinder::getStatus() const {
    return status;
}

// ------------------------------------------------------------------------ //
// Methods for function evaluation
// ------------------------------------------------------------------------ //

/**
 * @brief Calls the function F with a given argument to evaluate the system.
 * 
 * @param y1 The input vector to the function F, representing a candidate root.
 * @return The result of calling the function F on the input vector y1.
 */
Eigen::VectorXd RootFinder::callF(Eigen::VectorXd y1) {
    return F(y1);
}

// ------------------------------------------------------------------------ //
// Logging and Output
// ------------------------------------------------------------------------ //

/**
 * @brief Logs a step in the root-finding process. This can be overridden for custom logging.
 * 
 * @param iteration The current iteration number.
 * @param currentSolution The current solution as an Eigen::VectorXd.
 * @param currentError The error in the current solution.
 */
void RootFinder::logStep(int iteration, const Eigen::VectorXd& currentSolution, double currentError) {
    // Default log output (this can be overridden in derived classes)
    std::cout << "Iteration " << iteration << ": Error = " << currentError << std::endl;
}
