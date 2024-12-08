//
// Created by janzgraggen on 27/11/2024.
//
#ifndef __ROOT_FINDER__HH__
#define __ROOT_FINDER__HH__

#include <Eigen/Dense>
#include <functional>
#include <iostream>
#include <cmath> 
#include <stdexcept>
#include "../Logger/Logger.hh"

/**
 * @file RootFinder.hh
 * @brief Defines the abstract `RootFinder` class used to find roots of a system of equations.
 */

/**
 * @typedef F_TYPE
 * @brief Type definition for a function that takes an Eigen vector as input and returns an Eigen vector.
 */
using F_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&)> ;

class RootFinder {
/* ------------------------------------------------------------------------ */
/* Members                                                                  */
/* ------------------------------------------------------------------------ */
private:
    /** 
     * @brief Function used to find the root. 
     * 
     * This function represents the system of equations for which we are searching for a root.
     */
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> F;

    /** @brief Tolerance for convergence. */
    double tolerance;

    /** @brief Maximum number of iterations allowed. */
    int maxIterations;

    /** @brief Current number of iterations performed. */
    int iterationCount;

    /** @brief Flag to indicate if the root-finding process has converged. */
    bool converged;

    /** @brief Status description for the root-finding process. */
    std::string status;

    /** @brief Initial guess for the root of the equation. */
    Eigen::VectorXd initialGuess;

    /** @brief Last computed solution. */
    Eigen::VectorXd lastSolution;


/* ------------------------------------------------------------------------ */
/* Methods                                                                  */
/* ------------------------------------------------------------------------ */
public:
    /** @brief Logger object for logging messages. */
    Logger logger;
    /**
     * @brief Constructor to initialize the root-finding process with a given function.
     * 
     * @param F Function to find the root of.
     */
    RootFinder(Logger& logger_,F_TYPE F);

    /**
     * @brief Constructor to initialize the root-finding process with a given function, tolerance, and maximum iterations.
     * 
     * @param F Function to find the root of.
     * @param tol The tolerance for the solution.
     * @param maxIter The maximum number of iterations allowed.
     */
    RootFinder(Logger& logger_,F_TYPE F, double tol, int maxIter);

    /** 
     * @brief Destructor for cleaning up resources (if necessary).
     */
    virtual ~RootFinder();

    /** 
     * @brief Sets the tolerance for convergence.
     * 
     * @param tol The tolerance value.
     */
    void setTolerance(double tol);

    /** 
     * @brief Sets the maximum number of iterations.
     * 
     * @param maxIter The maximum number of iterations.
     */
    void setMaxIterations(int maxIter);

    /** 
     * @brief Sets the initial guess for the root.
     * 
     * @param guess The initial guess as an Eigen::VectorXd.
     */
    void setInitialGuess(const Eigen::VectorXd& guess);

    /** 
     * @brief Sets the current iteration count.
     * 
     * @param iter The current iteration count.
     */
    void setIterationCount(int iter);

    /** 
     * @brief Gets the current iteration count.
     * 
     * @return The current iteration count.
     */
    int getIterationCount() const;

    /** 
     * @brief Gets the tolerance for convergence.
     * 
     * @return The tolerance value.
     */
    double getTolerance() const;

    /** 
     * @brief Gets the maximum number of iterations allowed.
     * 
     * @return The maximum number of iterations.
     */
    int getMaxIterations() const;

    /** 
     * @brief Gets the initial guess for the root.
     * 
     * @return The initial guess as an Eigen::VectorXd.
     */
    Eigen::VectorXd getInitialGuess() const;

    /** 
     * @brief Gets the last computed solution.
     * 
     * @return The last solution found as an Eigen::VectorXd.
     */
    Eigen::VectorXd getLastSolution() const;

    /** 
     * @brief Checks if the root-finding process has converged.
     * 
     * @return True if converged, false otherwise.
     */
    bool isConverged() const;

    /** 
     * @brief Gets the status description of the root-finding process.
     * 
     * @return A string describing the current status.
     */
    std::string getStatus() const;

    /**
     * @brief Calls the function F with a given argument to evaluate the system.
     * 
     * @param y1 Input to the function F, representing a candidate root.
     * @return The result of the function F evaluated at `y1`.
     */
    Eigen::VectorXd callF(Eigen::VectorXd y1);

    /**
     * @brief Virtual method to solve the root-finding problem. 
     * 
     * This method must be overridden in derived classes to implement specific root-finding algorithms.
     * 
     * @return The solution to the root-finding problem as an Eigen::VectorXd.
     */
    virtual Eigen::VectorXd Solve() = 0;

protected:
    /** 
     * @brief Logs a step in the root-finding process.
     * 
     * This method can be overridden to provide custom logging of each step of the iteration.
     * 
     * @param iteration The current iteration number.
     * @param currentSolution The current solution as an Eigen::VectorXd.
     * @param currentError The error in the current solution.
     */
    virtual void logStep(int iteration, const Eigen::VectorXd& currentSolution, double currentError);

};

#endif //__ROOT_FINDER__HH__
