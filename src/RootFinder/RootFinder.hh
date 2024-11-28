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
using F_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&)> ;

class RootFinder {
/* ------------------------------------------------------------------------ */
/* Members                                                                  */
/* ------------------------------------------------------------------------ */
private:
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> F; // Change to std::function

    double tolerance;             // Error tolerance
    int maxIterations;            // Maximum number of iterations
    int iterationCount;           // Number of iterations performed
    bool converged;               // Convergence flag
    std::string status;           // Status description
    Eigen::VectorXd initialGuess; // Initial guess for the root
    Eigen::VectorXd lastSolution; // Store the last solution

/* ------------------------------------------------------------------------ */
/* Methods                                                                  */
/* ------------------------------------------------------------------------ */
public:
    // Constructor
    RootFinder(F_TYPE F);
    RootFinder(F_TYPE F, double tol, int maxIter);

    // Destructor
    virtual ~RootFinder();

    // Setters
    void setTolerance(double tol);
    void setMaxIterations(int maxIter);
    void setInitialGuess(const Eigen::VectorXd& guess);
    void setIterationCount(int iter);

    // Getters
    int getIterationCount() const;
    double getTolerance() const;
    int getMaxIterations() const;
    Eigen::VectorXd getInitialGuess() const;
    Eigen::VectorXd getLastSolution() const;
    bool isConverged() const;
    std::string getStatus() const;

    // Function that calls the root function F (getter)
    Eigen::VectorXd callF(Eigen::VectorXd y1);

    // Virtual function to solve the root-finding problem (to be overridden)
    virtual Eigen::VectorXd Solve() = 0;

protected:
    // Log a step (can be overridden for custom logging)
    virtual void logStep(int iteration, const Eigen::VectorXd& currentSolution, double currentError);

};

#endif //__ROOT_FINDER__HH__
